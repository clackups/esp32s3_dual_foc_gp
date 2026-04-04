/*
 * tmc6300.c -- TMC6300 three-phase BLDC gate driver (3 PWM inputs).
 *
 * Uses the ESP32-S3 MCPWM peripheral for center-aligned PWM with
 * synchronized duty updates across all three motor phases.
 */

#include "tmc6300.h"
#include "driver/mcpwm_prelude.h"

/* MCPWM timer resolution.  80 MHz gives a peak count of 2000 at
 * 20 kHz (period_ticks = 4000, peak = 2000), providing ~11-bit
 * equivalent duty resolution.                                         */
#define MCPWM_RESOLUTION_HZ  80000000U

/* ------------------------------------------------------------------ */
/* Helper: create one operator + comparator + generator for a single  */
/* motor phase, connect it to the shared timer, and configure         */
/* center-aligned PWM actions.                                        */
/*                                                                    */
/* The generator output is forced HIGH during setup so that the       */
/* TMC6300's high-side FET stays on (phase at Vmotor) until the PWM   */
/* waveform takes over.  Without this, the default-LOW output would   */
/* turn on the always-enabled low-side FET, shorting the phase to GND */
/* and potentially triggering the TMC6300's fault protection.         */
/* The caller must release the force level after the timer starts     */
/* (see tmc6300_init).                                                */
/* ------------------------------------------------------------------ */
static esp_err_t init_phase(int group_id, mcpwm_timer_handle_t timer,
                            int gpio, uint32_t init_cmp,
                            mcpwm_cmpr_handle_t *out_cmpr,
                            mcpwm_gen_handle_t *out_gen)
{
    /* Operator */
    mcpwm_operator_config_t oper_cfg = { .group_id = group_id };
    mcpwm_oper_handle_t oper;
    esp_err_t err = mcpwm_new_operator(&oper_cfg, &oper);
    if (err != ESP_OK) return err;

    err = mcpwm_operator_connect_timer(oper, timer);
    if (err != ESP_OK) return err;

    /* Comparator -- update on timer-equals-zero (trough) so all three
     * phases switch to their new duties at the same instant.          */
    mcpwm_comparator_config_t cmpr_cfg = {
        .flags.update_cmp_on_tez = true,
    };
    err = mcpwm_new_comparator(oper, &cmpr_cfg, out_cmpr);
    if (err != ESP_OK) return err;

    err = mcpwm_comparator_set_compare_value(*out_cmpr, init_cmp);
    if (err != ESP_OK) return err;

    /* Generator -- output on the GPIO */
    mcpwm_generator_config_t gen_cfg = {
        .gen_gpio_num = gpio,
    };
    mcpwm_gen_handle_t gen;
    err = mcpwm_new_generator(oper, &gen_cfg, &gen);
    if (err != ESP_OK) return err;

    /* Force the output HIGH immediately.  MCPWM generators default to
     * LOW, which with UL/VL/WL hardwired HIGH would turn on the
     * low-side FET and short the phase to GND.  The force level is
     * released by tmc6300_init() after the timer is running.          */
    err = mcpwm_generator_set_force_level(gen, 1, true);
    if (err != ESP_OK) return err;

    /* Center-aligned PWM waveform:
     *
     *   - At the trough (count = 0, direction UP): set HIGH
     *   - When the timer counts UP past the compare:  set LOW
     *   - When the timer counts DOWN past the compare: set HIGH
     *
     * The trough action ensures correct initialisation of the output
     * level on the very first PWM cycle after the force is released.
     *
     * This produces a symmetric pulse centered on the timer trough.
     * Duty = compare_value / peak.
     *
     *   compare = 0      ->   0 % duty (always LOW)
     *   compare = peak/2 ->  50 % duty
     *   compare = peak   -> 100 % duty (always HIGH)          */
    err = mcpwm_generator_set_action_on_timer_event(gen,
              MCPWM_GEN_TIMER_EVENT_ACTION(
                  MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
                  MCPWM_GEN_ACTION_HIGH));
    if (err != ESP_OK) return err;

    err = mcpwm_generator_set_action_on_compare_event(gen,
              MCPWM_GEN_COMPARE_EVENT_ACTION(
                  MCPWM_TIMER_DIRECTION_UP, *out_cmpr,
                  MCPWM_GEN_ACTION_LOW));
    if (err != ESP_OK) return err;

    err = mcpwm_generator_set_action_on_compare_event(gen,
              MCPWM_GEN_COMPARE_EVENT_ACTION(
                  MCPWM_TIMER_DIRECTION_DOWN, *out_cmpr,
                  MCPWM_GEN_ACTION_HIGH));
    if (err != ESP_OK) return err;

    *out_gen = gen;
    return ESP_OK;
}

esp_err_t tmc6300_init(tmc6300_t *drv, int mcpwm_group,
                       int uh_gpio, int vh_gpio, int wh_gpio,
                       uint32_t freq_hz)
{
    /* Center-aligned (UP_DOWN) timer.  For UP_DOWN mode, period_ticks
     * is the total number of ticks in one complete PWM cycle.  The
     * timer counts from 0 to peak (= period_ticks / 2) and back to 0.
     *
     * period_ticks = resolution_hz / freq_hz
     * peak         = period_ticks / 2  (maximum comparator value)     */
    uint32_t period_ticks = MCPWM_RESOLUTION_HZ / freq_hz;
    drv->max_duty = period_ticks / 2;

    mcpwm_timer_config_t timer_cfg = {
        .group_id      = mcpwm_group,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MCPWM_RESOLUTION_HZ,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
        .period_ticks  = period_ticks,
    };
    mcpwm_timer_handle_t timer;
    esp_err_t err = mcpwm_new_timer(&timer_cfg, &timer);
    if (err != ESP_OK) return err;

    /* UL/VL/WL and VIO are hardwired to +3.3 V on the PCB, so no GPIO
     * setup is needed for low-side enables or standby control.
     *
     * Start every channel at 50 % duty (the neutral, zero-torque
     * point).  With the low-side FETs always enabled, duty = 0 would
     * short all three phases to GND simultaneously, which can trigger
     * the TMC6300's overcurrent/fault protection and latch the driver
     * into a disabled state.  50 % keeps each phase at Vmotor / 2 on
     * average, so no current flows and no fault is triggered.
     *
     * Each generator's output is forced HIGH during setup (see
     * init_phase) and released below after the timer is running, so
     * the TMC6300 never sees a LOW transient on any UH/VH/WH pin.    */
    uint32_t half = drv->max_duty / 2;

    mcpwm_cmpr_handle_t cu, cv, cw;
    mcpwm_gen_handle_t  gu, gv, gw;
    err = init_phase(mcpwm_group, timer, uh_gpio, half, &cu, &gu);
    if (err != ESP_OK) return err;
    err = init_phase(mcpwm_group, timer, vh_gpio, half, &cv, &gv);
    if (err != ESP_OK) return err;
    err = init_phase(mcpwm_group, timer, wh_gpio, half, &cw, &gw);
    if (err != ESP_OK) return err;

    drv->cmpr_u = cu;
    drv->cmpr_v = cv;
    drv->cmpr_w = cw;

    /* Enable and start the timer -- PWM output begins immediately.   */
    err = mcpwm_timer_enable(timer);
    if (err != ESP_OK) return err;

    err = mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
    if (err != ESP_OK) return err;

    /* Release the force level on all three generators.  The timer is
     * now running so the TEZ action (set HIGH at trough) and the
     * compare actions produce a correct center-aligned waveform from
     * the very first cycle.  There is no LOW glitch on any output.   */
    err = mcpwm_generator_set_force_level(gu, -1, true);
    if (err != ESP_OK) return err;
    err = mcpwm_generator_set_force_level(gv, -1, true);
    if (err != ESP_OK) return err;
    return mcpwm_generator_set_force_level(gw, -1, true);
}

esp_err_t tmc6300_set_three_phase(const tmc6300_t *drv,
                                  uint32_t duty_u, uint32_t duty_v,
                                  uint32_t duty_w)
{
    if (duty_u > drv->max_duty) duty_u = drv->max_duty;
    if (duty_v > drv->max_duty) duty_v = drv->max_duty;
    if (duty_w > drv->max_duty) duty_w = drv->max_duty;

    /* All three values are written to shadow registers and take effect
     * together at the next timer trough (update_cmp_on_tez = true).  */
    esp_err_t err;
    err = mcpwm_comparator_set_compare_value(drv->cmpr_u, duty_u);
    if (err != ESP_OK) return err;
    err = mcpwm_comparator_set_compare_value(drv->cmpr_v, duty_v);
    if (err != ESP_OK) return err;
    return mcpwm_comparator_set_compare_value(drv->cmpr_w, duty_w);
}

esp_err_t tmc6300_coast(const tmc6300_t *drv)
{
    /* Set all phases to 50 % duty (Vmotor / 2 average on each phase).
     * No inter-phase current flows, no overcurrent fault risk.       */
    uint32_t half = drv->max_duty / 2;
    return tmc6300_set_three_phase(drv, half, half, half);
}
