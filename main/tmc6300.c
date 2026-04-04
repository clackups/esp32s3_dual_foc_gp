/*
 * tmc6300.c -- TMC6300 three-phase BLDC gate driver (3 PWM inputs).
 *
 * Uses the ESP32-S3 MCPWM peripheral for center-aligned PWM with
 * synchronized duty updates across all three motor phases.
 */

#include "tmc6300.h"
#include "driver/mcpwm_prelude.h"

/* MCPWM timer resolution.  80 MHz gives 2000 ticks per half-cycle at
 * 20 kHz, providing ~11-bit equivalent duty resolution.               */
#define MCPWM_RESOLUTION_HZ  80000000U

/* ------------------------------------------------------------------ */
/* Helper: create one operator + comparator + generator for a single  */
/* motor phase, connect it to the shared timer, and configure         */
/* center-aligned PWM actions.                                        */
/* ------------------------------------------------------------------ */
static esp_err_t init_phase(int group_id, mcpwm_timer_handle_t timer,
                            int gpio, uint32_t init_cmp,
                            mcpwm_cmpr_handle_t *out_cmpr)
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

    /* Center-aligned PWM waveform:
     *   - Output goes LOW  when the timer counts UP past the compare
     *   - Output goes HIGH when the timer counts DOWN past the compare
     *
     * This produces a symmetric pulse centred on the timer trough
     * (count = 0).  Duty = compare_value / period_ticks.
     *
     *   compare = 0            ->   0 % duty (always LOW)
     *   compare = period / 2   ->  50 % duty
     *   compare = period       -> 100 % duty (always HIGH)          */
    err = mcpwm_generator_set_action_on_compare_event(gen,
              MCPWM_GEN_COMPARE_EVENT_ACTION(
                  MCPWM_TIMER_DIRECTION_UP, *out_cmpr,
                  MCPWM_GEN_ACTION_LOW));
    if (err != ESP_OK) return err;

    return mcpwm_generator_set_action_on_compare_event(gen,
              MCPWM_GEN_COMPARE_EVENT_ACTION(
                  MCPWM_TIMER_DIRECTION_DOWN, *out_cmpr,
                  MCPWM_GEN_ACTION_HIGH));
}

esp_err_t tmc6300_init(tmc6300_t *drv, int mcpwm_group,
                       int uh_gpio, int vh_gpio, int wh_gpio,
                       uint32_t freq_hz)
{
    /* Center-aligned (UP_DOWN) timer.  One full PWM cycle counts
     * 0 -> period_ticks -> 0, so the cycle time is
     * 2 * period_ticks / resolution_hz.
     *
     * period_ticks = resolution_hz / (2 * freq_hz)                   */
    uint32_t period_ticks = MCPWM_RESOLUTION_HZ / (2 * freq_hz);
    drv->max_duty = period_ticks;

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
     * average, so no current flows and no fault is triggered.         */
    uint32_t half = period_ticks / 2;

    mcpwm_cmpr_handle_t cu, cv, cw;
    err = init_phase(mcpwm_group, timer, uh_gpio, half, &cu);
    if (err != ESP_OK) return err;
    err = init_phase(mcpwm_group, timer, vh_gpio, half, &cv);
    if (err != ESP_OK) return err;
    err = init_phase(mcpwm_group, timer, wh_gpio, half, &cw);
    if (err != ESP_OK) return err;

    drv->cmpr_u = cu;
    drv->cmpr_v = cv;
    drv->cmpr_w = cw;

    /* Enable and start the timer -- PWM output begins immediately.   */
    err = mcpwm_timer_enable(timer);
    if (err != ESP_OK) return err;

    return mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
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
