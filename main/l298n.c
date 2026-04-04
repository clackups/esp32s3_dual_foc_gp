/*
 * l298n.c -- Mini L298N motor driver (3-phase sinusoidal PWM via MCPWM).
 */

#include "l298n.h"

/* MCPWM timer resolution -- 80 MHz gives sub-microsecond duty steps. */
#define MCPWM_RESOLUTION_HZ  80000000

/*
 * Create one MCPWM operator + comparator + generator for a single
 * motor phase, and configure center-aligned PWM actions.
 */
static esp_err_t init_phase(int group_id, mcpwm_timer_handle_t timer,
                            int gpio, mcpwm_cmpr_handle_t *out_cmp)
{
    /* Operator -------------------------------------------------------- */
    const mcpwm_operator_config_t oper_cfg = {
        .group_id = group_id,
    };
    mcpwm_oper_handle_t oper;
    esp_err_t err = mcpwm_new_operator(&oper_cfg, &oper);
    if (err != ESP_OK) return err;

    err = mcpwm_operator_connect_timer(oper, timer);
    if (err != ESP_OK) return err;

    /* Comparator ------------------------------------------------------ */
    const mcpwm_comparator_config_t cmp_cfg = {
        .flags.update_cmp_on_tez = true,   /* sync all phases at trough */
    };
    mcpwm_cmpr_handle_t cmp;
    err = mcpwm_new_comparator(oper, &cmp_cfg, &cmp);
    if (err != ESP_OK) return err;

    err = mcpwm_comparator_set_compare_value(cmp, 0);
    if (err != ESP_OK) return err;

    /* Generator ------------------------------------------------------- */
    const mcpwm_generator_config_t gen_cfg = {
        .gen_gpio_num = gpio,
    };
    mcpwm_gen_handle_t gen;
    err = mcpwm_new_generator(oper, &gen_cfg, &gen);
    if (err != ESP_OK) return err;

    /* Center-aligned PWM actions:
     *   count == 0 (trough): set HIGH
     *   count up == compare:  set LOW
     *   count down == compare: set HIGH                                */
    err = mcpwm_generator_set_action_on_timer_event(gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH));
    if (err != ESP_OK) return err;

    err = mcpwm_generator_set_action_on_compare_event(gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       cmp, MCPWM_GEN_ACTION_LOW));
    if (err != ESP_OK) return err;

    err = mcpwm_generator_set_action_on_compare_event(gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN,
                                       cmp, MCPWM_GEN_ACTION_HIGH));
    if (err != ESP_OK) return err;

    *out_cmp = cmp;
    return ESP_OK;
}

esp_err_t l298n_init(l298n_t *drv, int group_id,
                     int in1_gpio, int in2_gpio, int in3_gpio,
                     uint32_t freq_hz)
{
    /* Timer (center-aligned, UP_DOWN counting) ----------------------- */
    uint32_t period_ticks = MCPWM_RESOLUTION_HZ / freq_hz;
    const mcpwm_timer_config_t timer_cfg = {
        .group_id      = group_id,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MCPWM_RESOLUTION_HZ,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
        .period_ticks  = period_ticks,
    };
    mcpwm_timer_handle_t timer;
    esp_err_t err = mcpwm_new_timer(&timer_cfg, &timer);
    if (err != ESP_OK) return err;

    drv->max_duty = period_ticks / 2;  /* peak ticks */

    /* Three phases ---------------------------------------------------- */
    err = init_phase(group_id, timer, in1_gpio, &drv->cmp_u);
    if (err != ESP_OK) return err;
    err = init_phase(group_id, timer, in2_gpio, &drv->cmp_v);
    if (err != ESP_OK) return err;
    err = init_phase(group_id, timer, in3_gpio, &drv->cmp_w);
    if (err != ESP_OK) return err;

    /* Enable and start the timer ------------------------------------- */
    err = mcpwm_timer_enable(timer);
    if (err != ESP_OK) return err;
    return mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
}

esp_err_t l298n_set_three_phase(const l298n_t *drv,
                                uint32_t duty_u, uint32_t duty_v,
                                uint32_t duty_w)
{
    if (duty_u > drv->max_duty) duty_u = drv->max_duty;
    if (duty_v > drv->max_duty) duty_v = drv->max_duty;
    if (duty_w > drv->max_duty) duty_w = drv->max_duty;

    esp_err_t err;
    err = mcpwm_comparator_set_compare_value(drv->cmp_u, duty_u);
    if (err != ESP_OK) return err;
    err = mcpwm_comparator_set_compare_value(drv->cmp_v, duty_v);
    if (err != ESP_OK) return err;
    return mcpwm_comparator_set_compare_value(drv->cmp_w, duty_w);
}

esp_err_t l298n_coast(const l298n_t *drv)
{
    esp_err_t err;
    err = mcpwm_comparator_set_compare_value(drv->cmp_u, 0);
    if (err != ESP_OK) return err;
    err = mcpwm_comparator_set_compare_value(drv->cmp_v, 0);
    if (err != ESP_OK) return err;
    return mcpwm_comparator_set_compare_value(drv->cmp_w, 0);
}
