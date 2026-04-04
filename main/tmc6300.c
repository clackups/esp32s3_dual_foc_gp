/*
 * tmc6300.c -- TMC6300 three-phase BLDC gate driver (3 PWM inputs).
 */

#include "tmc6300.h"

static esp_err_t init_pwm_channel(ledc_channel_t ch, ledc_timer_t timer,
                                  int gpio)
{
    const ledc_channel_config_t cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = ch,
        .timer_sel  = timer,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = gpio,
        .duty       = 0,
        .hpoint     = 0,
    };
    return ledc_channel_config(&cfg);
}

esp_err_t tmc6300_init(tmc6300_t *drv, ledc_timer_t timer,
                       int uh_gpio, int vh_gpio, int wh_gpio,
                       ledc_channel_t ch_base,
                       uint32_t freq_hz, ledc_timer_bit_t resolution)
{
    /* Configure the LEDC timer for this TMC6300 instance. */
    const ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = timer,
        .duty_resolution = resolution,
        .freq_hz         = freq_hz,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) return err;

    drv->ch_u     = ch_base;
    drv->ch_v     = ch_base + 1;
    drv->ch_w     = ch_base + 2;
    drv->max_duty = (1U << resolution) - 1;

    /* UL/VL/WL and VIO are hardwired to +3.3 V on the PCB, so no GPIO
     * setup is needed for low-side enables or standby control. */

    /* PWM on UH, VH, WH (three motor phases). */
    err = init_pwm_channel(drv->ch_u, timer, uh_gpio);
    if (err != ESP_OK) return err;
    err = init_pwm_channel(drv->ch_v, timer, vh_gpio);
    if (err != ESP_OK) return err;
    return init_pwm_channel(drv->ch_w, timer, wh_gpio);
}

static esp_err_t set_duty(ledc_channel_t ch, uint32_t duty)
{
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    if (err != ESP_OK) return err;
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

esp_err_t tmc6300_set_three_phase(const tmc6300_t *drv,
                                  uint32_t duty_u, uint32_t duty_v,
                                  uint32_t duty_w)
{
    if (duty_u > drv->max_duty) duty_u = drv->max_duty;
    if (duty_v > drv->max_duty) duty_v = drv->max_duty;
    if (duty_w > drv->max_duty) duty_w = drv->max_duty;

    esp_err_t err;
    err = set_duty(drv->ch_u, duty_u);
    if (err != ESP_OK) return err;
    err = set_duty(drv->ch_v, duty_v);
    if (err != ESP_OK) return err;
    return set_duty(drv->ch_w, duty_w);
}

esp_err_t tmc6300_coast(const tmc6300_t *drv)
{
    esp_err_t err;
    err = set_duty(drv->ch_u, 0);
    if (err != ESP_OK) return err;
    err = set_duty(drv->ch_v, 0);
    if (err != ESP_OK) return err;
    return set_duty(drv->ch_w, 0);
}
