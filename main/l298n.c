/*
 * l298n.c — Mini L298N motor driver (3-phase sinusoidal PWM).
 */

#include "l298n.h"

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

esp_err_t l298n_init(l298n_t *drv, ledc_timer_t timer,
                     int in1_gpio, int in2_gpio, int in3_gpio,
                     ledc_channel_t ch_base,
                     uint32_t freq_hz, ledc_timer_bit_t resolution)
{
    /* Configure the LEDC timer for this Mini L298N instance. */
    const ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = timer,
        .duty_resolution = resolution,
        .freq_hz         = freq_hz,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) return err;

    drv->ch_in1   = ch_base;
    drv->ch_in2   = ch_base + 1;
    drv->ch_in3   = ch_base + 2;
    drv->max_duty = (1U << resolution) - 1;

    /* PWM on IN1–IN3 (three motor phases). */
    err = init_pwm_channel(drv->ch_in1, timer, in1_gpio);
    if (err != ESP_OK) return err;
    err = init_pwm_channel(drv->ch_in2, timer, in2_gpio);
    if (err != ESP_OK) return err;
    return init_pwm_channel(drv->ch_in3, timer, in3_gpio);
}

static esp_err_t set_duty(ledc_channel_t ch, uint32_t duty)
{
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    if (err != ESP_OK) return err;
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

esp_err_t l298n_set_three_phase(const l298n_t *drv,
                                uint32_t duty_u, uint32_t duty_v,
                                uint32_t duty_w)
{
    if (duty_u > drv->max_duty) duty_u = drv->max_duty;
    if (duty_v > drv->max_duty) duty_v = drv->max_duty;
    if (duty_w > drv->max_duty) duty_w = drv->max_duty;

    esp_err_t err;
    err = set_duty(drv->ch_in1, duty_u);
    if (err != ESP_OK) return err;
    err = set_duty(drv->ch_in2, duty_v);
    if (err != ESP_OK) return err;
    return set_duty(drv->ch_in3, duty_w);
}

esp_err_t l298n_coast(const l298n_t *drv)
{
    esp_err_t err;
    err = set_duty(drv->ch_in1, 0);
    if (err != ESP_OK) return err;
    err = set_duty(drv->ch_in2, 0);
    if (err != ESP_OK) return err;
    return set_duty(drv->ch_in3, 0);
}
