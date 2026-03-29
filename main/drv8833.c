/*
 * drv8833.c — DRV8833 dual-H-bridge motor driver (LEDC PWM).
 */

#include "drv8833.h"

static esp_err_t init_channel(ledc_channel_t ch, ledc_timer_t timer,
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

esp_err_t drv8833_init(drv8833_t *drv, ledc_timer_t timer,
                       int ain1_gpio, int ain2_gpio,
                       int bin1_gpio, int bin2_gpio,
                       ledc_channel_t ch_base,
                       uint32_t freq_hz, ledc_timer_bit_t resolution)
{
    /* Configure the shared timer once. */
    const ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = timer,
        .duty_resolution = resolution,
        .freq_hz         = freq_hz,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) return err;

    drv->ch_a1    = ch_base;
    drv->ch_a2    = ch_base + 1;
    drv->ch_b1    = ch_base + 2;
    drv->ch_b2    = ch_base + 3;
    drv->max_duty = (1U << resolution) - 1;

    err = init_channel(drv->ch_a1, timer, ain1_gpio);
    if (err != ESP_OK) return err;
    err = init_channel(drv->ch_a2, timer, ain2_gpio);
    if (err != ESP_OK) return err;
    err = init_channel(drv->ch_b1, timer, bin1_gpio);
    if (err != ESP_OK) return err;
    return init_channel(drv->ch_b2, timer, bin2_gpio);
}

static esp_err_t set_duty(ledc_channel_t ch, uint32_t duty)
{
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    if (err != ESP_OK) return err;
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

esp_err_t drv8833_set_phase(const drv8833_t *drv,
                            int32_t phase_a, int32_t phase_b)
{
    uint32_t a1 = 0, a2 = 0, b1 = 0, b2 = 0;

    if (phase_a >= 0) {
        a1 = (uint32_t)phase_a;
    } else {
        a2 = (uint32_t)(-phase_a);
    }
    if (phase_b >= 0) {
        b1 = (uint32_t)phase_b;
    } else {
        b2 = (uint32_t)(-phase_b);
    }

    if (a1 > drv->max_duty) a1 = drv->max_duty;
    if (a2 > drv->max_duty) a2 = drv->max_duty;
    if (b1 > drv->max_duty) b1 = drv->max_duty;
    if (b2 > drv->max_duty) b2 = drv->max_duty;

    esp_err_t err;
    err = set_duty(drv->ch_a1, a1); if (err != ESP_OK) return err;
    err = set_duty(drv->ch_a2, a2); if (err != ESP_OK) return err;
    err = set_duty(drv->ch_b1, b1); if (err != ESP_OK) return err;
    return set_duty(drv->ch_b2, b2);
}

esp_err_t drv8833_coast(const drv8833_t *drv)
{
    return drv8833_set_phase(drv, 0, 0);
}
