/*
 * tmc6300.c -- TMC6300 three-phase BLDC gate driver (3 PWM inputs).
 */

#include "tmc6300.h"
#include "driver/gpio.h"

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
                       int ul_gpio, int vl_gpio, int wl_gpio,
                       ledc_channel_t ch_base,
                       uint32_t freq_hz, ledc_timer_bit_t resolution,
                       int standby_gpio)
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
    drv->standby_gpio = standby_gpio;

    /* Drive UL/VL/WL HIGH so the low-side FETs are always on.
     * This enables 3-PWM mode: high-side PWM + low-side always
     * conducting, completing the current path through each phase. */
    {
        const int low_gpios[] = { ul_gpio, vl_gpio, wl_gpio };
        for (int i = 0; i < 3; i++) {
            const gpio_config_t lo_cfg = {
                .pin_bit_mask = 1ULL << low_gpios[i],
                .mode         = GPIO_MODE_OUTPUT,
                .pull_up_en   = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type    = GPIO_INTR_DISABLE,
            };
            err = gpio_config(&lo_cfg);
            if (err != ESP_OK) return err;
            err = gpio_set_level(low_gpios[i], 1);
            if (err != ESP_OK) return err;
        }
    }

    /* If a STANDBY GPIO is provided, configure it as output and
     * drive HIGH to take the TMC6300 out of standby (active).      */
    if (standby_gpio >= 0) {
        const gpio_config_t stby_cfg = {
            .pin_bit_mask = 1ULL << standby_gpio,
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        err = gpio_config(&stby_cfg);
        if (err != ESP_OK) return err;
        err = gpio_set_level(standby_gpio, 1);
        if (err != ESP_OK) return err;
    }

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

esp_err_t tmc6300_standby(const tmc6300_t *drv)
{
    esp_err_t err = tmc6300_coast(drv);
    if (err != ESP_OK) return err;
    if (drv->standby_gpio >= 0) {
        return gpio_set_level(drv->standby_gpio, 0);
    }
    return ESP_OK;
}

esp_err_t tmc6300_wake(const tmc6300_t *drv)
{
    if (drv->standby_gpio >= 0) {
        return gpio_set_level(drv->standby_gpio, 1);
    }
    return ESP_OK;
}
