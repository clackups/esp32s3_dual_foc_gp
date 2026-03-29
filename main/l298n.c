/*
 * l298n.c — L298N dual-H-bridge motor driver (LEDC PWM + GPIO).
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

static esp_err_t init_dir_gpio(int gpio)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << gpio,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) return err;
    return gpio_set_level(gpio, 0);
}

esp_err_t l298n_init(l298n_t *drv, ledc_timer_t timer,
                     int ena_gpio, int in1_gpio, int in2_gpio,
                     int enb_gpio, int in3_gpio, int in4_gpio,
                     ledc_channel_t ch_base,
                     uint32_t freq_hz, ledc_timer_bit_t resolution)
{
    /* Configure the LEDC timer for this L298N instance. */
    const ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = timer,
        .duty_resolution = resolution,
        .freq_hz         = freq_hz,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) return err;

    drv->ch_ena   = ch_base;
    drv->ch_enb   = ch_base + 1;
    drv->in1_gpio = (gpio_num_t)in1_gpio;
    drv->in2_gpio = (gpio_num_t)in2_gpio;
    drv->in3_gpio = (gpio_num_t)in3_gpio;
    drv->in4_gpio = (gpio_num_t)in4_gpio;
    drv->max_duty = (1U << resolution) - 1;

    /* PWM on enable pins. */
    err = init_pwm_channel(drv->ch_ena, timer, ena_gpio);
    if (err != ESP_OK) return err;
    err = init_pwm_channel(drv->ch_enb, timer, enb_gpio);
    if (err != ESP_OK) return err;

    /* Direction GPIOs. */
    err = init_dir_gpio(in1_gpio);
    if (err != ESP_OK) return err;
    err = init_dir_gpio(in2_gpio);
    if (err != ESP_OK) return err;
    err = init_dir_gpio(in3_gpio);
    if (err != ESP_OK) return err;
    return init_dir_gpio(in4_gpio);
}

static esp_err_t set_duty(ledc_channel_t ch, uint32_t duty)
{
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    if (err != ESP_OK) return err;
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

esp_err_t l298n_set_phase(const l298n_t *drv,
                          int32_t phase_a, int32_t phase_b)
{
    uint32_t duty_a, duty_b;

    /* H-bridge A (coil U): direction via IN1/IN2, speed via ENA. */
    if (phase_a >= 0) {
        duty_a = (uint32_t)phase_a;
        gpio_set_level(drv->in1_gpio, 1);
        gpio_set_level(drv->in2_gpio, 0);
    } else {
        duty_a = (uint32_t)(-phase_a);
        gpio_set_level(drv->in1_gpio, 0);
        gpio_set_level(drv->in2_gpio, 1);
    }

    /* H-bridge B (coil V): direction via IN3/IN4, speed via ENB. */
    if (phase_b >= 0) {
        duty_b = (uint32_t)phase_b;
        gpio_set_level(drv->in3_gpio, 1);
        gpio_set_level(drv->in4_gpio, 0);
    } else {
        duty_b = (uint32_t)(-phase_b);
        gpio_set_level(drv->in3_gpio, 0);
        gpio_set_level(drv->in4_gpio, 1);
    }

    if (duty_a > drv->max_duty) duty_a = drv->max_duty;
    if (duty_b > drv->max_duty) duty_b = drv->max_duty;

    esp_err_t err;
    err = set_duty(drv->ch_ena, duty_a);
    if (err != ESP_OK) return err;
    return set_duty(drv->ch_enb, duty_b);
}

esp_err_t l298n_coast(const l298n_t *drv)
{
    /* Disable both enable pins, set all direction pins LOW. */
    gpio_set_level(drv->in1_gpio, 0);
    gpio_set_level(drv->in2_gpio, 0);
    gpio_set_level(drv->in3_gpio, 0);
    gpio_set_level(drv->in4_gpio, 0);

    esp_err_t err = set_duty(drv->ch_ena, 0);
    if (err != ESP_OK) return err;
    return set_duty(drv->ch_enb, 0);
}
