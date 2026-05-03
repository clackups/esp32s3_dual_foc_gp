/*
 * status_led.c -- Status RGB LED abstraction.
 *
 * Compile-time variant selection via the "Dual-FOC GP" menuconfig
 * menu.  See status_led.h for the API.
 */

#include "status_led.h"
#include "sdkconfig.h"

#if defined(CONFIG_DFGP_STATUS_LED_WS2812)

#include "led_strip.h"

static led_strip_handle_t s_strip;

esp_err_t status_led_init(void)
{
    const led_strip_config_t strip_cfg = {
        .strip_gpio_num = CONFIG_DFGP_STATUS_LED_WS2812_GPIO,
        .max_leds       = 1,
    };
    const led_strip_rmt_config_t rmt_cfg = {
        .resolution_hz = 10 * 1000 * 1000,  /* 10 MHz */
    };
    return led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_strip);
}

esp_err_t status_led_set(uint8_t r, uint8_t g, uint8_t b)
{
    esp_err_t err = led_strip_set_pixel(s_strip, 0, r, g, b);
    if (err != ESP_OK) return err;
    return led_strip_refresh(s_strip);
}

#elif defined(CONFIG_DFGP_STATUS_LED_RGB_GPIOS)

#include "driver/gpio.h"

#define STATUS_LED_R_GPIO   CONFIG_DFGP_STATUS_LED_R_GPIO
#define STATUS_LED_G_GPIO   CONFIG_DFGP_STATUS_LED_G_GPIO
#define STATUS_LED_B_GPIO   CONFIG_DFGP_STATUS_LED_B_GPIO

#ifdef CONFIG_DFGP_STATUS_LED_RGB_ACTIVE_LOW
#define STATUS_LED_ON_LEVEL   0
#define STATUS_LED_OFF_LEVEL  1
#else
#define STATUS_LED_ON_LEVEL   1
#define STATUS_LED_OFF_LEVEL  0
#endif

esp_err_t status_led_init(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << STATUS_LED_R_GPIO) |
                        (1ULL << STATUS_LED_G_GPIO) |
                        (1ULL << STATUS_LED_B_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) return err;

    /* Start with all channels off. */
    gpio_set_level(STATUS_LED_R_GPIO, STATUS_LED_OFF_LEVEL);
    gpio_set_level(STATUS_LED_G_GPIO, STATUS_LED_OFF_LEVEL);
    gpio_set_level(STATUS_LED_B_GPIO, STATUS_LED_OFF_LEVEL);
    return ESP_OK;
}

esp_err_t status_led_set(uint8_t r, uint8_t g, uint8_t b)
{
    /* Discrete LEDs are on/off only -- treat any non-zero component
     * as "on" for that channel.                                       */
    gpio_set_level(STATUS_LED_R_GPIO,
                   r ? STATUS_LED_ON_LEVEL : STATUS_LED_OFF_LEVEL);
    gpio_set_level(STATUS_LED_G_GPIO,
                   g ? STATUS_LED_ON_LEVEL : STATUS_LED_OFF_LEVEL);
    gpio_set_level(STATUS_LED_B_GPIO,
                   b ? STATUS_LED_ON_LEVEL : STATUS_LED_OFF_LEVEL);
    return ESP_OK;
}

#else  /* CONFIG_DFGP_STATUS_LED_NONE (or any unset state) */

esp_err_t status_led_init(void)
{
    return ESP_OK;
}

esp_err_t status_led_set(uint8_t r, uint8_t g, uint8_t b)
{
    (void)r;
    (void)g;
    (void)b;
    return ESP_OK;
}

#endif
