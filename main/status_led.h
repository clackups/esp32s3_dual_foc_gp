/*
 * status_led.h -- Status RGB LED abstraction.
 *
 * Three implementations are selected at compile time via menuconfig:
 *   - DFGP_STATUS_LED_WS2812      single addressable WS2812 (RMT)
 *   - DFGP_STATUS_LED_RGB_GPIOS   three discrete LEDs on three GPIOs
 *   - DFGP_STATUS_LED_NONE        no LED (all calls become no-ops)
 *
 * Colour values are 8-bit (0-255).  Implementations may scale them
 * internally to suit the hardware.
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>

/** Initialise the status LED according to the menuconfig selection. */
esp_err_t status_led_init(void);

/** Set the LED colour.  All three components are 8-bit (0-255). */
esp_err_t status_led_set(uint8_t r, uint8_t g, uint8_t b);
