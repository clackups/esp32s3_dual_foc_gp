/*
 * usb_gamepad.h — USB HID gamepad with two 16-bit axes and 10 buttons.
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>

/**
 * Initialise TinyUSB and register the HID gamepad device.
 * Call once from app_main().
 */
esp_err_t usb_gamepad_init(void);

/**
 * Send a gamepad report with updated axis and button values.
 *
 * @param axis_x   X-axis value (−32767 … +32767, 0 = centre).
 * @param axis_y   Y-axis value (−32767 … +32767, 0 = centre).
 * @param buttons  Bitmask of pressed buttons (bit 0 = button 1, …,
 *                 bit 9 = button 10).  1 = pressed.
 * @return ESP_OK on success, ESP_ERR_NOT_FINISHED if the host has
 *         not yet consumed the previous report.
 */
esp_err_t usb_gamepad_report(int16_t axis_x, int16_t axis_y,
                             uint16_t buttons);
