/*
 * as5600.h -- AS5600 12-bit magnetic rotary encoder (I2C) driver.
 *
 * Uses the new ESP-IDF I2C master driver (driver/i2c_master.h).
 */

#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

#define AS5600_I2C_ADDR 0x36

typedef struct {
    i2c_master_dev_handle_t dev_handle;
} as5600_t;

/**
 * Initialise the I2C bus and add the AS5600 device.
 *
 * @param dev  Pointer to an as5600_t that will hold the device handle.
 * @param port I2C port (I2C_NUM_0 or I2C_NUM_1).
 * @param sda  SDA GPIO number.
 * @param scl  SCL GPIO number.
 * @param freq_hz I2C clock frequency in Hz.
 * @return ESP_OK on success.
 */
esp_err_t as5600_init(as5600_t *dev, i2c_port_t port,
                      int sda, int scl, uint32_t freq_hz);

/**
 * Read the raw 12-bit angle (0 - 4095).
 */
esp_err_t as5600_read_raw(const as5600_t *dev, uint16_t *raw_angle);

/**
 * Read the angle in radians (0 - 2pi).
 */
esp_err_t as5600_read_angle_rad(const as5600_t *dev, float *angle_rad);
