/*
 * roller485.h -- Minimal I2C driver for the M5Stack Unit-Roller485.
 *
 * The Roller485 is an integrated BLDC motor + magnetic encoder + FOC
 * controller in a single unit.  Communication is over I2C using the
 * register map documented in:
 *   https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/776/Unit-Roller485-I2C-Protocol-EN.pdf
 *
 * Each Roller485 unit must live on its own I2C bus because the device
 * has a single fixed I2C address (default 0x64).
 *
 * Uses the new ESP-IDF I2C master driver (driver/i2c_master.h).
 */

#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>

/** Default I2C address of the Roller485 unit. */
#define ROLLER485_I2C_ADDR 0x64

/** Roller485 operation modes (register 0x01). */
typedef enum {
    ROLLER485_MODE_SPEED    = 1,
    ROLLER485_MODE_POSITION = 2,
    ROLLER485_MODE_CURRENT  = 3,
    ROLLER485_MODE_ENCODER  = 4,
} roller485_mode_t;

typedef struct {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
} roller485_t;

/**
 * Initialise the I2C bus and add the Roller485 device.
 *
 * @param dev      Pointer to a roller485_t that will hold the handles.
 * @param port     I2C port (I2C_NUM_0 or I2C_NUM_1).
 * @param sda      SDA GPIO number.
 * @param scl      SCL GPIO number.
 * @param freq_hz  I2C clock frequency in Hz.
 * @return ESP_OK on success.
 */
esp_err_t roller485_init(roller485_t *dev, i2c_port_t port,
                         int sda, int scl, uint32_t freq_hz);

/**
 * Set the operating mode (register 0x01).
 */
esp_err_t roller485_set_mode(const roller485_t *dev, roller485_mode_t mode);

/**
 * Enable (1) or disable (0) the motor output (register 0x00).
 */
esp_err_t roller485_set_output(const roller485_t *dev, uint8_t enable);

/**
 * Set the maximum current limit used in position mode (register 0x20).
 * Units match the Roller485 firmware convention (0.01 mA).
 */
esp_err_t roller485_set_pos_max_current(const roller485_t *dev,
                                        int32_t current);

/**
 * Set the target position in position mode (register 0x80).
 * Units are 0.01 deg, so 36000 == one full revolution.
 */
esp_err_t roller485_set_pos(const roller485_t *dev, int32_t pos);

/**
 * Read back the actual position (register 0x90).
 * Units are 0.01 deg, so 36000 == one full revolution.
 */
esp_err_t roller485_get_pos_readback(const roller485_t *dev, int32_t *pos);
