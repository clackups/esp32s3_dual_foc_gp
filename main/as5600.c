/*
 * as5600.c — AS5600 12-bit magnetic rotary encoder (I2C) driver.
 */

#include "as5600.h"
#include <math.h>

#define AS5600_REG_RAW_ANGLE 0x0C

esp_err_t as5600_init(as5600_t *dev, i2c_port_t port,
                      int sda, int scl, uint32_t freq_hz)
{
    dev->i2c_port = port;

    const i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = sda,
        .scl_io_num       = scl,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = freq_hz,
    };

    esp_err_t err = i2c_param_config(port, &conf);
    if (err != ESP_OK) return err;

    return i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
}

esp_err_t as5600_read_raw(const as5600_t *dev, uint16_t *raw_angle)
{
    uint8_t buf[2];
    const uint8_t reg = AS5600_REG_RAW_ANGLE;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buf, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    if (err == ESP_OK) {
        *raw_angle = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
    }
    return err;
}

esp_err_t as5600_read_angle_rad(const as5600_t *dev, float *angle_rad)
{
    uint16_t raw;
    esp_err_t err = as5600_read_raw(dev, &raw);
    if (err == ESP_OK) {
        *angle_rad = (float)raw / 4096.0f * 2.0f * (float)M_PI;
    }
    return err;
}
