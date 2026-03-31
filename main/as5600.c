/*
 * as5600.c — AS5600 12-bit magnetic rotary encoder (I2C) driver.
 *
 * Uses the new ESP-IDF I2C master driver (driver/i2c_master.h).
 */

#include "as5600.h"
#include <math.h>

#define AS5600_REG_RAW_ANGLE 0x0C

esp_err_t as5600_init(as5600_t *dev, i2c_port_t port,
                      int sda, int scl, uint32_t freq_hz)
{
    const i2c_master_bus_config_t bus_cfg = {
        .clk_source                = I2C_CLK_SRC_DEFAULT,
        .i2c_port                  = port,
        .sda_io_num                = sda,
        .scl_io_num                = scl,
        .glitch_ignore_cnt         = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus_handle);
    if (err != ESP_OK) return err;

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = AS5600_I2C_ADDR,
        .scl_speed_hz    = freq_hz,
    };

    return i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev->dev_handle);
}

esp_err_t as5600_read_raw(const as5600_t *dev, uint16_t *raw_angle)
{
    const uint8_t reg = AS5600_REG_RAW_ANGLE;
    uint8_t buf[2];

    esp_err_t err = i2c_master_transmit_receive(dev->dev_handle,
                                                &reg, sizeof(reg),
                                                buf, sizeof(buf),
                                                pdMS_TO_TICKS(50));
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
