/*
 * roller485.c -- Minimal I2C driver for the M5Stack Unit-Roller485.
 *
 * See the protocol document for the full register map:
 *   https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/776/Unit-Roller485-I2C-Protocol-EN.pdf
 *
 * Multi-byte registers are little-endian on the wire (the device uses
 * raw int32_t / uint32_t representations as confirmed by the official
 * UnitRollerI2C Arduino driver).
 */

#include "roller485.h"

#include <string.h>

/* Register addresses (subset used by this project). */
#define ROLLER485_REG_OUTPUT          0x00
#define ROLLER485_REG_MODE            0x01
#define ROLLER485_REG_POS_MAX_CURRENT 0x20
#define ROLLER485_REG_POS             0x80
#define ROLLER485_REG_POS_READBACK    0x90

/* I2C transfer timeout (ms). */
#define ROLLER485_I2C_TIMEOUT_MS      50

static esp_err_t roller485_write(const roller485_t *dev, uint8_t reg,
                                 const uint8_t *data, size_t len)
{
    /* Pack [reg | data...] into a small stack buffer.  The longest
     * transfer used by this driver is 4 bytes of payload (int32_t). */
    uint8_t buf[1 + 4];
    if (len > sizeof(buf) - 1) return ESP_ERR_INVALID_SIZE;

    buf[0] = reg;
    if (len > 0 && data != NULL) {
        memcpy(&buf[1], data, len);
    }
    return i2c_master_transmit(dev->dev_handle, buf, 1 + len,
                               ROLLER485_I2C_TIMEOUT_MS);
}

static esp_err_t roller485_read(const roller485_t *dev, uint8_t reg,
                                uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev->dev_handle,
                                       &reg, 1,
                                       data, len,
                                       ROLLER485_I2C_TIMEOUT_MS);
}

esp_err_t roller485_init(roller485_t *dev, i2c_port_t port,
                         int sda, int scl, uint32_t freq_hz)
{
    const i2c_master_bus_config_t bus_cfg = {
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .i2c_port                     = port,
        .sda_io_num                   = sda,
        .scl_io_num                   = scl,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &dev->bus_handle);
    if (err != ESP_OK) return err;

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = ROLLER485_I2C_ADDR,
        .scl_speed_hz    = freq_hz,
    };

    return i2c_master_bus_add_device(dev->bus_handle, &dev_cfg,
                                     &dev->dev_handle);
}

esp_err_t roller485_set_mode(const roller485_t *dev, roller485_mode_t mode)
{
    uint8_t v = (uint8_t)mode;
    return roller485_write(dev, ROLLER485_REG_MODE, &v, 1);
}

esp_err_t roller485_set_output(const roller485_t *dev, uint8_t enable)
{
    uint8_t v = enable ? 1 : 0;
    return roller485_write(dev, ROLLER485_REG_OUTPUT, &v, 1);
}

esp_err_t roller485_set_pos_max_current(const roller485_t *dev,
                                        int32_t current)
{
    uint8_t buf[4];
    memcpy(buf, &current, 4);
    return roller485_write(dev, ROLLER485_REG_POS_MAX_CURRENT, buf, 4);
}

esp_err_t roller485_set_pos(const roller485_t *dev, int32_t pos)
{
    uint8_t buf[4];
    memcpy(buf, &pos, 4);
    return roller485_write(dev, ROLLER485_REG_POS, buf, 4);
}

esp_err_t roller485_get_pos_readback(const roller485_t *dev, int32_t *pos)
{
    uint8_t buf[4];
    esp_err_t err = roller485_read(dev, ROLLER485_REG_POS_READBACK, buf, 4);
    if (err == ESP_OK) {
        memcpy(pos, buf, 4);
    }
    return err;
}
