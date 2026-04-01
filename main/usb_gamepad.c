/*
 * usb_gamepad.c -- USB HID gamepad with two 16-bit axes and 10 buttons
 *                 via TinyUSB.
 */

#include "usb_gamepad.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include <string.h>

/* -- HID report descriptor ------------------------------------------
 *
 * Report layout (6 bytes total):
 *   byte 0     - buttons 1-8  (bits 0-7)
 *   byte 1     - buttons 9-10 (bits 0-1) + 6-bit padding
 *   bytes 2-3  - X axis (signed 16-bit LE, -32767 ... +32767, 0 = centre)
 *   bytes 4-5  - Y axis (signed 16-bit LE, -32767 ... +32767, 0 = centre)
 */
static const uint8_t s_hid_report_desc[] = {
    0x05, 0x01,        /*  Usage Page (Generic Desktop)        */
    0x09, 0x05,        /*  Usage (Game Pad)                    */
    0xA1, 0x01,        /*  Collection (Application)            */

    /* 10 buttons */
    0x05, 0x09,        /*    Usage Page (Button)               */
    0x19, 0x01,        /*    Usage Minimum (Button 1)          */
    0x29, 0x0A,        /*    Usage Maximum (Button 10)         */
    0x15, 0x00,        /*    Logical Minimum (0)               */
    0x25, 0x01,        /*    Logical Maximum (1)               */
    0x75, 0x01,        /*    Report Size (1)                   */
    0x95, 0x0A,        /*    Report Count (10)                 */
    0x81, 0x02,        /*    Input (Data, Var, Abs)            */

    /* 6-bit padding to byte-align */
    0x75, 0x01,        /*    Report Size (1)                   */
    0x95, 0x06,        /*    Report Count (6)                  */
    0x81, 0x01,        /*    Input (Const)                     */

    /* X and Y axes (signed 16-bit) */
    0x05, 0x01,        /*    Usage Page (Generic Desktop)      */
    0x09, 0x01,        /*    Usage (Pointer)                   */
    0xA1, 0x00,        /*    Collection (Physical)             */
    0x09, 0x30,        /*      Usage (X)                       */
    0x09, 0x31,        /*      Usage (Y)                       */
    0x16, 0x01, 0x80,  /*      Logical Minimum (-32767)        */
    0x26, 0xFF, 0x7F,  /*      Logical Maximum (32767)         */
    0x75, 0x10,        /*      Report Size (16)                */
    0x95, 0x02,        /*      Report Count (2)                */
    0x81, 0x02,        /*      Input (Data, Var, Abs)          */
    0xC0,              /*    End Collection                    */

    0xC0               /*  End Collection                      */
};

/* -- USB configuration descriptor ---------------------------------- */

#define TUSB_DESC_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)

static const uint8_t s_configuration_desc[] = {
    /* config 1, 1 interface, no string, bus-powered 100 mA */
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, 0, 100),
    /* itf 0, EP 0x81 (IN 1), 64-byte packet, 10 ms poll interval */
    TUD_HID_DESCRIPTOR(0, 0, HID_ITF_PROTOCOL_NONE,
                       sizeof(s_hid_report_desc), 0x81, 64, 10),
};

/* -- TinyUSB descriptor callbacks ---------------------------------- */

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    return s_hid_report_desc;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                                hid_report_type_t report_type,
                                uint8_t *buffer, uint16_t reqlen)
{
    (void)instance; (void)report_id; (void)report_type;
    (void)buffer;   (void)reqlen;
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                            hid_report_type_t report_type,
                            uint8_t const *buffer, uint16_t bufsize)
{
    (void)instance; (void)report_id; (void)report_type;
    (void)buffer;   (void)bufsize;
}

/* -- Public API ---------------------------------------------------- */

esp_err_t usb_gamepad_init(void)
{
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor        = NULL,  /* use built-in default */
        .string_descriptor        = NULL,
        .string_descriptor_count  = 0,
        .external_phy             = false,
        .configuration_descriptor = s_configuration_desc,
    };

    return tinyusb_driver_install(&tusb_cfg);
}

esp_err_t usb_gamepad_report(int16_t axis_x, int16_t axis_y,
                            uint16_t buttons)
{
    if (!tud_hid_ready()) {
        return ESP_ERR_NOT_FINISHED;
    }

    /* Report: 2 bytes buttons (10 bits + 6 padding) + 4 bytes axes
     * (two signed 16-bit little-endian values).                     */
    uint8_t report[6] = {
        (uint8_t)(buttons & 0xFF),         /* buttons 1-8          */
        (uint8_t)((buttons >> 8) & 0x03),  /* buttons 9-10 + pad   */
        (uint8_t)(axis_x & 0xFF),          /* X low byte           */
        (uint8_t)((axis_x >> 8) & 0xFF),   /* X high byte          */
        (uint8_t)(axis_y & 0xFF),          /* Y low byte           */
        (uint8_t)((axis_y >> 8) & 0xFF),   /* Y high byte          */
    };
    if (!tud_hid_report(0, report, sizeof(report))) {
        return ESP_FAIL;
    }
    return ESP_OK;
}
