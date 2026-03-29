/*
 * usb_gamepad.c — USB HID gamepad with two 8-bit axes via TinyUSB.
 */

#include "usb_gamepad.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include <string.h>

/* ── HID report descriptor: gamepad with X and Y axes (8-bit each) ── */
static const uint8_t s_hid_report_desc[] = {
    0x05, 0x01,        /*  Usage Page (Generic Desktop)        */
    0x09, 0x05,        /*  Usage (Game Pad)                    */
    0xA1, 0x01,        /*  Collection (Application)            */
    0x09, 0x01,        /*    Usage (Pointer)                   */
    0xA1, 0x00,        /*    Collection (Physical)             */
    0x09, 0x30,        /*      Usage (X)                       */
    0x09, 0x31,        /*      Usage (Y)                       */
    0x15, 0x00,        /*      Logical Minimum (0)             */
    0x26, 0xFF, 0x00,  /*      Logical Maximum (255)           */
    0x75, 0x08,        /*      Report Size (8)                 */
    0x95, 0x02,        /*      Report Count (2)                */
    0x81, 0x02,        /*      Input (Data, Var, Abs)          */
    0xC0,              /*    End Collection                    */
    0xC0               /*  End Collection                      */
};

/* ── TinyUSB descriptor callbacks ────────────────────────────────── */

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

/* ── Public API ──────────────────────────────────────────────────── */

esp_err_t usb_gamepad_init(void)
{
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor        = NULL,  /* use built-in default */
        .string_descriptor        = NULL,
        .string_descriptor_count  = 0,
        .external_phy             = false,
        .configuration_descriptor = NULL,
    };

    return tinyusb_driver_install(&tusb_cfg);
}

esp_err_t usb_gamepad_report(uint8_t axis_x, uint8_t axis_y)
{
    if (!tud_hid_ready()) {
        return ESP_ERR_NOT_FINISHED;
    }

    uint8_t report[2] = { axis_x, axis_y };
    if (!tud_hid_report(0, report, sizeof(report))) {
        return ESP_FAIL;
    }
    return ESP_OK;
}
