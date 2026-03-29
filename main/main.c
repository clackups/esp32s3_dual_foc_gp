/*
 * main.c — Dual-FOC haptic USB gamepad application entry point.
 *
 * Two 2804 BLDC motors (3 coil inputs, 7 pole pairs) with AS5600
 * encoders provide force-feedback detents.  The angular positions are
 * reported as USB HID gamepad axes.
 */

#include "pin_config.h"
#include "as5600.h"
#include "drv8833.h"
#include "foc.h"
#include "haptic.h"
#include "usb_gamepad.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "main";

/* ── Per-axis configuration ────────────────────────────────────────── */
#define MOTOR1_STEPS  12      /* haptic detent steps per revolution   */
#define MOTOR2_STEPS  24      /* second axis can use a different count */
#define MOTOR_POLE_PAIRS  FOC_DEFAULT_POLE_PAIRS

/* ── Hardware instances ────────────────────────────────────────────── */
static as5600_t      s_enc1, s_enc2;
static drv8833_t     s_drv1, s_drv2;
static foc_motor_t   s_foc1, s_foc2;
static haptic_axis_t s_axis1, s_axis2;

/* ── Haptic + report task ─────────────────────────────────────────── */
static void haptic_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        uint16_t pos1 = 0, pos2 = 0;
        haptic_update(&s_axis1, &pos1);
        haptic_update(&s_axis2, &pos2);

        /* Map step index to 0 – 255 range for each axis. */
        uint8_t x = (uint8_t)((uint32_t)pos1 * 255 / (s_axis1.steps - 1));
        uint8_t y = (uint8_t)((uint32_t)pos2 * 255 / (s_axis2.steps - 1));
        usb_gamepad_report(x, y);

        /* Run at ~1 kHz. */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}

/* ── Application entry point ──────────────────────────────────────── */
void app_main(void)
{
    ESP_LOGI(TAG, "Initialising encoders …");
    ESP_ERROR_CHECK(as5600_init(&s_enc1, ENCODER1_I2C_PORT,
                                ENCODER1_SDA_GPIO, ENCODER1_SCL_GPIO,
                                ENCODER_I2C_FREQ_HZ));
    ESP_ERROR_CHECK(as5600_init(&s_enc2, ENCODER2_I2C_PORT,
                                ENCODER2_SDA_GPIO, ENCODER2_SCL_GPIO,
                                ENCODER_I2C_FREQ_HZ));

    ESP_LOGI(TAG, "Initialising motor drivers …");
    ESP_ERROR_CHECK(drv8833_init(&s_drv1, LEDC_TIMER_0,
                                 MOTOR1_AIN1_GPIO, MOTOR1_AIN2_GPIO,
                                 MOTOR1_BIN1_GPIO, MOTOR1_BIN2_GPIO,
                                 LEDC_CHANNEL_0,
                                 MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION));
    ESP_ERROR_CHECK(drv8833_init(&s_drv2, LEDC_TIMER_1,
                                 MOTOR2_AIN1_GPIO, MOTOR2_AIN2_GPIO,
                                 MOTOR2_BIN1_GPIO, MOTOR2_BIN2_GPIO,
                                 LEDC_CHANNEL_4,
                                 MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION));

    ESP_LOGI(TAG, "Calibrating FOC …");
    foc_init(&s_foc1, &s_enc1, &s_drv1, MOTOR_POLE_PAIRS);
    foc_init(&s_foc2, &s_enc2, &s_drv2, MOTOR_POLE_PAIRS);
    ESP_ERROR_CHECK(foc_calibrate(&s_foc1));
    ESP_ERROR_CHECK(foc_calibrate(&s_foc2));

    ESP_LOGI(TAG, "Setting up haptic axes …");
    haptic_init(&s_axis1, &s_foc1, MOTOR1_STEPS, HAPTIC_DEFAULT_STRENGTH);
    haptic_init(&s_axis2, &s_foc2, MOTOR2_STEPS, HAPTIC_DEFAULT_STRENGTH);

    ESP_LOGI(TAG, "Starting USB gamepad …");
    ESP_ERROR_CHECK(usb_gamepad_init());

    xTaskCreatePinnedToCore(haptic_task, "haptic", 4096, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "Dual-FOC haptic gamepad running.");
}
