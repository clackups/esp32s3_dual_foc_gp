/*
 * main.c — Dual-FOC haptic USB gamepad application entry point.
 *
 * Two 2804 BLDC motors (3 coil inputs, 7 pole pairs) with AS5600
 * encoders provide force-feedback detents.  The angular positions are
 * reported as USB HID gamepad axes.
 */

#include "pin_config.h"
#include "as5600.h"
#include "l298n.h"
#include "foc.h"
#include "haptic.h"
#include "usb_gamepad.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "main";

/* ── Per-axis configuration ────────────────────────────────────────── */
#define MOTOR_POLE_PAIRS  FOC_DEFAULT_POLE_PAIRS

/* Steps and strength are runtime variables so they can be made
 * configurable later (e.g. via NVS or USB commands).               */
static uint16_t s_motor1_steps    = HAPTIC_DEFAULT_STEPS;  /* 7 */
static uint16_t s_motor2_steps    = HAPTIC_DEFAULT_STEPS;  /* 7 */
static float    s_motor1_strength = HAPTIC_DEFAULT_STRENGTH;
static float    s_motor2_strength = HAPTIC_DEFAULT_STRENGTH;

/* ── Button GPIO table ─────────────────────────────────────────────── */
static const gpio_num_t s_button_gpios[BUTTON_COUNT] = {
    BUTTON1_GPIO,  BUTTON2_GPIO,  BUTTON3_GPIO,  BUTTON4_GPIO,
    BUTTON5_GPIO,  BUTTON6_GPIO,  BUTTON7_GPIO,  BUTTON8_GPIO,
    BUTTON9_GPIO,  BUTTON10_GPIO,
};

/* ── Hardware instances ────────────────────────────────────────────── */
static as5600_t      s_enc1, s_enc2;
static l298n_t       s_drv1, s_drv2;
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

        /* Read button states (active-low → invert). */
        uint16_t buttons = 0;
        for (int i = 0; i < BUTTON_COUNT; i++) {
            if (gpio_get_level(s_button_gpios[i]) == 0) {
                buttons |= (1U << i);
            }
        }

        /* Map step index to 0 – 255 range for each axis. */
        uint8_t x = (uint8_t)((uint32_t)pos1 * 255U / (uint32_t)(s_axis1.steps - 1));
        uint8_t y = (uint8_t)((uint32_t)pos2 * 255U / (uint32_t)(s_axis2.steps - 1));
        usb_gamepad_report(x, y, buttons);

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

    ESP_LOGI(TAG, "Initialising buttons …");
    for (int i = 0; i < BUTTON_COUNT; i++) {
        const gpio_config_t btn_cfg = {
            .pin_bit_mask = 1ULL << s_button_gpios[i],
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&btn_cfg));
    }

    ESP_LOGI(TAG, "Initialising motor drivers …");
    ESP_ERROR_CHECK(l298n_init(&s_drv1, LEDC_TIMER_0,
                               MOTOR1_IN1_GPIO, MOTOR1_IN2_GPIO,
                               MOTOR1_IN3_GPIO,
                               LEDC_CHANNEL_0,
                               MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION));
    ESP_ERROR_CHECK(l298n_init(&s_drv2, LEDC_TIMER_1,
                               MOTOR2_IN1_GPIO, MOTOR2_IN2_GPIO,
                               MOTOR2_IN3_GPIO,
                               LEDC_CHANNEL_3,
                               MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION));

    ESP_LOGI(TAG, "Calibrating FOC …");
    foc_init(&s_foc1, &s_enc1, &s_drv1, MOTOR_POLE_PAIRS);
    foc_init(&s_foc2, &s_enc2, &s_drv2, MOTOR_POLE_PAIRS);
    ESP_ERROR_CHECK(foc_calibrate(&s_foc1));
    ESP_ERROR_CHECK(foc_calibrate(&s_foc2));

    ESP_LOGI(TAG, "Setting up haptic axes …");
    haptic_init(&s_axis1, &s_foc1, s_motor1_steps, s_motor1_strength);
    haptic_init(&s_axis2, &s_foc2, s_motor2_steps, s_motor2_strength);

    ESP_LOGI(TAG, "Starting USB gamepad …");
    ESP_ERROR_CHECK(usb_gamepad_init());

    xTaskCreatePinnedToCore(haptic_task, "haptic", 4096, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "Dual-FOC haptic gamepad running.");
}
