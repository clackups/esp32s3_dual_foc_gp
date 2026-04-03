/*
 * main.c -- Dual-FOC haptic USB gamepad application entry point.
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
#include "led_strip.h"
#include <math.h>
#include <stdbool.h>

static const char *TAG = "main";

/* -- Per-axis configuration ------------------------------------------ */
#define MOTOR_POLE_PAIRS  FOC_DEFAULT_POLE_PAIRS

/* Steps and strength are runtime variables so they can be made
 * configurable later (e.g. via NVS or USB commands).               */
static uint16_t s_motor1_steps    = HAPTIC_DEFAULT_STEPS;  /* 21 */
static uint16_t s_motor2_steps    = HAPTIC_DEFAULT_STEPS;  /* 21 */
static float    s_motor1_strength = HAPTIC_DEFAULT_STRENGTH;
static float    s_motor2_strength = HAPTIC_DEFAULT_STRENGTH;
static float    s_motor1_dead_zone = HAPTIC_DEFAULT_DEAD_ZONE;
static float    s_motor2_dead_zone = HAPTIC_DEFAULT_DEAD_ZONE;
static float    s_motor1_angle_offset = 0.0f;  /* magnet mounting offset (rad) */
static float    s_motor2_angle_offset = 0.0f;  /* magnet mounting offset (rad) */

/* -- Continuous centering mode variables ----------------------------- */
static float    s_motor1_cont_dead_zone     = HAPTIC_DEFAULT_CONTINUOUS_DEAD_ZONE;
static float    s_motor2_cont_dead_zone     = HAPTIC_DEFAULT_CONTINUOUS_DEAD_ZONE;
static float    s_motor1_cont_initial_force = HAPTIC_DEFAULT_CONTINUOUS_INITIAL_FORCE;
static float    s_motor2_cont_initial_force = HAPTIC_DEFAULT_CONTINUOUS_INITIAL_FORCE;
static float    s_motor1_cont_max_force     = HAPTIC_DEFAULT_CONTINUOUS_MAX_FORCE;
static float    s_motor2_cont_max_force     = HAPTIC_DEFAULT_CONTINUOUS_MAX_FORCE;

/* -- Button GPIO table ----------------------------------------------- */
static const gpio_num_t s_button_gpios[BUTTON_COUNT] = {
    BUTTON0_GPIO,  BUTTON1_GPIO,  BUTTON2_GPIO,  BUTTON3_GPIO,
    BUTTON4_GPIO,  BUTTON5_GPIO,  BUTTON6_GPIO,  BUTTON7_GPIO,
    BUTTON8_GPIO,  BUTTON9_GPIO,
};

/* -- Hardware instances ---------------------------------------------- */
static as5600_t      s_enc1, s_enc2;
static l298n_t       s_drv1, s_drv2;
static foc_motor_t   s_foc1, s_foc2;
static haptic_axis_t s_axis1, s_axis2;
static led_strip_handle_t s_status_led;

/* -- Shared state (written by individual tasks, read by report task) -- */
static volatile uint16_t s_pos1;
static volatile uint16_t s_pos2;
static volatile int16_t  s_hid1;   /* pre-computed HID axis (continuous) */
static volatile int16_t  s_hid2;
static volatile uint16_t s_buttons;
static volatile bool     s_continuous_mode;  /* true = continuous centering */
static TaskHandle_t      s_report_task_handle;

/* Centre position for each axis (steps / 2).  The HID report sends the
 * signed deviation from this midpoint, scaled to +/-32767.               */
static uint16_t s_pos1_middle;
static uint16_t s_pos2_middle;

/* Centre angle and half-range for continuous centering mode, computed
 * once after haptic calibration so that continuous mode uses the same
 * mechanical centre as the haptic detent layout.                       */
static float s_center_angle1;
static float s_center_angle2;
static float s_half_range1;
static float s_half_range2;

/* -- Haptic task for axis 1 (runs as fast as possible) -------------- */
static void haptic1_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (s_continuous_mode) {
            float raw_angle = 0.0f;
            haptic_continuous_update(&s_axis1,
                                    s_center_angle1, s_half_range1,
                                    s_motor1_cont_dead_zone,
                                    s_motor1_cont_initial_force,
                                    s_motor1_cont_max_force,
                                    &raw_angle);
            /* Continuous HID: deviation from centre mapped linearly. */
            float dev = raw_angle - s_center_angle1;
            if (dev >  (float)M_PI) dev -= 2.0f * (float)M_PI;
            if (dev < -(float)M_PI) dev += 2.0f * (float)M_PI;
            int32_t v = (s_half_range1 > 1e-6f)
                      ? (int32_t)(dev / s_half_range1 * 32767.0f)
                      : 0;
            if (v >  32767) v =  32767;
            if (v < -32767) v = -32767;
            s_hid1 = (int16_t)v;
        } else {
            uint16_t pos = 0;
            haptic_update(&s_axis1, &pos);
            s_pos1 = pos;
        }
        xTaskNotifyGive(s_report_task_handle);
    }
}

/* -- Haptic task for axis 2 (runs as fast as possible) -------------- */
static void haptic2_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (s_continuous_mode) {
            float raw_angle = 0.0f;
            haptic_continuous_update(&s_axis2,
                                    s_center_angle2, s_half_range2,
                                    s_motor2_cont_dead_zone,
                                    s_motor2_cont_initial_force,
                                    s_motor2_cont_max_force,
                                    &raw_angle);
            float dev = raw_angle - s_center_angle2;
            if (dev >  (float)M_PI) dev -= 2.0f * (float)M_PI;
            if (dev < -(float)M_PI) dev += 2.0f * (float)M_PI;
            int32_t v = (s_half_range2 > 1e-6f)
                      ? (int32_t)(dev / s_half_range2 * 32767.0f)
                      : 0;
            if (v >  32767) v =  32767;
            if (v < -32767) v = -32767;
            s_hid2 = (int16_t)v;
        } else {
            uint16_t pos = 0;
            haptic_update(&s_axis2, &pos);
            s_pos2 = pos;
        }
        xTaskNotifyGive(s_report_task_handle);
    }
}

/* -- Button polling task (~1 kHz) ----------------------------------- */
static void button_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();

    /* Edge detection for mode toggle pin (active-low). */
    int prev_toggle_level = 1;

    for (;;) {
        uint16_t buttons = 0;
        for (int i = 0; i < BUTTON_COUNT; i++) {
            if (gpio_get_level(s_button_gpios[i]) == 0) {
                buttons |= (1U << i);
            }
        }
        s_buttons = buttons;

        /* Detect falling edge on mode toggle GPIO. */
        int toggle_level = gpio_get_level(MODE_TOGGLE_GPIO);
        if (toggle_level == 0 && prev_toggle_level == 1) {
            s_continuous_mode = !s_continuous_mode;
            ESP_LOGI(TAG, "Mode toggled: %s",
                     s_continuous_mode ? "continuous" : "haptic");
            /* Status LED: yellow = continuous, green = haptic. */
            if (s_continuous_mode) {
                led_strip_set_pixel(s_status_led, 0, 32, 32, 0); /* yellow */
            } else {
                led_strip_set_pixel(s_status_led, 0, 0, 32, 0);  /* green */
            }
            led_strip_refresh(s_status_led);
        }
        prev_toggle_level = toggle_level;

        xTaskNotifyGive(s_report_task_handle);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}

/* -- USB HID report task (sends on change + every 100 ms) ----------- */
static void report_task(void *arg)
{
    (void)arg;
    int16_t  prev_v1 = 0, prev_v2 = 0;
    uint16_t prev_buttons = 0;
    TickType_t last_send = xTaskGetTickCount();
    const TickType_t periodic_interval = pdMS_TO_TICKS(100);

    for (;;) {
        /* Wait for any writer to signal, or time out after 1 ms. */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));

        uint16_t buttons = s_buttons;
        int16_t  v1, v2;

        if (s_continuous_mode) {
            /* In continuous mode the haptic tasks pre-compute the HID
             * axis values from the raw encoder angle.                  */
            v1 = s_hid1;
            v2 = s_hid2;
        } else {
            uint16_t pos1 = s_pos1;
            uint16_t pos2 = s_pos2;

            /* Map position deviation from middle to signed 16-bit HID
             * axis (-32767 ... +32767).  The centre detent (pos == middle)
             * maps to exactly 0, so the host sees a true zero with no
             * rounding artefacts.
             *
             * half = steps / 2 (equal to middle for odd step counts).
             * val  = (pos - middle) * 32767 / half, clamped to +/-32767. */
            int32_t half1 = (int32_t)(s_axis1.steps / 2);
            int32_t dev1  = (int32_t)pos1 - (int32_t)s_pos1_middle;
            int32_t val1  = (half1 > 0) ? (dev1 * 32767 / half1) : 0;
            if (val1 >  32767) val1 =  32767;
            if (val1 < -32767) val1 = -32767;
            v1 = (int16_t)val1;

            int32_t half2 = (int32_t)(s_axis2.steps / 2);
            int32_t dev2  = (int32_t)pos2 - (int32_t)s_pos2_middle;
            int32_t val2  = (half2 > 0) ? (dev2 * 32767 / half2) : 0;
            if (val2 >  32767) val2 =  32767;
            if (val2 < -32767) val2 = -32767;
            v2 = (int16_t)val2;
        }

        bool changed = (v1 != prev_v1 || v2 != prev_v2 ||
                        buttons != prev_buttons);
        bool periodic = (xTaskGetTickCount() - last_send >= periodic_interval);

        if (changed || periodic) {
            usb_gamepad_report(v1, v2, buttons);
            prev_v1      = v1;
            prev_v2      = v2;
            prev_buttons = buttons;
            last_send    = xTaskGetTickCount();
        }
    }
}

/* -- Application entry point ---------------------------------------- */
void app_main(void)
{
    /* -- Status LED -- red while booting / calibrating --------------- */
    const led_strip_config_t strip_cfg = {
        .strip_gpio_num   = STATUS_LED_GPIO,
        .max_leds         = 1,
    };
    const led_strip_rmt_config_t rmt_cfg = {
        .resolution_hz = 10 * 1000 * 1000,  /* 10 MHz */
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg,
                                             &s_status_led));
    led_strip_set_pixel(s_status_led, 0, 32, 0, 0);   /* red */
    led_strip_refresh(s_status_led);

    ESP_LOGI(TAG, "Initialising encoders ...");
    ESP_ERROR_CHECK(as5600_init(&s_enc1, ENCODER1_I2C_PORT,
                                ENCODER1_SDA_GPIO, ENCODER1_SCL_GPIO,
                                ENCODER_I2C_FREQ_HZ));
    ESP_ERROR_CHECK(as5600_init(&s_enc2, ENCODER2_I2C_PORT,
                                ENCODER2_SDA_GPIO, ENCODER2_SCL_GPIO,
                                ENCODER_I2C_FREQ_HZ));

    ESP_LOGI(TAG, "Initialising buttons ...");
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

    /* Mode toggle GPIO (active-low, internal pull-up). */
    {
        const gpio_config_t toggle_cfg = {
            .pin_bit_mask = 1ULL << MODE_TOGGLE_GPIO,
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&toggle_cfg));
    }

    ESP_LOGI(TAG, "Initialising motor drivers ...");
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

    ESP_LOGI(TAG, "Calibrating FOC ...");
    foc_init(&s_foc1, &s_enc1, &s_drv1, MOTOR_POLE_PAIRS, s_motor1_angle_offset);
    foc_init(&s_foc2, &s_enc2, &s_drv2, MOTOR_POLE_PAIRS, s_motor2_angle_offset);
    ESP_ERROR_CHECK(foc_calibrate(&s_foc1));
    ESP_LOGI(TAG, "Zero angle #1: %f", s_foc1.zero_electrical_angle);
    ESP_ERROR_CHECK(foc_calibrate(&s_foc2));
    ESP_LOGI(TAG, "Zero angle #2: %f", s_foc2.zero_electrical_angle);

    ESP_LOGI(TAG, "Setting up haptic axes ...");
    haptic_init(&s_axis1, &s_foc1, s_motor1_steps, s_motor1_strength, s_motor1_dead_zone);
    haptic_init(&s_axis2, &s_foc2, s_motor2_steps, s_motor2_strength, s_motor2_dead_zone);

    ESP_LOGI(TAG, "Calibrating haptic detent positions ...");
    ESP_ERROR_CHECK(haptic_calibrate(&s_axis1));
    ESP_ERROR_CHECK(haptic_calibrate(&s_axis2));

    /* Compute and store the centre detent index for HID reporting. */
    s_pos1_middle = s_axis1.steps / 2;
    s_pos2_middle = s_axis2.steps / 2;

    /* Compute centre angle and half-range for continuous centering mode.
     * The centre matches the middle haptic detent so that switching
     * modes does not shift the neutral point.                          */
    s_center_angle1 = (float)s_pos1_middle * s_axis1.step_angle
                    + s_axis1.phase_offset;
    s_center_angle2 = (float)s_pos2_middle * s_axis2.step_angle
                    + s_axis2.phase_offset;
    s_half_range1   = (float)(s_axis1.steps / 2) * s_axis1.step_angle;
    s_half_range2   = (float)(s_axis2.steps / 2) * s_axis2.step_angle;

    ESP_LOGI(TAG, "Moving motors to centre position ...");
    ESP_ERROR_CHECK(haptic_move_to_detent(&s_axis1, s_pos1_middle));
    ESP_ERROR_CHECK(haptic_move_to_detent(&s_axis2, s_pos2_middle));

    /* -- Status LED -- green, calibration complete ------------------- */
    led_strip_set_pixel(s_status_led, 0, 0, 32, 0);   /* green */
    led_strip_refresh(s_status_led);

    ESP_LOGI(TAG, "Starting USB gamepad ...");
    ESP_ERROR_CHECK(usb_gamepad_init());

    /* Create report task first so its handle is available to writers. */
    xTaskCreatePinnedToCore(report_task,  "report",  4096, NULL, 4, &s_report_task_handle, 0);
    xTaskCreatePinnedToCore(haptic1_task, "haptic1", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(haptic2_task, "haptic2", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(button_task,  "button",  2048, NULL, 3, NULL, 0);

    ESP_LOGI(TAG, "Dual-FOC haptic gamepad running.");
}
