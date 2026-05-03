/*
 * main.c -- Dual-FOC haptic USB gamepad application entry point.
 *
 * Two M5Stack Roller485 units (each on its own I2C bus) provide the
 * motor + encoder + FOC controller for the two haptic axes.  The
 * Roller485 is run in position mode and the haptic engine snaps the
 * target position to the nearest detent at every loop iteration; the
 * unit's internal PID + current limit (register 0x20) provides the
 * detent "snap" feel.  The angular positions are reported as USB HID
 * gamepad axes.
 */

#include "pin_config.h"
#include "roller485.h"
#include "haptic.h"
#include "usb_gamepad.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include <stdbool.h>
#include <stdint.h>

static const char *TAG = "main";

/* -- Per-axis configuration ------------------------------------------ */
/* Steps and max current are runtime variables so they can be made
 * configurable later (e.g. via NVS or USB commands).  Defaults come
 * from the "Dual-FOC GP" Kconfig menu.                                */
static uint16_t s_motor1_steps       = HAPTIC_DEFAULT_STEPS;
static uint16_t s_motor2_steps       = HAPTIC_DEFAULT_STEPS;
static int32_t  s_motor1_max_current = HAPTIC_DEFAULT_MAX_CURRENT;
static int32_t  s_motor2_max_current = HAPTIC_DEFAULT_MAX_CURRENT;

/* -- Button GPIO table ----------------------------------------------- */
static const gpio_num_t s_button_gpios[BUTTON_COUNT] = {
    BUTTON0_GPIO,  BUTTON1_GPIO,  BUTTON2_GPIO,  BUTTON3_GPIO,
    BUTTON4_GPIO,  BUTTON5_GPIO,  BUTTON6_GPIO,  BUTTON7_GPIO,
    BUTTON8_GPIO,  BUTTON9_GPIO,
};

/* -- Hardware instances ---------------------------------------------- */
static roller485_t   s_roller1, s_roller2;
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

/* Centre detent index for haptic mode HID reporting. */
static uint16_t s_pos1_middle;
static uint16_t s_pos2_middle;

/* Centre position (Roller485 0.01 deg counts) and half-range for the
 * continuous centering mode.  The centre is captured at startup so
 * that switching modes does not shift the neutral point.             */
static int32_t s_center_counts1;
static int32_t s_center_counts2;
static int32_t s_half_range1;
static int32_t s_half_range2;

/* Map a raw position deviation in counts to an int16 HID axis value
 * scaled to +/-32767, clamping out-of-range values.                  */
static int16_t counts_to_hid(int32_t deviation, int32_t half_range)
{
    if (half_range <= 0) return 0;
    int32_t v = deviation * 32767 / half_range;
    if (v >  32767) v =  32767;
    if (v < -32767) v = -32767;
    return (int16_t)v;
}

/* -- Haptic task for axis 1 ----------------------------------------- */
static void haptic1_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(2);
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        if (s_continuous_mode) {
            int32_t raw = 0;
            haptic_continuous_update(&s_axis1, s_center_counts1, &raw);
            s_hid1 = counts_to_hid(raw - s_center_counts1, s_half_range1);
        } else {
            uint16_t pos = 0;
            haptic_update(&s_axis1, &pos);
            s_pos1 = pos;
        }
        xTaskNotifyGive(s_report_task_handle);
        vTaskDelayUntil(&last_wake, period);
    }
}

/* -- Haptic task for axis 2 ----------------------------------------- */
static void haptic2_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(2);
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        if (s_continuous_mode) {
            int32_t raw = 0;
            haptic_continuous_update(&s_axis2, s_center_counts2, &raw);
            s_hid2 = counts_to_hid(raw - s_center_counts2, s_half_range2);
        } else {
            uint16_t pos = 0;
            haptic_update(&s_axis2, &pos);
            s_pos2 = pos;
        }
        xTaskNotifyGive(s_report_task_handle);
        vTaskDelayUntil(&last_wake, period);
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
            /* Status LED: blue = continuous, green = haptic. */
            if (s_continuous_mode) {
                led_strip_set_pixel(s_status_led, 0, 0, 0, 32);  /* blue */
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
             * axis values from the actual measured position.          */
            v1 = s_hid1;
            v2 = s_hid2;
        } else {
            uint16_t pos1 = s_pos1;
            uint16_t pos2 = s_pos2;

            /* Map detent index deviation from the middle to a signed
             * 16-bit HID axis value.  half = steps / 2 (equal to
             * middle for odd step counts). */
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
    /* -- Status LED -- red while booting / initialising -------------- */
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

    ESP_LOGI(TAG, "Initialising Roller485 unit #1 ...");
    ESP_ERROR_CHECK(roller485_init(&s_roller1, ROLLER1_I2C_PORT,
                                   ROLLER1_SDA_GPIO, ROLLER1_SCL_GPIO,
                                   ROLLER_I2C_FREQ_HZ));
    ESP_LOGI(TAG, "Initialising Roller485 unit #2 ...");
    ESP_ERROR_CHECK(roller485_init(&s_roller2, ROLLER2_I2C_PORT,
                                   ROLLER2_SDA_GPIO, ROLLER2_SCL_GPIO,
                                   ROLLER_I2C_FREQ_HZ));

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

    ESP_LOGI(TAG, "Setting up haptic axes (steps=%u, max_current=%ld) ...",
             (unsigned)s_motor1_steps, (long)s_motor1_max_current);
    ESP_ERROR_CHECK(haptic_init(&s_axis1, &s_roller1,
                                s_motor1_steps, s_motor1_max_current));
    ESP_ERROR_CHECK(haptic_init(&s_axis2, &s_roller2,
                                s_motor2_steps, s_motor2_max_current));

    /* Compute and store the centre detent index for HID reporting. */
    s_pos1_middle = s_axis1.steps / 2;
    s_pos2_middle = s_axis2.steps / 2;

    /* Centre position (in Roller485 counts) and half-range for the
     * continuous centering mode.  Uses the same mechanical centre as
     * the haptic-detent middle so switching modes does not shift the
     * neutral point.                                                  */
    s_center_counts1 = s_axis1.origin_counts +
                       (int32_t)s_pos1_middle * s_axis1.counts_per_step;
    s_center_counts2 = s_axis2.origin_counts +
                       (int32_t)s_pos2_middle * s_axis2.counts_per_step;
    s_half_range1    = (int32_t)(s_axis1.steps / 2) * s_axis1.counts_per_step;
    s_half_range2    = (int32_t)(s_axis2.steps / 2) * s_axis2.counts_per_step;

    ESP_LOGI(TAG, "Moving motors to centre position ...");
    ESP_ERROR_CHECK(haptic_move_to_detent(&s_axis1, s_pos1_middle));
    ESP_ERROR_CHECK(haptic_move_to_detent(&s_axis2, s_pos2_middle));

    /* -- Status LED -- green, initialisation complete --------------- */
    led_strip_set_pixel(s_status_led, 0, 0, 32, 0);   /* green */
    led_strip_refresh(s_status_led);

    ESP_LOGI(TAG, "Starting USB gamepad ...");
    ESP_ERROR_CHECK(usb_gamepad_init());

    /* Create report task first so its handle is available to writers. */
    xTaskCreatePinnedToCore(report_task,  "report",  4096, NULL, 4, &s_report_task_handle, 0);
    xTaskCreatePinnedToCore(haptic1_task, "haptic1", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(haptic2_task, "haptic2", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(button_task,  "button",  4096, NULL, 3, NULL, 0);

    ESP_LOGI(TAG, "Dual-FOC haptic gamepad running.");
}
