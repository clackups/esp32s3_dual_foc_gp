/*
 * pin_config.h -- GPIO assignments for the dual-FOC gamepad.
 *
 * All hardware pin definitions live here so that porting to a different
 * PCB only requires editing this single file.
 */

#pragma once

/*
 * Motor wiring (2804 BLDC, 3 coil inputs U/V/W):
 *   TMC6300 -- three-phase MOSFET gate driver.
 *   UH (PWM)  -> Phase U high-side gate  (sinusoidal PWM)
 *   VH (PWM)  -> Phase V high-side gate  (sinusoidal PWM)
 *   WH (PWM)  -> Phase W high-side gate  (sinusoidal PWM)
 *   UL, VL, WL -> tied to +3.3 V (low-side FETs always on, 3-PWM mode)
 *   VIO        -> tied to +3.3 V (driver always active)
 */

/* -- Motor 1 -- TMC6300 #1 ------------------------------------------- */
#define MOTOR1_UH_GPIO      2   /* Phase U high-side - PWM */
#define MOTOR1_VH_GPIO      39  /* Phase V high-side - PWM */
#define MOTOR1_WH_GPIO      41  /* Phase W high-side - PWM */

/* -- Motor 2 -- TMC6300 #2 ------------------------------------------- */
#define MOTOR2_UH_GPIO      14  /* Phase U high-side - PWM */
#define MOTOR2_VH_GPIO      16  /* Phase V high-side - PWM */
#define MOTOR2_WH_GPIO      42  /* Phase W high-side - PWM */

/* -- Game controller buttons (active-low, internal pull-up) ---------- */
#define BUTTON_COUNT        10
#define BUTTON0_GPIO        4
#define BUTTON1_GPIO        5
#define BUTTON2_GPIO        6
#define BUTTON3_GPIO        7
#define BUTTON4_GPIO        8
#define BUTTON5_GPIO        18
#define BUTTON6_GPIO        21
#define BUTTON7_GPIO        36
#define BUTTON8_GPIO        37
#define BUTTON9_GPIO        38

/* -- Encoder 1 -- AS5600 on I2C port 0 ------------------------------- */
#define ENCODER1_SDA_GPIO   9
#define ENCODER1_SCL_GPIO   10
#define ENCODER1_I2C_PORT   I2C_NUM_0

/* -- Encoder 2 -- AS5600 on I2C port 1 ------------------------------- */
#define ENCODER2_SDA_GPIO   11
#define ENCODER2_SCL_GPIO   12
#define ENCODER2_I2C_PORT   I2C_NUM_1

/* -- I2C bus speed (shared by both encoders) ------------------------- */
#define ENCODER_I2C_FREQ_HZ 400000

/* -- Mode toggle (haptic <-> continuous, active-low, internal pull-up) - */
#define MODE_TOGGLE_GPIO    35

/* -- Status LED (WS2812 on GPIO 48) ---------------------------------- */
#define STATUS_LED_GPIO     48

/* -- LEDC PWM settings (shared by all motor channels) ---------------- */
#define MOTOR_PWM_FREQ_HZ   20000
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_11_BIT  /* 0-2048 duty range */
