/*
 * pin_config.h -- GPIO assignments for the dual-FOC gamepad.
 *
 * All hardware pin definitions live here so that porting to a different
 * PCB only requires editing this single file.
 */

#pragma once

/*
 * Motor wiring (2804 BLDC, 3 coil inputs U/V/W):
 *   Mini L298N board -- no ENA/ENB; PWM goes directly on IN1-IN3.
 *   IN1 (PWM) -> OUT1 -> Coil U
 *   IN2 (PWM) -> OUT2 -> Coil V
 *   IN3 (PWM) -> OUT3 -> Coil W
 */

/* -- Motor 1 -- Mini L298N #1 ---------------------------------------- */
#define MOTOR1_IN1_GPIO     1   /* Coil U - PWM */
#define MOTOR1_IN2_GPIO     2   /* Coil V - PWM */
#define MOTOR1_IN3_GPIO     3   /* Coil W - PWM */

/* -- Motor 2 -- Mini L298N #2 ---------------------------------------- */
#define MOTOR2_IN1_GPIO     15  /* Coil U - PWM */
#define MOTOR2_IN2_GPIO     16  /* Coil V - PWM */
#define MOTOR2_IN3_GPIO     17  /* Coil W - PWM */

/* -- Game controller buttons (active-low, internal pull-up) ---------- */
#define BUTTON_COUNT        10
#define BUTTON1_GPIO        4
#define BUTTON2_GPIO        5
#define BUTTON3_GPIO        13
#define BUTTON4_GPIO        14
#define BUTTON5_GPIO        6
#define BUTTON6_GPIO        7
#define BUTTON7_GPIO        8
#define BUTTON8_GPIO        18
#define BUTTON9_GPIO        21
#define BUTTON10_GPIO       38

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

/* -- Status LED (WS2812 on GPIO 48) ---------------------------------- */
#define STATUS_LED_GPIO     48

/* -- LEDC PWM settings (shared by all motor channels) ---------------- */
#define MOTOR_PWM_FREQ_HZ   20000
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_10_BIT  /* 0-1023 duty range */
