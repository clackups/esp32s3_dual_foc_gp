/*
 * pin_config.h — GPIO assignments for the dual-FOC gamepad.
 *
 * All hardware pin definitions live here so that porting to a different
 * PCB only requires editing this single file.
 */

#pragma once

/*
 * Motor wiring (2804 BLDC, 3 coil inputs U/V/W):
 *   Coil U → AOUT1–AOUT2  (H-bridge A)
 *   Coil V → BOUT1–BOUT2  (H-bridge B)
 *   Coil W → left floating (two-phase drive)
 */

/* ── Motor 1 — DRV8833 #1 ─────────────────────────────────────────── */
#define MOTOR1_AIN1_GPIO    1   /* Coil U – H-bridge A, IN1 */
#define MOTOR1_AIN2_GPIO    2   /* Coil U – H-bridge A, IN2 */
#define MOTOR1_BIN1_GPIO    3   /* Coil V – H-bridge B, IN1 */
#define MOTOR1_BIN2_GPIO    4   /* Coil V – H-bridge B, IN2 */

/* ── Motor 2 — DRV8833 #2 ─────────────────────────────────────────── */
#define MOTOR2_AIN1_GPIO    5   /* Coil U – H-bridge A, IN1 */
#define MOTOR2_AIN2_GPIO    6   /* Coil U – H-bridge A, IN2 */
#define MOTOR2_BIN1_GPIO    7   /* Coil V – H-bridge B, IN1 */
#define MOTOR2_BIN2_GPIO    8   /* Coil V – H-bridge B, IN2 */

/* ── Encoder 1 — AS5600 on I2C port 0 ─────────────────────────────── */
#define ENCODER1_SDA_GPIO   9
#define ENCODER1_SCL_GPIO   10
#define ENCODER1_I2C_PORT   I2C_NUM_0

/* ── Encoder 2 — AS5600 on I2C port 1 ─────────────────────────────── */
#define ENCODER2_SDA_GPIO   11
#define ENCODER2_SCL_GPIO   12
#define ENCODER2_I2C_PORT   I2C_NUM_1

/* ── I2C bus speed (shared by both encoders) ───────────────────────── */
#define ENCODER_I2C_FREQ_HZ 400000

/* ── LEDC PWM settings (shared by all motor channels) ──────────────── */
#define MOTOR_PWM_FREQ_HZ   20000
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_10_BIT  /* 0-1023 duty range */
