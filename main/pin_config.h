/*
 * pin_config.h — GPIO assignments for the dual-FOC gamepad.
 *
 * All hardware pin definitions live here so that porting to a different
 * PCB only requires editing this single file.
 */

#pragma once

/*
 * Motor wiring (2804 BLDC, 3 coil inputs U/V/W):
 *   ENA + IN1/IN2 → coil U  (H-bridge A)
 *   ENB + IN3/IN4 → coil V  (H-bridge B)
 *   Coil W → left floating  (two-phase drive)
 */

/* ── Motor 1 — L298N #1 ───────────────────────────────────────────── */
#define MOTOR1_ENA_GPIO     1   /* Coil U – H-bridge A, enable (PWM) */
#define MOTOR1_IN1_GPIO     2   /* Coil U – H-bridge A, direction    */
#define MOTOR1_IN2_GPIO     3   /* Coil U – H-bridge A, direction    */
#define MOTOR1_ENB_GPIO     4   /* Coil V – H-bridge B, enable (PWM) */
#define MOTOR1_IN3_GPIO     5   /* Coil V – H-bridge B, direction    */
#define MOTOR1_IN4_GPIO     6   /* Coil V – H-bridge B, direction    */

/* ── Motor 2 — L298N #2 ───────────────────────────────────────────── */
#define MOTOR2_ENA_GPIO     7   /* Coil U – H-bridge A, enable (PWM) */
#define MOTOR2_IN1_GPIO     8   /* Coil U – H-bridge A, direction    */
#define MOTOR2_IN2_GPIO     15  /* Coil U – H-bridge A, direction    */
#define MOTOR2_ENB_GPIO     16  /* Coil V – H-bridge B, enable (PWM) */
#define MOTOR2_IN3_GPIO     17  /* Coil V – H-bridge B, direction    */
#define MOTOR2_IN4_GPIO     18  /* Coil V – H-bridge B, direction    */

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
