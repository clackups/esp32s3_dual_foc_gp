/*
 * pin_config.h — GPIO assignments for the dual-FOC gamepad.
 *
 * All hardware pin definitions live here so that porting to a different
 * PCB only requires editing this single file.
 */

#pragma once

/* ── Motor 1 — DRV8833 #1 ─────────────────────────────────────────── */
#define MOTOR1_AIN1_GPIO    1   /* Phase-A positive */
#define MOTOR1_AIN2_GPIO    2   /* Phase-A negative */
#define MOTOR1_BIN1_GPIO    3   /* Phase-B positive */
#define MOTOR1_BIN2_GPIO    4   /* Phase-B negative */

/* ── Motor 2 — DRV8833 #2 ─────────────────────────────────────────── */
#define MOTOR2_AIN1_GPIO    5
#define MOTOR2_AIN2_GPIO    6
#define MOTOR2_BIN1_GPIO    7
#define MOTOR2_BIN2_GPIO    8

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
