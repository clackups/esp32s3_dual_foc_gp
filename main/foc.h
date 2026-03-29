/*
 * foc.h — Simplified two-phase field-oriented control for BLDC motors
 *         driven through DRV8833 dual-H-bridge drivers.
 */

#pragma once

#include "as5600.h"
#include "drv8833.h"
#include <stdint.h>

/** Number of magnetic pole pairs in the BLDC motor.  Adjust to match
 *  the actual motor.  A typical small gimbal motor has 7 pole pairs. */
#define FOC_DEFAULT_POLE_PAIRS 7

typedef struct {
    as5600_t  *encoder;
    drv8833_t *driver;
    uint8_t    pole_pairs;
    float      zero_electrical_angle; /* calibration offset (radians) */
} foc_motor_t;

/**
 * Initialise and link an encoder + driver pair.
 * Call after as5600_init() and drv8833_init().
 */
void foc_init(foc_motor_t *motor, as5600_t *encoder, drv8833_t *driver,
              uint8_t pole_pairs);

/**
 * Run a simple open-loop alignment to find the electrical zero.
 * The motor will briefly energise — keep the shaft unloaded.
 */
esp_err_t foc_calibrate(foc_motor_t *motor);

/**
 * Apply a torque command to the motor.
 *
 * @param motor   Initialised & calibrated motor.
 * @param torque  Normalised torque (−1.0 … +1.0).
 * @return ESP_OK on success.
 */
esp_err_t foc_set_torque(foc_motor_t *motor, float torque);

/**
 * Read the current mechanical angle in radians.
 */
esp_err_t foc_read_angle(const foc_motor_t *motor, float *angle_rad);
