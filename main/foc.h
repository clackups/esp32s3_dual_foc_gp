/*
 * foc.h — Three-phase field-oriented control for BLDC motors
 *         driven through Mini L298N boards (PWM on IN1–IN3).
 */

#pragma once

#include "as5600.h"
#include "l298n.h"
#include <stdint.h>

/** Number of magnetic pole pairs.  The 2804 BLDC motor (14 poles /
 *  12 slots) has 7 pole pairs. */
#define FOC_DEFAULT_POLE_PAIRS 7

/** Number of bins in the encoder-nonlinearity correction table.
 *  Populated by foc_calibrate() when it sweeps one mechanical revolution. */
#define FOC_CAL_TABLE_SIZE 128

typedef struct {
    as5600_t *encoder;
    l298n_t  *driver;
    uint8_t   pole_pairs;
    float     zero_electrical_angle;            /* calibration offset (rad) */
    float     cal_table[FOC_CAL_TABLE_SIZE];    /* per-bin elec. angle correction */
} foc_motor_t;

/**
 * Initialise and link an encoder + driver pair.
 * Call after as5600_init() and l298n_init().
 */
void foc_init(foc_motor_t *motor, as5600_t *encoder, l298n_t *driver,
              uint8_t pole_pairs);

/**
 * Calibrate the motor: find the electrical zero, then sweep one full
 * mechanical revolution forward and backward to build a correction
 * table that compensates for encoder nonlinearity.  The bidirectional
 * sweep cancels any directional bias from motor inertia.  The motor
 * will energise for several seconds — keep the shaft unloaded.
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
 * Coast the motor (de-energise all phases).
 */
esp_err_t foc_coast(foc_motor_t *motor);

/**
 * Read the current mechanical angle in radians.
 */
esp_err_t foc_read_angle(const foc_motor_t *motor, float *angle_rad);
