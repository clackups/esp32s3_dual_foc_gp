/*
 * haptic.h -- Haptic-detent feedback engine.
 *
 * Each motor axis is divided into a configurable number of equal steps
 * per full rotation.  The engine pulls the rotor toward the nearest
 * detent position with a spring-like restoring torque.
 */

#pragma once

#include "foc.h"

/** Default number of steps per 360 deg revolution.
 *  Matches FOC_DEFAULT_POLE_PAIRS (7) so that every detent centre
 *  falls at the same electrical angle, giving uniform resistance. */
#define HAPTIC_DEFAULT_STEPS 21

/** Maximum normalised torque applied for the detent effect (0 - 1). */
#define HAPTIC_DEFAULT_STRENGTH 0.80f

/** Default dead-zone expressed as a fraction of one step angle.
 *  Within this zone around a detent centre the rotor is treated as
 *  being at the neutral position and no restoring torque is applied.
 *  Valid range: 0 (disabled) to just below 0.5. */
#define HAPTIC_DEFAULT_DEAD_ZONE 0.00f

/** Default smoothing factor for exponential moving average on torque.
 *  1.0 = no smoothing (raw torque used directly).
 *  Lower values give heavier smoothing (slower response).
 *  Valid range: 0 (exclusive) to 1 (inclusive); values at or below 0
 *  are clamped to 0.01. */
#define HAPTIC_DEFAULT_SMOOTHING_ALPHA 0.7f

typedef struct {
    foc_motor_t *motor;
    uint16_t     steps;            /* detent positions per revolution  */
    float        strength;         /* peak normalised torque (0 - 1)   */
    float        step_angle;       /* 2pi / steps (computed)            */
    float        dead_zone;        /* fraction of step_angle (0-<0.5)  */
    float        smoothing_alpha;  /* EMA factor (0 < alpha <= 1)           */
    float        phase_offset;     /* angular offset for detent centres */
} haptic_axis_t;

/**
 * Initialise a haptic axis.
 *
 * @param axis      Pointer to haptic_axis_t to initialise.
 * @param motor     An already-initialised and calibrated foc_motor_t.
 * @param steps     Number of detent steps per full rotation.
 * @param strength  Peak normalised torque (0 - 1).
 * @param dead_zone Fraction of one step angle that is still treated as
 *                  the neutral (detent-centre) position.  0 disables
 *                  the dead zone; values are clamped below 0.5.
 * @param smoothing_alpha  EMA smoothing factor (0 < alpha <= 1).
 *                         1.0 = no smoothing; values <= 0 are clamped
 *                         to 0.01, values > 1 are clamped to 1.
 */
void haptic_init(haptic_axis_t *axis, foc_motor_t *motor,
                 uint16_t steps, float strength, float dead_zone,
                 float smoothing_alpha);

/**
 * Run one tick of the haptic loop: read angle, compute nearest detent,
 * apply restoring torque.
 *
 * Call this at a fixed rate (e.g. every 1 ms from a FreeRTOS task).
 *
 * @param axis  Initialised axis.
 * @param[out] position  If non-NULL, receives the current step index
 *                       (0 ... steps-1).
 * @param[in,out] prev_torque  Pointer to the previous smoothed torque.
 *                             Read for EMA input, written with the new
 *                             smoothed value.  Caller should initialise
 *                             to 0 before the first call.
 * @return ESP_OK on success.
 */
esp_err_t haptic_update(haptic_axis_t *axis, uint16_t *position,
                        float *prev_torque);

/**
 * Calibrate the detent phase offset.
 *
 * The AS5600 encoder has an arbitrary orientation relative to the motor
 * coils, so the default detent positions (multiples of step_angle
 * starting at 0) may not coincide with the motor's natural cogging
 * positions.  This routine gives the rotor several kicks in alternating
 * directions, lets it coast to rest after each one, and measures where
 * it settles.  The mean rest position (circular average) within one
 * step is stored in axis->phase_offset, shifting every detent so that
 * its centre aligns with the cogging equilibrium.
 *
 * Call after haptic_init() and before the haptic loop starts.
 * Blocks for approximately 2 seconds.
 *
 * @param axis  Initialised axis (motor must be calibrated).
 * @return ESP_OK on success.
 */
esp_err_t haptic_calibrate(haptic_axis_t *axis);

/**
 * Drive the motor to a specific detent position using proportional
 * control, then coast.  Blocks for approximately 500 ms.
 *
 * @param axis    Initialised and calibrated axis.
 * @param detent  Target detent index (0 ... steps-1).
 * @return ESP_OK on success.
 */
esp_err_t haptic_move_to_detent(haptic_axis_t *axis, uint16_t detent);
