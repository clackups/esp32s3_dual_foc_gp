/*
 * haptic.h -- Haptic-detent feedback engine.
 *
 * Each motor axis is divided into a configurable number of equal steps
 * per full rotation.  The engine uses a zone-transition algorithm: the
 * motor coasts while the rotor stays within the current zone, and
 * pushes the rotor to the centre of the new zone when a boundary is
 * crossed.  This creates a tactile "click" at each detent transition.
 */

#pragma once

#include "foc.h"
#include <stdbool.h>

/** Default number of steps per 360 deg revolution.
 *  Matches FOC_DEFAULT_POLE_PAIRS (7) so that every detent centre
 *  falls at the same electrical angle, giving uniform resistance. */
#define HAPTIC_DEFAULT_STEPS 21

/** Maximum normalised torque applied for the detent effect (0 - 1). */
#define HAPTIC_DEFAULT_STRENGTH 0.60f

/** Default target zone expressed as a fraction of one step angle.
 *  After the rotor crosses a zone boundary, the motor pushes the rotor
 *  until it is within this fraction of the step angle from the new
 *  detent centre.  Smaller values give a tighter snap; larger values
 *  stop earlier.  Valid range: > 0 to just below 0.5. */
#define HAPTIC_DEFAULT_TARGET_ZONE 0.10f

typedef struct {
    foc_motor_t *motor;
    uint16_t     steps;            /* detent positions per revolution  */
    float        strength;         /* peak normalised torque (0 - 1)   */
    float        step_angle;       /* 2pi / steps (computed)            */
    float        target_zone;      /* fraction of step_angle (>0, <0.5) */
    float        phase_offset;     /* angular offset for detent centres */
    int          target_detent;    /* current zone the rotor has settled in */
    bool         pushing;          /* true while driving to new zone centre */
} haptic_axis_t;

/**
 * Initialise a haptic axis.
 *
 * @param axis      Pointer to haptic_axis_t to initialise.
 * @param motor     An already-initialised and calibrated foc_motor_t.
 * @param steps     Number of detent steps per full rotation.
 * @param strength  Peak normalised torque (0 - 1).
 * @param target_zone  Fraction of one step angle that defines the
 *                     settle area around a detent centre.  After the
 *                     rotor enters a new zone, the motor pushes it
 *                     until it is within this distance from the centre.
 *                     Clamped to 0.01 ... 0.49.
 */
void haptic_init(haptic_axis_t *axis, foc_motor_t *motor,
                 uint16_t steps, float strength, float target_zone);

/**
 * Run one tick of the haptic loop: read angle, detect zone transitions,
 * and push the rotor to the centre of the new zone when a transition
 * occurs.  While the rotor stays inside the current zone, no torque is
 * applied (the motor coasts).  Inspired by the "notchyWheel" algorithm
 * from https://github.com/dmcke5/Hapticpad.
 *
 * Call this at a fixed rate (e.g. every 1 ms from a FreeRTOS task).
 *
 * @param axis  Initialised axis.
 * @param[out] position  If non-NULL, receives the current step index
 *                       (0 ... steps-1).
 * @return ESP_OK on success.
 */
esp_err_t haptic_update(haptic_axis_t *axis, uint16_t *position);

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
