/*
 * haptic.h — Haptic-detent feedback engine.
 *
 * Each motor axis is divided into a configurable number of equal steps
 * per full rotation.  The engine pulls the rotor toward the nearest
 * detent position with a spring-like restoring torque.
 */

#pragma once

#include "foc.h"

/** Default number of steps per 360° revolution. */
#define HAPTIC_DEFAULT_STEPS 12

/** Maximum normalised torque applied for the detent effect (0 – 1). */
#define HAPTIC_DEFAULT_STRENGTH 0.25f

typedef struct {
    foc_motor_t *motor;
    uint16_t     steps;       /* detent positions per revolution */
    float        strength;    /* peak normalised torque (0 – 1)  */
    float        step_angle;  /* 2π / steps (computed)           */
} haptic_axis_t;

/**
 * Initialise a haptic axis.
 *
 * @param axis     Pointer to haptic_axis_t to initialise.
 * @param motor    An already-initialised and calibrated foc_motor_t.
 * @param steps    Number of detent steps per full rotation.
 * @param strength Peak normalised torque (0 – 1).
 */
void haptic_init(haptic_axis_t *axis, foc_motor_t *motor,
                 uint16_t steps, float strength);

/**
 * Run one tick of the haptic loop: read angle, compute nearest detent,
 * apply restoring torque.
 *
 * Call this at a fixed rate (e.g. every 1 ms from a FreeRTOS task).
 *
 * @param axis  Initialised axis.
 * @param[out] position  If non-NULL, receives the current step index
 *                       (0 … steps−1).
 * @return ESP_OK on success.
 */
esp_err_t haptic_update(haptic_axis_t *axis, uint16_t *position);
