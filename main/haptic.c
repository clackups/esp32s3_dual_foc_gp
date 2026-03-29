/*
 * haptic.c — Haptic-detent feedback engine.
 */

#include "haptic.h"
#include <math.h>

void haptic_init(haptic_axis_t *axis, foc_motor_t *motor,
                 uint16_t steps, float strength)
{
    axis->motor      = motor;
    axis->steps      = steps;
    axis->strength   = strength;
    axis->step_angle = 2.0f * (float)M_PI / (float)steps;
}

esp_err_t haptic_update(haptic_axis_t *axis, uint16_t *position)
{
    float angle;
    esp_err_t err = foc_read_angle(axis->motor, &angle);
    if (err != ESP_OK) return err;

    /* Nearest detent index and its centre angle. */
    float idx_f      = angle / axis->step_angle;
    int   nearest_idx = (int)roundf(idx_f);
    float detent_angle = (float)nearest_idx * axis->step_angle;

    /* Signed angular error from the detent centre. */
    float delta = detent_angle - angle;

    /* Wrap to −π … +π (in practice delta is small). */
    if (delta >  (float)M_PI) delta -= 2.0f * (float)M_PI;
    if (delta < -(float)M_PI) delta += 2.0f * (float)M_PI;

    /*
     * Torque is proportional to the error, clamped to ±strength.
     * Gain is chosen so that at half a step-angle the torque reaches
     * the configured strength.
     */
    float gain   = axis->strength / (axis->step_angle * 0.5f);
    float torque = gain * delta;
    if (torque >  axis->strength) torque =  axis->strength;
    if (torque < -axis->strength) torque = -axis->strength;

    err = foc_set_torque(axis->motor, torque);
    if (err != ESP_OK) return err;

    if (position) {
        int idx = nearest_idx % (int)axis->steps;
        if (idx < 0) idx += (int)axis->steps;
        *position = (uint16_t)idx;
    }
    return ESP_OK;
}
