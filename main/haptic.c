/*
 * haptic.c — Haptic-detent feedback engine.
 */

#include "haptic.h"
#include <math.h>

void haptic_init(haptic_axis_t *axis, foc_motor_t *motor,
                 uint16_t steps, float strength, float dead_zone,
                 float smoothing_alpha)
{
    axis->motor      = motor;
    axis->steps      = (steps >= 2) ? steps : 2;
    axis->strength   = strength;
    axis->step_angle = 2.0f * (float)M_PI / (float)axis->steps;
    axis->dead_zone  = (dead_zone < 0.0f) ? 0.0f
                     : (dead_zone > 0.49f) ? 0.49f
                     : dead_zone;
    axis->smoothing_alpha = (smoothing_alpha <= 0.0f) ? 0.01f
                          : (smoothing_alpha > 1.0f)  ? 1.0f
                          : smoothing_alpha;
}

esp_err_t haptic_update(haptic_axis_t *axis, uint16_t *position,
                        float *prev_torque)
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
     * A dead-zone around the detent centre allows a small region where
     * no restoring torque is applied (neutral position).
     *
     * Outside the dead zone the torque ramps from zero at the dead-zone
     * boundary to ±strength at half a step-angle.
     */
    float dz_rad = axis->dead_zone * axis->step_angle;
    float abs_delta = (delta >= 0.0f) ? delta : -delta;
    float torque;

    if (abs_delta <= dz_rad) {
        torque = 0.0f;
    } else {
        float active_range = axis->step_angle * 0.5f - dz_rad;
        float gain = (active_range > 1e-6f)
                   ? axis->strength / active_range
                   : 0.0f;
        float sign = (delta >= 0.0f) ? 1.0f : -1.0f;
        torque = gain * (abs_delta - dz_rad) * sign;
        if (torque >  axis->strength) torque =  axis->strength;
        if (torque < -axis->strength) torque = -axis->strength;
    }

    /*
     * Exponential moving average smoothing:
     *   smoothed = α · raw + (1 − α) · previous
     * α = 1 bypasses smoothing entirely.
     */
    if (prev_torque) {
        torque = axis->smoothing_alpha * torque
               + (1.0f - axis->smoothing_alpha) * (*prev_torque);
        *prev_torque = torque;
    }

    err = foc_set_torque(axis->motor, torque);
    if (err != ESP_OK) return err;

    if (position) {
        int idx = nearest_idx % (int)axis->steps;
        if (idx < 0) idx += (int)axis->steps;
        *position = (uint16_t)idx;
    }
    return ESP_OK;
}
