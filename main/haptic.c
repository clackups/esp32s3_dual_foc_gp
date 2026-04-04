/*
 * haptic.c -- Haptic-detent feedback engine.
 */

#include "haptic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

void haptic_init(haptic_axis_t *axis, foc_motor_t *motor,
                 uint16_t steps, float strength, float dead_zone)
{
    axis->motor      = motor;
    axis->steps      = (steps >= 2) ? steps : 2;
    axis->strength   = strength;
    axis->step_angle = 2.0f * (float)M_PI / (float)axis->steps;
    axis->dead_zone  = (dead_zone < 0.0f) ? 0.0f
                     : (dead_zone > 0.49f) ? 0.49f
                     : dead_zone;
    axis->phase_offset    = 0.0f;
    axis->cont_ref_tick   = 0;
    axis->cont_kicking    = 0;
    axis->cont_kick_start = 0;
}

esp_err_t haptic_update(haptic_axis_t *axis, uint16_t *position)
{
    float angle;
    esp_err_t err = foc_read_angle(axis->motor, &angle);
    if (err != ESP_OK) return err;

    /* Nearest detent index and its centre angle. */
    float shifted     = angle - axis->phase_offset;
    float idx_f       = shifted / axis->step_angle;
    int   nearest_idx = (int)roundf(idx_f);
    float detent_angle = (float)nearest_idx * axis->step_angle + axis->phase_offset;

    /* Signed angular error from the detent centre. */
    float delta = detent_angle - angle;

    /* Wrap to -pi ... +pi (in practice delta is small). */
    if (delta >  (float)M_PI) delta -= 2.0f * (float)M_PI;
    if (delta < -(float)M_PI) delta += 2.0f * (float)M_PI;

    /*
     * Torque is proportional to the error, clamped to +/-strength.
     * A dead-zone around the detent centre allows a small region where
     * no restoring torque is applied (neutral position).
     *
     * Outside the dead zone the torque ramps from zero at the dead-zone
     * boundary to +/-strength at half a step-angle.
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

    err = foc_set_torque(axis->motor, torque);
    if (err != ESP_OK) return err;

    if (position) {
        int idx = nearest_idx % (int)axis->steps;
        if (idx < 0) idx += (int)axis->steps;
        *position = (uint16_t)idx;
    }
    return ESP_OK;
}

/* -- Haptic calibration -------------------------------------------- */

/** Number of kick-settle measurements during calibration. */
#define HAPTIC_CAL_SAMPLES   4

/** Normalised torque amplitude for each calibration kick. */
#define HAPTIC_CAL_KICK      0.3f

/** Duration of each calibration kick (ms). */
#define HAPTIC_CAL_KICK_MS   150

/** Settle time after coasting (ms). */
#define HAPTIC_CAL_SETTLE_MS 300

esp_err_t haptic_calibrate(haptic_axis_t *axis)
{
    float two_pi  = 2.0f * (float)M_PI;
    float sum_sin = 0.0f;
    float sum_cos = 0.0f;

    for (int i = 0; i < HAPTIC_CAL_SAMPLES; i++) {
        /*
         * Alternating kick direction moves the rotor to different
         * cogging positions so we sample more than one resting spot.
         */
        float kick = (i % 2 == 0) ? HAPTIC_CAL_KICK : -HAPTIC_CAL_KICK;

        esp_err_t err = foc_set_torque(axis->motor, kick);
        if (err != ESP_OK) return err;
        vTaskDelay(pdMS_TO_TICKS(HAPTIC_CAL_KICK_MS));

        /*
         * Coast the motor -- let it settle at the nearest cogging
         * position under the rotor magnets' natural pull.
         */
        err = foc_coast(axis->motor);
        if (err != ESP_OK) return err;
        vTaskDelay(pdMS_TO_TICKS(HAPTIC_CAL_SETTLE_MS));

        float rest;
        err = foc_read_angle(axis->motor, &rest);
        if (err != ESP_OK) return err;

        /*
         * Convert the rest angle to a phase within one step
         * (0 ... 2pi), then accumulate sin/cos for circular averaging.
         */
        float rem = fmodf(rest, axis->step_angle);
        if (rem < 0.0f) rem += axis->step_angle;
        float phase = rem / axis->step_angle * two_pi;
        sum_sin += sinf(phase);
        sum_cos += cosf(phase);
    }

    /* Circular mean of the sampled rest phases. */
    float mean_phase = atan2f(sum_sin, sum_cos);
    if (mean_phase < 0.0f) mean_phase += two_pi;

    axis->phase_offset = mean_phase / two_pi * axis->step_angle;

    return ESP_OK;
}

/* -- Move to target detent ------------------------------------------ */

/** Duration of the proportional-control settle loop (ms). */
#define HAPTIC_MOVE_SETTLE_MS 500

esp_err_t haptic_move_to_detent(haptic_axis_t *axis, uint16_t detent)
{
    float target = (float)detent * axis->step_angle + axis->phase_offset;
    float gain   = axis->strength / (axis->step_angle * 0.5f);

    for (int i = 0; i < HAPTIC_MOVE_SETTLE_MS; i++) {
        float angle;
        esp_err_t err = foc_read_angle(axis->motor, &angle);
        if (err != ESP_OK) return err;

        float error = target - angle;
        if (error >  (float)M_PI) error -= 2.0f * (float)M_PI;
        if (error < -(float)M_PI) error += 2.0f * (float)M_PI;

        float torque = error * gain;
        if (torque >  axis->strength) torque =  axis->strength;
        if (torque < -axis->strength) torque = -axis->strength;

        err = foc_set_torque(axis->motor, torque);
        if (err != ESP_OK) return err;

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return foc_coast(axis->motor);
}

/* -- Continuous centering mode --------------------------------------- */

/** Angular threshold below which the rotor is considered stationary
 *  for stuck detection (~1 degree). */
#define CONT_STUCK_RAD      0.02f

/** Time the rotor must be stuck before an anti-cogging kick (ms). */
#define CONT_STUCK_MS       300

/** Duration of the anti-cogging kick (ms). */
#define CONT_KICK_MS        30

/** Normalised torque applied during the anti-cogging kick. */
#define CONT_KICK_TORQUE    1.0f

esp_err_t haptic_continuous_update(haptic_axis_t *axis,
                                   float center_angle,
                                   float half_range,
                                   float dead_zone,
                                   float initial_force,
                                   float max_force,
                                   float *raw_angle)
{
    float angle;
    esp_err_t err = foc_read_angle(axis->motor, &angle);
    if (err != ESP_OK) return err;

    if (raw_angle) *raw_angle = angle;

    /* Signed error from centre, wrapped to -pi ... +pi. */
    float error = center_angle - angle;
    if (error >  (float)M_PI) error -= 2.0f * (float)M_PI;
    if (error < -(float)M_PI) error += 2.0f * (float)M_PI;

    /* Normalise to [-1, +1] and clamp. */
    float norm = (half_range > 1e-6f) ? (error / half_range) : 0.0f;
    if (norm >  1.0f) norm =  1.0f;
    if (norm < -1.0f) norm = -1.0f;

    float abs_norm = fabsf(norm);
    float torque;

    /* Clamp dead_zone to a safe range. */
    if (dead_zone < 0.0f) dead_zone = 0.0f;
    if (dead_zone > 0.99f) dead_zone = 0.99f;

    if (abs_norm <= dead_zone) {
        /* Inside the dead zone -- no restoring force. */
        torque = 0.0f;
    } else {
        /* Linear ramp from initial_force at the dead-zone edge to
         * max_force at |norm| == 1 (full half_range).
         *
         *   active_range = 1 - dead_zone
         *   t = (|norm| - dead_zone) / active_range   (0 at edge, 1 at max)
         *   force = initial_force + t * (max_force - initial_force)        */
        float active_range = 1.0f - dead_zone;
        float t = (abs_norm - dead_zone) / active_range;
        float force = initial_force + t * (max_force - initial_force);
        float sign  = (norm >= 0.0f) ? 1.0f : -1.0f;
        torque = force * sign;
    }

    /* Anti-cogging: if the rotor is stuck outside the dead zone for
     * longer than CONT_STUCK_MS, apply a brief full-torque kick toward
     * the centre to break free from the cogging equilibrium.          */
    uint32_t now = (uint32_t)xTaskGetTickCount();

    if (abs_norm <= dead_zone) {
        /* At centre -- reset stuck detection. */
        axis->cont_ref_tick = 0;
        axis->cont_kicking  = 0;
    } else if (axis->cont_kicking) {
        if ((now - axis->cont_kick_start) < pdMS_TO_TICKS(CONT_KICK_MS)) {
            /* Kick in progress -- override with full torque. */
            torque = (norm >= 0.0f) ? CONT_KICK_TORQUE : -CONT_KICK_TORQUE;
        } else {
            /* Kick finished -- reset for fresh stuck detection. */
            axis->cont_kicking = 0;
            axis->cont_ref_tick = 0;
        }
    } else {
        /* Normal centering -- check if rotor is stuck. */
        float diff = angle - axis->cont_ref_angle;
        if (diff >  (float)M_PI) diff -= 2.0f * (float)M_PI;
        if (diff < -(float)M_PI) diff += 2.0f * (float)M_PI;

        if (axis->cont_ref_tick == 0 || fabsf(diff) > CONT_STUCK_RAD) {
            /* Not tracking or rotor moved -- (re)start stuck timer. */
            axis->cont_ref_angle = angle;
            axis->cont_ref_tick  = now | 1u;  /* ensure non-zero */
        } else if ((now - axis->cont_ref_tick) >=
                   pdMS_TO_TICKS(CONT_STUCK_MS)) {
            /* Stuck too long -- start anti-cogging kick. */
            axis->cont_kicking    = 1;
            axis->cont_kick_start = now;
            torque = (norm >= 0.0f) ? CONT_KICK_TORQUE : -CONT_KICK_TORQUE;
        }
    }

    return foc_set_torque(axis->motor, torque);
}
