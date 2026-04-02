/*
 * haptic.c -- Haptic-detent feedback engine.
 */

#include "haptic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

/* -- Haptic-detent feedback engine ---------------------------------- */

/** Minimum settle threshold as a fraction of one step angle.
 *  When the dead-zone is smaller than this value, the settle check
 *  uses this fraction instead so that the motor does not hunt
 *  indefinitely around the zone centre. */
#define HAPTIC_MIN_SETTLE_FRAC 0.05f

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
    axis->phase_offset   = 0.0f;
    axis->target_detent  = 0;
    axis->pushing        = false;
}

esp_err_t haptic_update(haptic_axis_t *axis, uint16_t *position,
                        float *prev_torque)
{
    float angle;
    esp_err_t err = foc_read_angle(axis->motor, &angle);
    if (err != ESP_OK) return err;

    /* Nearest detent index from the current angle. */
    float shifted     = angle - axis->phase_offset;
    float idx_f       = shifted / axis->step_angle;
    int   nearest_idx = (int)roundf(idx_f);

    /*
     * Zone-transition algorithm (inspired by the Hapticpad notchyWheel):
     *
     * While the rotor stays in the current zone (nearest_idx ==
     * target_detent) no torque is applied -- the motor coasts and the
     * user can move the knob freely.
     *
     * Once the rotor crosses into a neighbouring zone the target is
     * updated and the motor pushes the rotor toward the centre of
     * the new zone (pushing = true).  The push continues until the
     * rotor is close enough to the new centre (within the settle
     * threshold), at which point pushing is cleared and the motor
     * coasts again.
     */
    if (nearest_idx != axis->target_detent) {
        axis->target_detent = nearest_idx;
        axis->pushing       = true;
    }

    float torque;

    if (axis->pushing) {
        float detent_angle = (float)axis->target_detent * axis->step_angle
                           + axis->phase_offset;

        /* Signed angular error from the zone centre. */
        float delta = detent_angle - angle;
        if (delta >  (float)M_PI) delta -= 2.0f * (float)M_PI;
        if (delta < -(float)M_PI) delta += 2.0f * (float)M_PI;

        float abs_delta = (delta >= 0.0f) ? delta : -delta;

        /*
         * Settle threshold: use the dead-zone if configured, otherwise
         * 5% of one step angle so the motor does not hunt forever.
         */
        float settle = axis->dead_zone * axis->step_angle;
        if (settle < HAPTIC_MIN_SETTLE_FRAC * axis->step_angle)
            settle = HAPTIC_MIN_SETTLE_FRAC * axis->step_angle;

        if (abs_delta <= settle) {
            /* Close enough to centre -- stop pushing, coast. */
            axis->pushing = false;
            torque = 0.0f;
        } else {
            /* Proportional torque toward zone centre. */
            float active_range = axis->step_angle * 0.5f;
            float gain = (active_range > 1e-6f)
                       ? axis->strength / active_range
                       : 0.0f;
            float sign = (delta >= 0.0f) ? 1.0f : -1.0f;
            torque = gain * abs_delta * sign;
            if (torque >  axis->strength) torque =  axis->strength;
            if (torque < -axis->strength) torque = -axis->strength;
        }
    } else {
        /* Inside current zone -- coast (no motor engagement). */
        torque = 0.0f;
    }

    /*
     * Exponential moving average smoothing:
     *   smoothed = alpha * raw + (1 - alpha) * previous
     * alpha = 1 bypasses smoothing entirely.
     */
    if (prev_torque) {
        torque = axis->smoothing_alpha * torque
               + (1.0f - axis->smoothing_alpha) * (*prev_torque);
        *prev_torque = torque;
    }

    err = foc_set_torque(axis->motor, torque);
    if (err != ESP_OK) return err;

    if (position) {
        int idx = axis->target_detent % (int)axis->steps;
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

    /* Seed the zone-transition state so that the haptic loop starts
     * in coast mode at the detent we just moved to.                 */
    axis->target_detent = (int)detent;
    axis->pushing       = false;

    return foc_coast(axis->motor);
}
