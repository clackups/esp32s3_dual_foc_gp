/*
 * haptic.c -- Haptic-detent feedback engine using a M5Stack Roller485.
 *
 * The Roller485 contains an integrated FOC controller, magnetic
 * encoder, and position PID.  This module commands the unit in
 * position mode to generate detent feel: every loop iteration we read
 * the actual position, find the nearest detent and (when it changes)
 * push the new target position over I2C.  The Roller485 then pulls
 * the rotor to that target with a current capped at axis->max_current
 * (register 0x20), which is what gives the haptic "snap" effect.
 */

#include "haptic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

esp_err_t haptic_init(haptic_axis_t *axis, roller485_t *roller,
                      uint16_t steps, int32_t max_current)
{
    if (steps < 2) steps = 2;

    axis->roller          = roller;
    axis->steps           = steps;
    axis->counts_per_step = HAPTIC_COUNTS_PER_REV / (int32_t)steps;
    axis->max_current     = max_current;

    /* Switch the unit off briefly while we change mode and limits.
     * The example sketches in M5Unit-Roller follow the same pattern
     * (setOutput(0) -> setMode -> setPos -> setOutput(1)).             */
    esp_err_t err = roller485_set_output(roller, 0);
    if (err != ESP_OK) return err;

    err = roller485_set_mode(roller, ROLLER485_MODE_POSITION);
    if (err != ESP_OK) return err;

    err = roller485_set_pos_max_current(roller, max_current);
    if (err != ESP_OK) return err;

    /* Capture the current shaft position as detent index 0 so the
     * detent grid lines up with wherever the rotor happens to be at
     * power-up rather than with the absolute encoder zero.            */
    int32_t pos = 0;
    err = roller485_get_pos_readback(roller, &pos);
    if (err != ESP_OK) return err;
    axis->origin_counts = pos;
    axis->last_target   = pos;

    err = roller485_set_pos(roller, pos);
    if (err != ESP_OK) return err;

    return roller485_set_output(roller, 1);
}

/* Snap a raw position to the nearest detent (in absolute counts) and
 * return the corresponding step index modulo axis->steps. */
static int32_t snap_to_detent(const haptic_axis_t *axis, int32_t pos,
                              uint16_t *step_index_out)
{
    int32_t rel = pos - axis->origin_counts;
    /* Round to the nearest counts_per_step (handle negatives). */
    int32_t cps = axis->counts_per_step;
    int32_t nearest;
    if (rel >= 0) {
        nearest = (rel + cps / 2) / cps;
    } else {
        nearest = -((-rel + cps / 2) / cps);
    }
    int32_t snapped = axis->origin_counts + nearest * cps;

    if (step_index_out) {
        int32_t idx = nearest % (int32_t)axis->steps;
        if (idx < 0) idx += (int32_t)axis->steps;
        *step_index_out = (uint16_t)idx;
    }
    return snapped;
}

esp_err_t haptic_update(haptic_axis_t *axis, uint16_t *position)
{
    int32_t pos = 0;
    esp_err_t err = roller485_get_pos_readback(axis->roller, &pos);
    if (err != ESP_OK) return err;

    uint16_t step_idx = 0;
    int32_t target = snap_to_detent(axis, pos, &step_idx);

    if (target != axis->last_target) {
        err = roller485_set_pos(axis->roller, target);
        if (err != ESP_OK) return err;
        axis->last_target = target;
    }

    if (position) *position = step_idx;
    return ESP_OK;
}

esp_err_t haptic_move_to_detent(haptic_axis_t *axis, uint16_t detent)
{
    int32_t target = axis->origin_counts +
                     (int32_t)detent * axis->counts_per_step;
    esp_err_t err = roller485_set_pos(axis->roller, target);
    if (err != ESP_OK) return err;
    axis->last_target = target;
    return ESP_OK;
}

esp_err_t haptic_continuous_update(haptic_axis_t *axis,
                                   int32_t center_counts,
                                   int32_t *raw_counts)
{
    int32_t pos = 0;
    esp_err_t err = roller485_get_pos_readback(axis->roller, &pos);
    if (err != ESP_OK) return err;

    if (raw_counts) *raw_counts = pos;

    if (center_counts != axis->last_target) {
        err = roller485_set_pos(axis->roller, center_counts);
        if (err != ESP_OK) return err;
        axis->last_target = center_counts;
    }
    return ESP_OK;
}
