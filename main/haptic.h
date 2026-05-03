/*
 * haptic.h -- Haptic-detent feedback engine using a M5Stack Roller485.
 *
 * Each motor axis is divided into a configurable number of equal steps
 * per full rotation.  The Roller485 is run in position mode and the
 * target position is continuously snapped to the nearest detent so
 * that the device's internal PID + current limiter pulls the rotor
 * toward that detent with a configurable maximum current.
 */

#pragma once

#include "roller485.h"
#include "sdkconfig.h"
#include <stdint.h>

/** Number of position counts in one full revolution.
 *  The Roller485 uses 0.01 deg position units, so 360 * 100 = 36000. */
#define HAPTIC_COUNTS_PER_REV 36000

/** Default number of detent steps per revolution.  Sourced from the
 *  user-visible "Dual-FOC GP" Kconfig menu. */
#define HAPTIC_DEFAULT_STEPS  CONFIG_DFGP_HAPTIC_DEFAULT_STEPS

/** Default maximum current limit (Roller485 register 0x20).
 *  Sourced from the "Dual-FOC GP" Kconfig menu. */
#define HAPTIC_DEFAULT_MAX_CURRENT CONFIG_DFGP_HAPTIC_MAX_CURRENT

typedef struct {
    roller485_t *roller;
    uint16_t     steps;             /* detent positions per revolution  */
    int32_t      counts_per_step;   /* HAPTIC_COUNTS_PER_REV / steps    */
    int32_t      max_current;       /* Roller485 register 0x20 value    */
    int32_t      origin_counts;     /* zero-step position (counts)      */
    int32_t      last_target;       /* last position written to roller  */
} haptic_axis_t;

/**
 * Initialise a haptic axis.
 *
 * Configures the Roller485 for closed-loop position mode with the
 * requested maximum current, captures the current shaft position as
 * the zero-step origin, and commands the axis to remain there.
 *
 * @param axis        Pointer to haptic_axis_t to initialise.
 * @param roller      Already-initialised Roller485 unit.
 * @param steps       Number of detent steps per full rotation (>= 2).
 * @param max_current Maximum current limit (Roller485 register 0x20).
 * @return ESP_OK on success.
 */
esp_err_t haptic_init(haptic_axis_t *axis, roller485_t *roller,
                      uint16_t steps, int32_t max_current);

/**
 * Run one tick of the haptic loop: read actual position, snap to the
 * nearest detent, command the Roller485 to that position if changed.
 *
 * @param axis      Initialised axis.
 * @param[out] position  If non-NULL, receives the current step index
 *                       (0 ... steps-1).
 * @return ESP_OK on success.
 */
esp_err_t haptic_update(haptic_axis_t *axis, uint16_t *position);

/**
 * Drive the motor to a specific detent position.  The Roller485
 * internal controller handles the trajectory, so this call only
 * issues the position target and returns immediately.
 *
 * @param axis    Initialised axis.
 * @param detent  Target detent index (0 ... steps-1).
 * @return ESP_OK on success.
 */
esp_err_t haptic_move_to_detent(haptic_axis_t *axis, uint16_t detent);

/**
 * Run one tick of the continuous centering loop.
 *
 * The Roller485 is held at the centre angle (in counts) by its
 * internal position PID.  The current limit caps the restoring force.
 *
 * @param axis             Initialised axis.
 * @param center_counts    Centre position (Roller485 0.01 deg units).
 * @param[out] raw_counts  If non-NULL, receives the current actual
 *                         position in 0.01 deg units.
 * @return ESP_OK on success.
 */
esp_err_t haptic_continuous_update(haptic_axis_t *axis,
                                   int32_t center_counts,
                                   int32_t *raw_counts);
