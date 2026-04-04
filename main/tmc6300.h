/*
 * tmc6300.h -- TMC6300 three-phase BLDC gate driver (3 PWM inputs).
 *
 * The TMC6300 is an integrated three-phase MOSFET driver designed for
 * low-voltage BLDC/PMSM motors.  It provides three half-bridge outputs
 * (U, V, W) with very low RDS(on), requiring only three PWM signals
 * to produce sinusoidal commutation:
 *
 *   UH (PWM) -> Phase U high-side gate
 *   VH (PWM) -> Phase V high-side gate
 *   WH (PWM) -> Phase W high-side gate
 *
 * An optional STANDBY pin puts the chip into low-power mode when the
 * motor is not in use.
 */

#pragma once

#include "driver/ledc.h"
#include "esp_err.h"

typedef struct {
    ledc_channel_t ch_u;      /* LEDC channel - phase U */
    ledc_channel_t ch_v;      /* LEDC channel - phase V */
    ledc_channel_t ch_w;      /* LEDC channel - phase W */
    uint32_t       max_duty;  /* Maximum PWM duty (2^resolution - 1) */
    int            standby_gpio; /* STANDBY pin GPIO, or -1 if unused */
} tmc6300_t;

/**
 * Initialise one TMC6300 (three LEDC PWM channels + optional STANDBY).
 *
 * @param drv          Pointer to an uninitialised tmc6300_t.
 * @param timer        LEDC timer to use (LEDC_TIMER_0 ... 3).
 * @param uh_gpio      UH GPIO (PWM -> phase U high-side).
 * @param vh_gpio      VH GPIO (PWM -> phase V high-side).
 * @param wh_gpio      WH GPIO (PWM -> phase W high-side).
 * @param ch_base      First LEDC channel to use; three consecutive
 *                     channels starting from ch_base will be consumed.
 * @param freq_hz      PWM frequency (e.g. 20 000).
 * @param resolution   LEDC timer resolution (e.g. LEDC_TIMER_11_BIT).
 * @param standby_gpio GPIO connected to STANDBY pin, or -1 if not used.
 *                     When >= 0, the pin is driven HIGH on init (active).
 *                     Use tmc6300_standby()/tmc6300_wake() to manage
 *                     the low-power state explicitly.
 * @return ESP_OK on success.
 */
esp_err_t tmc6300_init(tmc6300_t *drv, ledc_timer_t timer,
                       int uh_gpio, int vh_gpio, int wh_gpio,
                       ledc_channel_t ch_base,
                       uint32_t freq_hz, ledc_timer_bit_t resolution,
                       int standby_gpio);

/**
 * Set unsigned PWM duty for each of the three motor phases.
 *
 * @param duty_u  Duty for phase U (0 ... max_duty).
 * @param duty_v  Duty for phase V (0 ... max_duty).
 * @param duty_w  Duty for phase W (0 ... max_duty).
 */
esp_err_t tmc6300_set_three_phase(const tmc6300_t *drv,
                                  uint32_t duty_u, uint32_t duty_v,
                                  uint32_t duty_w);

/**
 * Coast (all PWM outputs to zero duty).  The driver remains active
 * (STANDBY stays HIGH) so that subsequent set_three_phase() calls
 * take effect immediately.
 */
esp_err_t tmc6300_coast(const tmc6300_t *drv);

/**
 * Enter low-power standby mode.  All PWM outputs are zeroed and the
 * STANDBY GPIO (if configured) is driven LOW.  Call tmc6300_wake()
 * before resuming motor operation.
 */
esp_err_t tmc6300_standby(const tmc6300_t *drv);

/**
 * Wake the driver from standby by driving the STANDBY GPIO HIGH.
 * Has no effect if no STANDBY GPIO was configured.
 */
esp_err_t tmc6300_wake(const tmc6300_t *drv);
