/*
 * tmc6300.h -- TMC6300 three-phase BLDC gate driver (3 PWM inputs).
 *
 * The TMC6300 is an integrated three-phase MOSFET driver designed for
 * low-voltage BLDC/PMSM motors.  It provides three half-bridge outputs
 * (U, V, W) with very low RDS(on).
 *
 * In 3-PWM mode only the high-side inputs receive PWM; the low-side
 * inputs (UL, VL, WL) and VIO are tied permanently to +3.3 V so the
 * complementary low-side FETs conduct whenever the corresponding
 * high-side FET is off, completing the current path through each
 * motor phase.  No GPIO pins are needed for UL/VL/WL or VIO.
 *
 *   UH (PWM)  -> Phase U high-side gate
 *   VH (PWM)  -> Phase V high-side gate
 *   WH (PWM)  -> Phase W high-side gate
 *   UL, VL, WL -> +3.3 V (always on)
 *   VIO        -> +3.3 V (always active)
 */

#pragma once

#include "driver/ledc.h"
#include "esp_err.h"

typedef struct {
    ledc_channel_t ch_u;      /* LEDC channel - phase U */
    ledc_channel_t ch_v;      /* LEDC channel - phase V */
    ledc_channel_t ch_w;      /* LEDC channel - phase W */
    uint32_t       max_duty;  /* Maximum PWM duty (2^resolution - 1) */
} tmc6300_t;

/**
 * Initialise one TMC6300 (three LEDC PWM channels).
 *
 * UL/VL/WL and VIO are assumed to be hardwired to +3.3 V on the PCB,
 * so no GPIO pins are needed for low-side enables or standby control.
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
 * @return ESP_OK on success.
 */
esp_err_t tmc6300_init(tmc6300_t *drv, ledc_timer_t timer,
                       int uh_gpio, int vh_gpio, int wh_gpio,
                       ledc_channel_t ch_base,
                       uint32_t freq_hz, ledc_timer_bit_t resolution);

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
 * Coast (all PWM outputs to 50 % duty -- neutral, zero torque).
 *
 * With UL/VL/WL hardwired to +3.3 V, setting duty to 0 would turn on
 * all low-side FETs (braking) and may trigger the TMC6300's overcurrent
 * protection.  50 % keeps each phase at Vmotor / 2 on average so no
 * inter-phase current flows.
 */
esp_err_t tmc6300_coast(const tmc6300_t *drv);
