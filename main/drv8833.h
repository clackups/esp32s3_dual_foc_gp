/*
 * drv8833.h — DRV8833 dual-H-bridge motor driver (LEDC PWM).
 *
 * Each DRV8833 instance controls one 2804 BLDC motor (3 coil inputs
 * U/V/W) using its two H-bridge channels (A → coil U, B → coil V).
 * Coil W is left floating; the two driven phases produce a sinusoidal
 * field that is sufficient for haptic-feedback torque control.
 */

#pragma once

#include "driver/ledc.h"
#include "esp_err.h"

typedef struct {
    ledc_channel_t ch_a1;   /* AIN1 LEDC channel */
    ledc_channel_t ch_a2;   /* AIN2 LEDC channel */
    ledc_channel_t ch_b1;   /* BIN1 LEDC channel */
    ledc_channel_t ch_b2;   /* BIN2 LEDC channel */
    uint32_t       max_duty; /* Maximum PWM duty (2^resolution − 1) */
} drv8833_t;

/**
 * Initialise one DRV8833 (four LEDC PWM channels).
 *
 * @param drv          Pointer to an uninitialised drv8833_t.
 * @param timer        LEDC timer to use (LEDC_TIMER_0 … 3).
 * @param ain1_gpio    AIN1 GPIO.
 * @param ain2_gpio    AIN2 GPIO.
 * @param bin1_gpio    BIN1 GPIO.
 * @param bin2_gpio    BIN2 GPIO.
 * @param ch_base      First LEDC channel to use; four consecutive
 *                     channels starting from ch_base will be consumed.
 * @param freq_hz      PWM frequency (e.g. 20 000).
 * @param resolution   LEDC timer resolution (e.g. LEDC_TIMER_10_BIT).
 * @return ESP_OK on success.
 */
esp_err_t drv8833_init(drv8833_t *drv, ledc_timer_t timer,
                       int ain1_gpio, int ain2_gpio,
                       int bin1_gpio, int bin2_gpio,
                       ledc_channel_t ch_base,
                       uint32_t freq_hz, ledc_timer_bit_t resolution);

/**
 * Set raw duty for each phase.
 *
 * @param phase_a  Signed duty for H-bridge A (−max_duty … +max_duty).
 * @param phase_b  Signed duty for H-bridge B (−max_duty … +max_duty).
 */
esp_err_t drv8833_set_phase(const drv8833_t *drv,
                            int32_t phase_a, int32_t phase_b);

/** Coast (all outputs LOW). */
esp_err_t drv8833_coast(const drv8833_t *drv);
