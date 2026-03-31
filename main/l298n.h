/*
 * l298n.h — Mini L298N motor driver (3-phase sinusoidal PWM).
 *
 * The Mini L298N board exposes only IN1–IN4 (no ENA/ENB).  Each INx
 * pin directly drives the corresponding OUTx via PWM.  Three of the
 * four outputs drive the three coils of a 2804 BLDC motor:
 *
 *   IN1 (PWM) → OUT1 → Coil U
 *   IN2 (PWM) → OUT2 → Coil V
 *   IN3 (PWM) → OUT3 → Coil W
 */

#pragma once

#include "driver/ledc.h"
#include "esp_err.h"

typedef struct {
    ledc_channel_t ch_in1;   /* IN1 LEDC channel – phase U */
    ledc_channel_t ch_in2;   /* IN2 LEDC channel – phase V */
    ledc_channel_t ch_in3;   /* IN3 LEDC channel – phase W */
    uint32_t       max_duty; /* Maximum PWM duty (2^resolution − 1) */
} l298n_t;

/**
 * Initialise one Mini L298N (three LEDC PWM channels).
 *
 * @param drv          Pointer to an uninitialised l298n_t.
 * @param timer        LEDC timer to use (LEDC_TIMER_0 … 3).
 * @param in1_gpio     IN1 GPIO (PWM → coil U).
 * @param in2_gpio     IN2 GPIO (PWM → coil V).
 * @param in3_gpio     IN3 GPIO (PWM → coil W).
 * @param ch_base      First LEDC channel to use; three consecutive
 *                     channels starting from ch_base will be consumed.
 * @param freq_hz      PWM frequency (e.g. 20 000).
 * @param resolution   LEDC timer resolution (e.g. LEDC_TIMER_10_BIT).
 * @return ESP_OK on success.
 */
esp_err_t l298n_init(l298n_t *drv, ledc_timer_t timer,
                     int in1_gpio, int in2_gpio, int in3_gpio,
                     ledc_channel_t ch_base,
                     uint32_t freq_hz, ledc_timer_bit_t resolution);

/**
 * Set unsigned PWM duty for each of the three motor phases.
 *
 * @param duty_u  Duty for coil U (0 … max_duty).
 * @param duty_v  Duty for coil V (0 … max_duty).
 * @param duty_w  Duty for coil W (0 … max_duty).
 */
esp_err_t l298n_set_three_phase(const l298n_t *drv,
                                uint32_t duty_u, uint32_t duty_v,
                                uint32_t duty_w);

/** Coast (all outputs LOW). */
esp_err_t l298n_coast(const l298n_t *drv);
