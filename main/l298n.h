/*
 * l298n.h -- Mini L298N motor driver (3-phase sinusoidal PWM via MCPWM).
 *
 * The Mini L298N board exposes only IN1-IN4 (no ENA/ENB).  Each INx
 * pin directly drives the corresponding OUTx via PWM.  Three of the
 * four outputs drive the three coils of a 2804 BLDC motor:
 *
 *   IN1 (PWM) -> OUT1 -> Coil U
 *   IN2 (PWM) -> OUT2 -> Coil V
 *   IN3 (PWM) -> OUT3 -> Coil W
 */

#pragma once

#include "driver/mcpwm_prelude.h"
#include "esp_err.h"

typedef struct {
    mcpwm_cmpr_handle_t cmp_u;     /* Phase U comparator */
    mcpwm_cmpr_handle_t cmp_v;     /* Phase V comparator */
    mcpwm_cmpr_handle_t cmp_w;     /* Phase W comparator */
    uint32_t            max_duty;  /* Maximum compare value (peak ticks) */
} l298n_t;

/**
 * Initialise one Mini L298N (three MCPWM channels, center-aligned).
 *
 * @param drv          Pointer to an uninitialised l298n_t.
 * @param group_id     MCPWM group (0 or 1).
 * @param in1_gpio     IN1 GPIO (PWM -> coil U).
 * @param in2_gpio     IN2 GPIO (PWM -> coil V).
 * @param in3_gpio     IN3 GPIO (PWM -> coil W).
 * @param freq_hz      PWM frequency (e.g. 20 000).
 * @return ESP_OK on success.
 */
esp_err_t l298n_init(l298n_t *drv, int group_id,
                     int in1_gpio, int in2_gpio, int in3_gpio,
                     uint32_t freq_hz);

/**
 * Set unsigned PWM duty for each of the three motor phases.
 *
 * @param duty_u  Duty for coil U (0 ... max_duty).
 * @param duty_v  Duty for coil V (0 ... max_duty).
 * @param duty_w  Duty for coil W (0 ... max_duty).
 */
esp_err_t l298n_set_three_phase(const l298n_t *drv,
                                uint32_t duty_u, uint32_t duty_v,
                                uint32_t duty_w);

/** Coast (all outputs LOW). */
esp_err_t l298n_coast(const l298n_t *drv);
