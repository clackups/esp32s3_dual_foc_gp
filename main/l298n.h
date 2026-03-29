/*
 * l298n.h — L298N dual-H-bridge motor driver (LEDC PWM + GPIO).
 *
 * Each L298N instance controls one 2804 BLDC motor (3 coil inputs
 * U/V/W) using its two H-bridge channels (A → coil U, B → coil V).
 * Coil W is left floating; the two driven phases produce a sinusoidal
 * field that is sufficient for haptic-feedback torque control.
 *
 * Control scheme:
 *   ENA / ENB — PWM speed (LEDC channels)
 *   IN1, IN2  — direction for H-bridge A (coil U)
 *   IN3, IN4  — direction for H-bridge B (coil V)
 */

#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

typedef struct {
    ledc_channel_t ch_ena;    /* ENA LEDC channel (coil U speed) */
    ledc_channel_t ch_enb;    /* ENB LEDC channel (coil V speed) */
    gpio_num_t     in1_gpio;  /* IN1 – H-bridge A direction */
    gpio_num_t     in2_gpio;  /* IN2 – H-bridge A direction */
    gpio_num_t     in3_gpio;  /* IN3 – H-bridge B direction */
    gpio_num_t     in4_gpio;  /* IN4 – H-bridge B direction */
    uint32_t       max_duty;  /* Maximum PWM duty (2^resolution − 1) */
} l298n_t;

/**
 * Initialise one L298N (two LEDC PWM channels + four direction GPIOs).
 *
 * @param drv          Pointer to an uninitialised l298n_t.
 * @param timer        LEDC timer to use (LEDC_TIMER_0 … 3).
 * @param ena_gpio     ENA GPIO (PWM for coil U).
 * @param in1_gpio     IN1 GPIO (direction A).
 * @param in2_gpio     IN2 GPIO (direction A).
 * @param enb_gpio     ENB GPIO (PWM for coil V).
 * @param in3_gpio     IN3 GPIO (direction B).
 * @param in4_gpio     IN4 GPIO (direction B).
 * @param ch_base      First LEDC channel to use; two consecutive
 *                     channels starting from ch_base will be consumed.
 * @param freq_hz      PWM frequency (e.g. 20 000).
 * @param resolution   LEDC timer resolution (e.g. LEDC_TIMER_10_BIT).
 * @return ESP_OK on success.
 */
esp_err_t l298n_init(l298n_t *drv, ledc_timer_t timer,
                     int ena_gpio, int in1_gpio, int in2_gpio,
                     int enb_gpio, int in3_gpio, int in4_gpio,
                     ledc_channel_t ch_base,
                     uint32_t freq_hz, ledc_timer_bit_t resolution);

/**
 * Set raw duty for each phase.
 *
 * @param phase_a  Signed duty for H-bridge A (−max_duty … +max_duty).
 * @param phase_b  Signed duty for H-bridge B (−max_duty … +max_duty).
 */
esp_err_t l298n_set_phase(const l298n_t *drv,
                          int32_t phase_a, int32_t phase_b);

/** Coast (all outputs LOW). */
esp_err_t l298n_coast(const l298n_t *drv);
