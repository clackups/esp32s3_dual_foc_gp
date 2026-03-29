/*
 * foc.c — Three-phase field-oriented control for 2804 BLDC motors
 *         (3 coil inputs, 7 pole pairs).
 *
 * The Mini L298N drives all three coils (U, V, W) with sinusoidal
 * PWM 120° apart:
 *
 *   duty_u = half + half · amplitude · sin(θ_e + 90°)
 *   duty_v = half + half · amplitude · sin(θ_e + 90° − 120°)
 *   duty_w = half + half · amplitude · sin(θ_e + 90° + 120°)
 *
 * where θ_e = mechanical_angle × pole_pairs − zero_offset,
 * half = max_duty / 2, and amplitude ∈ [−1, +1].
 * The 90° advance produces maximum torque in the positive direction.
 */

#include "foc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

/* 120° in radians. */
#define TWO_PI_OVER_3  (2.0f * (float)M_PI / 3.0f)

void foc_init(foc_motor_t *motor, as5600_t *encoder, l298n_t *driver,
              uint8_t pole_pairs)
{
    motor->encoder               = encoder;
    motor->driver                = driver;
    motor->pole_pairs            = pole_pairs;
    motor->zero_electrical_angle = 0.0f;
}

esp_err_t foc_calibrate(foc_motor_t *motor)
{
    /*
     * Apply a known electrical angle (0) with moderate amplitude so the
     * rotor aligns.  3-phase sinusoidal at θ_e = 0:
     *   U = half + half · 0.25 · sin(0)        = half
     *   V = half + half · 0.25 · sin(−120°)     ≈ half − 0.217·half
     *   W = half + half · 0.25 · sin(+120°)     ≈ half + 0.217·half
     */
    uint32_t half = motor->driver->max_duty / 2;
    float cal_amplitude = 0.25f;
    uint32_t du = (uint32_t)(half + half * cal_amplitude * sinf(0.0f));
    uint32_t dv = (uint32_t)(half + half * cal_amplitude * sinf(-TWO_PI_OVER_3));
    uint32_t dw = (uint32_t)(half + half * cal_amplitude * sinf(TWO_PI_OVER_3));

    esp_err_t err = l298n_set_three_phase(motor->driver, du, dv, dw);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(500));

    float angle;
    err = as5600_read_angle_rad(motor->encoder, &angle);
    if (err != ESP_OK) {
        l298n_coast(motor->driver);
        return err;
    }

    motor->zero_electrical_angle =
        fmodf(angle * motor->pole_pairs, 2.0f * (float)M_PI);

    return l298n_coast(motor->driver);
}

esp_err_t foc_set_torque(foc_motor_t *motor, float torque)
{
    float mech_angle;
    esp_err_t err = as5600_read_angle_rad(motor->encoder, &mech_angle);
    if (err != ESP_OK) return err;

    float elec_angle = fmodf(mech_angle * motor->pole_pairs, 2.0f * (float)M_PI)
                       - motor->zero_electrical_angle;

    /* 90° advance for torque production. */
    float field_angle = elec_angle + (float)M_PI_2;

    uint32_t half = motor->driver->max_duty / 2;
    float amp = torque;  /* −1 … +1 */

    uint32_t du = (uint32_t)(half + half * amp * sinf(field_angle));
    uint32_t dv = (uint32_t)(half + half * amp * sinf(field_angle - TWO_PI_OVER_3));
    uint32_t dw = (uint32_t)(half + half * amp * sinf(field_angle + TWO_PI_OVER_3));

    return l298n_set_three_phase(motor->driver, du, dv, dw);
}

esp_err_t foc_read_angle(const foc_motor_t *motor, float *angle_rad)
{
    return as5600_read_angle_rad(motor->encoder, angle_rad);
}
