/*
 * foc.c — Simplified two-phase field-oriented control for 2804 BLDC
 *         motors (3 coil inputs, 7 pole pairs).
 *
 * The L298N dual-H-bridge drives coils U and V with sinusoidal
 * voltages 90° apart; coil W is left floating.
 *
 *   V_u = amplitude · cos(θ_e + 90°)
 *   V_v = amplitude · sin(θ_e + 90°)
 *
 * where θ_e = mechanical_angle × pole_pairs − zero_offset.
 * The 90° advance produces maximum torque in the positive direction.
 */

#include "foc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

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
    /* Drive a known electrical angle (0) so the rotor aligns. */
    int32_t half = (int32_t)(motor->driver->max_duty / 4);
    esp_err_t err = l298n_set_phase(motor->driver, half, 0);
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

    float amplitude = torque * (float)motor->driver->max_duty;
    int32_t va = (int32_t)(amplitude * cosf(field_angle));
    int32_t vb = (int32_t)(amplitude * sinf(field_angle));

    return l298n_set_phase(motor->driver, va, vb);
}

esp_err_t foc_read_angle(const foc_motor_t *motor, float *angle_rad)
{
    return as5600_read_angle_rad(motor->encoder, angle_rad);
}
