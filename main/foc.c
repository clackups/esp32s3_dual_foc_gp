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
#include <string.h>

/* 120° in radians. */
#define TWO_PI_OVER_3  (2.0f * (float)M_PI / 3.0f)

void foc_init(foc_motor_t *motor, as5600_t *encoder, l298n_t *driver,
              uint8_t pole_pairs)
{
    motor->encoder               = encoder;
    motor->driver                = driver;
    motor->pole_pairs            = pole_pairs;
    motor->zero_electrical_angle = 0.0f;
    memset(motor->cal_table, 0, sizeof(motor->cal_table));
}

esp_err_t foc_calibrate(foc_motor_t *motor)
{
    uint32_t half = motor->driver->max_duty / 2;
    float cal_amplitude = 0.25f;
    float two_pi = 2.0f * (float)M_PI;
    float pp     = (float)motor->pole_pairs;

    /*
     * Phase 1 — Align the rotor to electrical angle 0 and record the
     * encoder reading to establish zero_electrical_angle.
     *
     *   U = half + half · 0.25 · sin(0)        = half
     *   V = half + half · 0.25 · sin(−120°)     ≈ half − 0.217·half
     *   W = half + half · 0.25 · sin(+120°)     ≈ half + 0.217·half
     */
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

    motor->zero_electrical_angle = fmodf(angle * pp, two_pi);

    /*
     * Phase 2 — Sweep one full mechanical revolution (pole_pairs
     * electrical cycles) while the motor is still energised.
     *
     * At each of the FOC_CAL_TABLE_SIZE steps the stator field is set
     * to a known electrical angle and the rotor aligns to it.  The
     * difference between the true (commanded) electrical angle and the
     * naïve value derived from the encoder reading is stored as a
     * per-bin correction.  At run-time foc_set_torque() interpolates
     * this table to compensate for AS5600 encoder nonlinearity.
     */
    for (int i = 0; i < FOC_CAL_TABLE_SIZE; i++) {
        float theta_e = (float)i / (float)FOC_CAL_TABLE_SIZE * pp * two_pi;

        du = (uint32_t)(half + half * cal_amplitude * sinf(theta_e));
        dv = (uint32_t)(half + half * cal_amplitude * sinf(theta_e - TWO_PI_OVER_3));
        dw = (uint32_t)(half + half * cal_amplitude * sinf(theta_e + TWO_PI_OVER_3));

        err = l298n_set_three_phase(motor->driver, du, dv, dw);
        if (err != ESP_OK) {
            memset(motor->cal_table, 0, sizeof(motor->cal_table));
            l298n_coast(motor->driver);
            return err;
        }

        vTaskDelay(pdMS_TO_TICKS(20));

        float mech;
        err = as5600_read_angle_rad(motor->encoder, &mech);
        if (err != ESP_OK) {
            memset(motor->cal_table, 0, sizeof(motor->cal_table));
            l298n_coast(motor->driver);
            return err;
        }

        /* What the naïve foc_set_torque() formula would compute: */
        float naive  = fmodf(mech * pp, two_pi) - motor->zero_electrical_angle;

        /* What the electrical angle actually is (rotor aligned to field): */
        float actual = fmodf(theta_e, two_pi);

        /* Correction = actual − naïve, normalised to [−π, +π]. */
        float corr = fmodf(actual - naive, two_pi);
        if (corr >  (float)M_PI) corr -= two_pi;
        if (corr < -(float)M_PI) corr += two_pi;

        motor->cal_table[i] = corr;
    }

    return l298n_coast(motor->driver);
}

esp_err_t foc_set_torque(foc_motor_t *motor, float torque)
{
    float mech_angle;
    esp_err_t err = as5600_read_angle_rad(motor->encoder, &mech_angle);
    if (err != ESP_OK) return err;

    float elec_angle = fmodf(mech_angle * motor->pole_pairs, 2.0f * (float)M_PI)
                       - motor->zero_electrical_angle;

    /* Apply encoder-nonlinearity correction from calibration table. */
    float bin_f     = mech_angle * ((float)FOC_CAL_TABLE_SIZE / (2.0f * (float)M_PI));
    float bin_floor = floorf(bin_f);
    int   bin0      = (int)bin_floor % FOC_CAL_TABLE_SIZE;
    if (bin0 < 0) bin0 += FOC_CAL_TABLE_SIZE;
    int   bin1 = (bin0 + 1) % FOC_CAL_TABLE_SIZE;
    float frac = bin_f - bin_floor;
    elec_angle += motor->cal_table[bin0]
               + frac * (motor->cal_table[bin1] - motor->cal_table[bin0]);

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
