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

/* Number of measurement points per sweep direction within one
 * electrical cycle.
 *
 * With 6 points the field advances 60° electrical per step, giving
 * alignment torque = 0.85 × sin(60°) ≈ 0.74 — well above any
 * cogging-torque peak.  The total mechanical travel is only ~43°
 * (one electrical cycle / pole_pairs), keeping calibration fast
 * and unobtrusive.
 *
 * The measured corrections are tiled across the full 128-bin table
 * using electrical-angle periodicity.  This captures motor
 * construction imperfections exactly (they repeat every electrical
 * cycle) and approximates the slow-varying AS5600 encoder
 * nonlinearity well.                                                  */
#define CAL_ELEC_STEPS    6

/* Milliseconds to let the rotor settle at each alignment point. */
#define CAL_SETTLE_MS     60

void foc_init(foc_motor_t *motor, as5600_t *encoder, l298n_t *driver,
              uint8_t pole_pairs, float angle_offset)
{
    motor->encoder               = encoder;
    motor->driver                = driver;
    motor->pole_pairs            = pole_pairs;
    motor->angle_offset          = angle_offset;
    motor->zero_electrical_angle = 0.0f;
    memset(motor->cal_table, 0, sizeof(motor->cal_table));
}

/* -----------------------------------------------------------------
 * Helper: read the AS5600 angle and apply the manual mounting offset.
 * Result is wrapped to [0, 2π).
 * ----------------------------------------------------------------- */
static esp_err_t read_corrected_angle(const foc_motor_t *motor,
                                      float *angle_rad)
{
    esp_err_t err = as5600_read_angle_rad(motor->encoder, angle_rad);
    if (err != ESP_OK) return err;
    float a = fmodf(*angle_rad + motor->angle_offset,
                     2.0f * (float)M_PI);
    if (a < 0.0f) a += 2.0f * (float)M_PI;
    *angle_rad = a;
    return ESP_OK;
}

/* -----------------------------------------------------------------
 * Helper: set three-phase PWM from a field angle and amplitude.
 * Computes sinusoidal duties, clamps to [0, max_duty], and writes.
 * ----------------------------------------------------------------- */
static esp_err_t set_field(const l298n_t *drv, uint32_t half,
                           float amplitude, float field_angle)
{
    float du = half + half * amplitude * sinf(field_angle);
    float dv = half + half * amplitude * sinf(field_angle - TWO_PI_OVER_3);
    float dw = half + half * amplitude * sinf(field_angle + TWO_PI_OVER_3);
    if (du < 0.0f) du = 0.0f;
    if (dv < 0.0f) dv = 0.0f;
    if (dw < 0.0f) dw = 0.0f;
    return l298n_set_three_phase(drv, (uint32_t)du, (uint32_t)dv, (uint32_t)dw);
}

esp_err_t foc_calibrate(foc_motor_t *motor)
{
    uint32_t half = motor->driver->max_duty / 2;
    float cal_amplitude = 0.85f;
    float two_pi = 2.0f * (float)M_PI;
    float pp     = (float)motor->pole_pairs;

    /*
     * Phase 1 — Align the rotor to electrical angle 0 and record the
     * encoder reading to establish zero_electrical_angle.
     */
    esp_err_t err = set_field(motor->driver, half, cal_amplitude, 0.0f);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(500));

    float angle;
    err = read_corrected_angle(motor, &angle);
    if (err != ESP_OK) {
        l298n_coast(motor->driver);
        return err;
    }

    motor->zero_electrical_angle = fmodf(angle * pp, two_pi);

    /*
     * Phase 2 — Open-loop sweep over one electrical cycle (~51° mech.).
     *
     * At each of CAL_ELEC_STEPS positions the stator field is set to a
     * known electrical angle and the rotor aligns to it.  The step size
     * (60° elec.) provides strong alignment torque that easily overcomes
     * cogging peaks.  Each correction is the difference between the
     * commanded electrical angle and the naïve value derived from the
     * encoder reading.
     *
     * Two sweeps — forward then backward — are averaged so that any
     * directional bias from motor inertia or limited settle time
     * cancels out.
     *
     * Because the motor's construction imperfections repeat every
     * electrical cycle, the corrections measured here are tiled across
     * the full 128-bin cal_table by electrical-angle lookup.
     */
    float elec_corr[CAL_ELEC_STEPS];

    /* ── Forward sweep (one electrical cycle) ──────────────────── */
    for (int i = 0; i < CAL_ELEC_STEPS; i++) {
        float theta_e = two_pi * (float)i / (float)CAL_ELEC_STEPS;

        err = set_field(motor->driver, half, cal_amplitude, theta_e);
        if (err != ESP_OK) {
            memset(motor->cal_table, 0, sizeof(motor->cal_table));
            l298n_coast(motor->driver);
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(CAL_SETTLE_MS));

        float mech;
        err = read_corrected_angle(motor, &mech);
        if (err != ESP_OK) {
            memset(motor->cal_table, 0, sizeof(motor->cal_table));
            l298n_coast(motor->driver);
            return err;
        }

        float naive  = fmodf(mech * pp, two_pi) - motor->zero_electrical_angle;
        float corr   = fmodf(theta_e - naive, two_pi);
        if (corr >  (float)M_PI) corr -= two_pi;
        if (corr < -(float)M_PI) corr += two_pi;
        elec_corr[i] = corr;
    }

    /* ── Backward sweep — average with the forward pass ────────── */
    for (int i = CAL_ELEC_STEPS - 1; i >= 0; i--) {
        float theta_e = two_pi * (float)i / (float)CAL_ELEC_STEPS;

        err = set_field(motor->driver, half, cal_amplitude, theta_e);
        if (err != ESP_OK) {
            memset(motor->cal_table, 0, sizeof(motor->cal_table));
            l298n_coast(motor->driver);
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(CAL_SETTLE_MS));

        float mech;
        err = read_corrected_angle(motor, &mech);
        if (err != ESP_OK) {
            memset(motor->cal_table, 0, sizeof(motor->cal_table));
            l298n_coast(motor->driver);
            return err;
        }

        float naive  = fmodf(mech * pp, two_pi) - motor->zero_electrical_angle;
        float corr   = fmodf(theta_e - naive, two_pi);
        if (corr >  (float)M_PI) corr -= two_pi;
        if (corr < -(float)M_PI) corr += two_pi;
        elec_corr[i] = (elec_corr[i] + corr) * 0.5f;
    }

    /* ── Tile corrections into the 128-bin table by elec. phase ── */
    for (int j = 0; j < FOC_CAL_TABLE_SIZE; j++) {
        float mech_j = (float)j / (float)FOC_CAL_TABLE_SIZE * two_pi;
        float phase  = fmodf(mech_j * pp - motor->zero_electrical_angle, two_pi);
        if (phase < 0.0f) phase += two_pi;

        float pos  = phase / two_pi * (float)CAL_ELEC_STEPS;
        int   idx0 = (int)floorf(pos) % CAL_ELEC_STEPS;
        int   idx1 = (idx0 + 1) % CAL_ELEC_STEPS;
        float frac = pos - floorf(pos);
        motor->cal_table[j] = elec_corr[idx0]
                             + frac * (elec_corr[idx1] - elec_corr[idx0]);
    }

    return l298n_coast(motor->driver);
}

esp_err_t foc_set_torque(foc_motor_t *motor, float torque)
{
    float mech_angle;
    esp_err_t err = read_corrected_angle(motor, &mech_angle);
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

    float du_f = half + half * amp * sinf(field_angle);
    float dv_f = half + half * amp * sinf(field_angle - TWO_PI_OVER_3);
    float dw_f = half + half * amp * sinf(field_angle + TWO_PI_OVER_3);

    if (du_f < 0.0f) du_f = 0.0f;
    if (dv_f < 0.0f) dv_f = 0.0f;
    if (dw_f < 0.0f) dw_f = 0.0f;

    return l298n_set_three_phase(motor->driver,
                                 (uint32_t)du_f, (uint32_t)dv_f,
                                 (uint32_t)dw_f);
}

esp_err_t foc_coast(foc_motor_t *motor)
{
    return l298n_coast(motor->driver);
}

esp_err_t foc_read_angle(const foc_motor_t *motor, float *angle_rad)
{
    return read_corrected_angle(motor, angle_rad);
}
