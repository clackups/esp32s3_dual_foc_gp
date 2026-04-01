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

/* Maximum iterations for the closed-loop position drive per cal bin. */
#define CAL_MAX_DRIVE_ITERS  500

/* Convergence tolerance for cumulative displacement (rad, ≈ 0.57°). */
#define CAL_POSITION_TOL     0.01f

void foc_init(foc_motor_t *motor, as5600_t *encoder, l298n_t *driver,
              uint8_t pole_pairs)
{
    motor->encoder               = encoder;
    motor->driver                = driver;
    motor->pole_pairs            = pole_pairs;
    motor->zero_electrical_angle = 0.0f;
    memset(motor->cal_table, 0, sizeof(motor->cal_table));
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

/* -----------------------------------------------------------------
 * Helper: read encoder, update cumulative mechanical displacement.
 * *cum is updated in place; *prev_mech tracks the previous reading.
 * Returns the current mechanical angle in *out_mech.
 * ----------------------------------------------------------------- */
static esp_err_t track_position(as5600_t *enc, float *prev_mech,
                                float *cum, float *out_mech)
{
    float mech;
    esp_err_t err = as5600_read_angle_rad(enc, &mech);
    if (err != ESP_OK) return err;

    float delta = mech - *prev_mech;
    float two_pi = 2.0f * (float)M_PI;
    if (delta >  (float)M_PI) delta -= two_pi;
    if (delta < -(float)M_PI) delta += two_pi;
    *cum       += delta;
    *prev_mech  = mech;
    *out_mech   = mech;
    return ESP_OK;
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
    err = as5600_read_angle_rad(motor->encoder, &angle);
    if (err != ESP_OK) {
        l298n_coast(motor->driver);
        return err;
    }

    motor->zero_electrical_angle = fmodf(angle * pp, two_pi);

    /*
     * Phase 2 — Closed-loop sweep of one full mechanical revolution.
     *
     * Previous attempts used open-loop alignment drive (field set to a
     * target angle, rotor expected to follow).  That fails on motors
     * with significant cogging because alignment torque ∝ sin(lag) is
     * near zero when the rotor is close to the target, too weak to
     * break free from a cogging peak.
     *
     * Instead we now use **closed-loop torque drive**: at each
     * iteration the encoder is read, the electrical angle is computed
     * from the Phase-1 zero, and the stator field is placed 90° ahead
     * of the rotor, producing *maximum* torque regardless of cogging
     * position.  Once the rotor reaches the vicinity of a calibration
     * point it is briefly held with open-loop alignment at a known
     * electrical angle for a precise encoder reading.
     *
     * Two sweeps — forward then backward — are averaged so that any
     * directional bias from motor inertia or limited settle time
     * cancels out, preventing asymmetric CW/CCW torque.
     */

    float prev_mech;
    err = as5600_read_angle_rad(motor->encoder, &prev_mech);
    if (err != ESP_OK) { l298n_coast(motor->driver); return err; }
    float cum = 0.0f;           /* cumulative mechanical displacement */

    /* ── Forward sweep ─────────────────────────────────────────── */
    for (int i = 0; i < FOC_CAL_TABLE_SIZE; i++) {
        float target = two_pi * (float)i / (float)FOC_CAL_TABLE_SIZE;

        /* (a) Drive rotor forward to target using closed-loop torque. */
        for (int iter = 0; iter < CAL_MAX_DRIVE_ITERS; iter++) {
            float mech;
            err = track_position(motor->encoder, &prev_mech, &cum, &mech);
            if (err != ESP_OK) {
                memset(motor->cal_table, 0, sizeof(motor->cal_table));
                l298n_coast(motor->driver);
                return err;
            }
            if (cum >= target - CAL_POSITION_TOL) break;

            /* Field 90° ahead of rotor → maximum forward torque. */
            float elec = fmodf(mech * pp - motor->zero_electrical_angle,
                               two_pi);
            err = set_field(motor->driver, half, cal_amplitude,
                            elec + (float)M_PI_2);
            if (err != ESP_OK) {
                memset(motor->cal_table, 0, sizeof(motor->cal_table));
                l298n_coast(motor->driver);
                return err;
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }

        /* (b) Briefly hold alignment at the known electrical angle so
         *     the rotor settles precisely for measurement. */
        float theta_e = (float)i / (float)FOC_CAL_TABLE_SIZE * pp * two_pi;
        err = set_field(motor->driver, half, cal_amplitude,
                        fmodf(theta_e, two_pi));
        if (err != ESP_OK) {
            memset(motor->cal_table, 0, sizeof(motor->cal_table));
            l298n_coast(motor->driver);
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(30));

        /* (c) Read encoder and record correction. */
        float mech;
        err = track_position(motor->encoder, &prev_mech, &cum, &mech);
        if (err != ESP_OK) {
            memset(motor->cal_table, 0, sizeof(motor->cal_table));
            l298n_coast(motor->driver);
            return err;
        }

        float naive  = fmodf(mech * pp, two_pi) - motor->zero_electrical_angle;
        float actual = fmodf(theta_e, two_pi);
        float corr   = fmodf(actual - naive, two_pi);
        if (corr >  (float)M_PI) corr -= two_pi;
        if (corr < -(float)M_PI) corr += two_pi;
        motor->cal_table[i] = corr;
    }

    /* ── Backward sweep — average with the forward pass ────────── */
    for (int i = FOC_CAL_TABLE_SIZE - 1; i >= 0; i--) {
        float target = two_pi * (float)i / (float)FOC_CAL_TABLE_SIZE;

        /* (a) Drive rotor backward to target. */
        for (int iter = 0; iter < CAL_MAX_DRIVE_ITERS; iter++) {
            float mech;
            err = track_position(motor->encoder, &prev_mech, &cum, &mech);
            if (err != ESP_OK) {
                memset(motor->cal_table, 0, sizeof(motor->cal_table));
                l298n_coast(motor->driver);
                return err;
            }
            if (cum <= target + CAL_POSITION_TOL) break;

            /* Field 90° behind rotor → maximum reverse torque. */
            float elec = fmodf(mech * pp - motor->zero_electrical_angle,
                               two_pi);
            err = set_field(motor->driver, half, cal_amplitude,
                            elec - (float)M_PI_2);
            if (err != ESP_OK) {
                memset(motor->cal_table, 0, sizeof(motor->cal_table));
                l298n_coast(motor->driver);
                return err;
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }

        /* (b) Alignment settle. */
        float theta_e = (float)i / (float)FOC_CAL_TABLE_SIZE * pp * two_pi;
        err = set_field(motor->driver, half, cal_amplitude,
                        fmodf(theta_e, two_pi));
        if (err != ESP_OK) {
            memset(motor->cal_table, 0, sizeof(motor->cal_table));
            l298n_coast(motor->driver);
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(30));

        /* (c) Read and average with forward pass. */
        float mech;
        err = track_position(motor->encoder, &prev_mech, &cum, &mech);
        if (err != ESP_OK) {
            memset(motor->cal_table, 0, sizeof(motor->cal_table));
            l298n_coast(motor->driver);
            return err;
        }

        float naive  = fmodf(mech * pp, two_pi) - motor->zero_electrical_angle;
        float actual = fmodf(theta_e, two_pi);
        float corr   = fmodf(actual - naive, two_pi);
        if (corr >  (float)M_PI) corr -= two_pi;
        if (corr < -(float)M_PI) corr += two_pi;
        motor->cal_table[i] = (motor->cal_table[i] + corr) * 0.5f;
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
    return as5600_read_angle_rad(motor->encoder, angle_rad);
}
