/*
 * foc.c -- Three-phase field-oriented control for 2804 BLDC motors
 *         (3 coil inputs, 7 pole pairs).
 *
 * The Mini L298N drives all three coils (U, V, W) with sinusoidal
 * PWM 120 deg apart:
 *
 *   duty_u = half + half * amplitude * sin(theta_e + 90 deg)
 *   duty_v = half + half * amplitude * sin(theta_e + 90 deg - 120 deg)
 *   duty_w = half + half * amplitude * sin(theta_e + 90 deg + 120 deg)
 *
 * where theta_e = mechanical_angle * pole_pairs - zero_offset,
 * half = max_duty / 2, and amplitude in [-1, +1].
 * The 90 deg advance produces maximum torque in the positive direction.
 */

#include "foc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

/* 120 deg in radians. */
#define TWO_PI_OVER_3  (2.0f * (float)M_PI / 3.0f)

/* Number of measurement points per sweep direction within one
 * electrical cycle.
 *
 * With 6 points the field advances 60 deg electrical per step, giving
 * alignment torque = 0.95 * sin(60 deg) ~= 0.82 -- well above any
 * cogging-torque peak.
 *
 * The calibration sweeps the FULL mechanical revolution (6 steps *
 * pole_pairs = 42 steps for a 7-PP motor) instead of a single
 * electrical cycle.  Each measurement position is reached via a
 * two-phase approach:
 *   (a) Closed-loop drive: foc_set_torque() pushes the rotor
 *       toward the target mechanical angle.  This overcomes motor
 *       cogging reliably at every position, including electrical
 *       cycle boundaries where the open-loop field wraps.
 *   (b) Open-loop alignment: set_field() at the known electrical
 *       angle precisely positions the rotor for the measurement.
 *
 * The measured corrections are interpolated directly into the
 * 128-bin cal_table by mechanical angle, capturing both motor
 * construction imperfections (periodic with the electrical cycle)
 * and the slow-varying AS5600 encoder nonlinearity (periodic with
 * the full rotation).                                                 */
#define CAL_ELEC_STEPS    6

/* Milliseconds to let the rotor settle at each open-loop alignment.
 * The closed-loop drive brings the rotor close to the target, so
 * the alignment step only fine-tunes the position.  150 ms is
 * sufficient for the small 2804 motor at 0.95 amplitude.             */
#define CAL_SETTLE_MS     150

/* Maximum supported pole-pair count for the calibration correction
 * array.  Motors with more pole pairs than this are rejected.         */
#define CAL_MAX_POLE_PAIRS 14

/* Duration of the closed-loop drive phase per measurement step (ms).
 * The motor only needs to advance ~8.6 deg mechanical per step.      */
#define CAL_DRIVE_MS      40

/* P-control gain and peak torque for the closed-loop drive.
 * torque = clamp(error_rad * CAL_DRIVE_GAIN, +/-CAL_DRIVE_MAX_TQ).   */
#define CAL_DRIVE_GAIN    5.0f
#define CAL_DRIVE_MAX_TQ  0.50f

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
 * Result is wrapped to [0, 2pi).  fmodf(x, 2pi) always returns a
 * value in (-2pi, 2pi), so one conditional addition suffices for any
 * offset magnitude.
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
    return l298n_set_three_phase(drv, (uint32_t)(du + 0.5f),
                                     (uint32_t)(dv + 0.5f),
                                     (uint32_t)(dw + 0.5f));
}

esp_err_t foc_calibrate(foc_motor_t *motor)
{
    uint32_t half = motor->driver->max_duty / 2;
    float cal_amplitude = 0.95f;
    float two_pi = 2.0f * (float)M_PI;
    float pp     = (float)motor->pole_pairs;

    /*
     * Phase 1 -- Align the rotor to electrical angle 0 and record the
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
     * Phase 2 -- Full-revolution sweep with closed-loop positioning.
     *
     * Each measurement position is reached via:
     *   (a) Closed-loop P-control drive using foc_set_torque()
     *       (cal_table is all zeros at this point, so commutation
     *       uses only zero_electrical_angle -- good enough to get
     *       the rotor near the target).
     *   (b) Open-loop alignment at the known electrical angle for
     *       precise positioning and measurement.
     *
     * Two sweeps -- forward then backward -- are averaged so that
     * any directional bias from motor inertia cancels out.
     *
     * The corrections are interpolated directly into the 128-bin
     * cal_table by mechanical angle (no tiling), capturing both
     * electrical-periodic errors and encoder nonlinearity.
     */
    int total_steps = CAL_ELEC_STEPS * (int)motor->pole_pairs;
    /* Correction at each measurement position.  Maximum
     * CAL_ELEC_STEPS * CAL_MAX_POLE_PAIRS entries.                */
    float mech_corr[CAL_ELEC_STEPS * CAL_MAX_POLE_PAIRS];
    if (total_steps > CAL_ELEC_STEPS * CAL_MAX_POLE_PAIRS) {
        l298n_coast(motor->driver);
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* Starting angle from Phase 1 alignment (rotor at field = 0). */
    float start_angle;
    err = read_corrected_angle(motor, &start_angle);
    if (err != ESP_OK) {
        l298n_coast(motor->driver);
        return err;
    }
    float step_mech = two_pi / (float)total_steps;

    /* -- Forward sweep (full revolution) ------------------------ */
    for (int i = 0; i < total_steps; i++) {
        /* (a) Closed-loop drive toward target mechanical angle. */
        float target = start_angle + step_mech * (float)i;
        for (int t = 0; t < CAL_DRIVE_MS; t++) {
            float cur;
            err = read_corrected_angle(motor, &cur);
            if (err != ESP_OK) {
                memset(motor->cal_table, 0, sizeof(motor->cal_table));
                l298n_coast(motor->driver);
                return err;
            }
            float e = target - cur;
            if (e >  (float)M_PI) e -= two_pi;
            if (e < -(float)M_PI) e += two_pi;
            float tq = e * CAL_DRIVE_GAIN;
            if (tq >  CAL_DRIVE_MAX_TQ) tq =  CAL_DRIVE_MAX_TQ;
            if (tq < -CAL_DRIVE_MAX_TQ) tq = -CAL_DRIVE_MAX_TQ;
            err = foc_set_torque(motor, tq);
            if (err != ESP_OK) {
                memset(motor->cal_table, 0, sizeof(motor->cal_table));
                l298n_coast(motor->driver);
                return err;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        /* (b) Open-loop alignment at the known electrical angle. */
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
        mech_corr[i] = corr;
    }

    /* -- Backward sweep -- average with the forward pass -------- */
    for (int i = total_steps - 1; i >= 0; i--) {
        float target = start_angle + step_mech * (float)i;
        for (int t = 0; t < CAL_DRIVE_MS; t++) {
            float cur;
            err = read_corrected_angle(motor, &cur);
            if (err != ESP_OK) {
                memset(motor->cal_table, 0, sizeof(motor->cal_table));
                l298n_coast(motor->driver);
                return err;
            }
            float e = target - cur;
            if (e >  (float)M_PI) e -= two_pi;
            if (e < -(float)M_PI) e += two_pi;
            float tq = e * CAL_DRIVE_GAIN;
            if (tq >  CAL_DRIVE_MAX_TQ) tq =  CAL_DRIVE_MAX_TQ;
            if (tq < -CAL_DRIVE_MAX_TQ) tq = -CAL_DRIVE_MAX_TQ;
            err = foc_set_torque(motor, tq);
            if (err != ESP_OK) {
                memset(motor->cal_table, 0, sizeof(motor->cal_table));
                l298n_coast(motor->driver);
                return err;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

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
        mech_corr[i] = (mech_corr[i] + corr) * 0.5f;
    }

    /* -- Interpolate into 128-bin table by mechanical angle ------
     *
     * Each measurement i is at mechanical angle 2*pi * i / total_steps.
     * For each table bin j (at 2*pi * j / 128), find the two nearest
     * measurements and linearly interpolate.                        */
    for (int j = 0; j < FOC_CAL_TABLE_SIZE; j++) {
        float pos  = (float)j / (float)FOC_CAL_TABLE_SIZE
                   * (float)total_steps;
        int   idx0 = (int)floorf(pos) % total_steps;
        int   idx1 = (idx0 + 1) % total_steps;
        float frac = pos - floorf(pos);
        motor->cal_table[j] = mech_corr[idx0]
                             + frac * (mech_corr[idx1] - mech_corr[idx0]);
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

    /* 90 deg advance for torque production. */
    float field_angle = elec_angle + (float)M_PI_2;

    uint32_t half = motor->driver->max_duty / 2;
    float amp = torque;  /* -1 ... +1 */

    float du_f = half + half * amp * sinf(field_angle);
    float dv_f = half + half * amp * sinf(field_angle - TWO_PI_OVER_3);
    float dw_f = half + half * amp * sinf(field_angle + TWO_PI_OVER_3);

    if (du_f < 0.0f) du_f = 0.0f;
    if (dv_f < 0.0f) dv_f = 0.0f;
    if (dw_f < 0.0f) dw_f = 0.0f;

    return l298n_set_three_phase(motor->driver,
                                 (uint32_t)(du_f + 0.5f),
                                 (uint32_t)(dv_f + 0.5f),
                                 (uint32_t)(dw_f + 0.5f));
}

esp_err_t foc_coast(foc_motor_t *motor)
{
    return l298n_coast(motor->driver);
}

esp_err_t foc_read_angle(const foc_motor_t *motor, float *angle_rad)
{
    return read_corrected_angle(motor, angle_rad);
}
