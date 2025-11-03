#include "ir.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include <math.h>
#include <stdio.h>

// ===== Strict RIGHT-edge follower (simple PD) =====
#define SENSOR_ON_RIGHT   1    // sensor on the RIGHT side (edge follower)
#define ALLOW_REVERSE     0    // no reversing (prevents spin/loops)

// ---- Pin map ----
#define PIN_IR_RIGHT_ADC_GPIO 26
#define PIN_IR_RIGHT_ADC_CH    0

// ---- Calibration (set at runtime) ----
static float SENSOR_WHITE = 180.0f;
static float SENSOR_BLACK = 2400.0f;

// ---- Edge PD (units: tps per unit error) ----
// error = (normalized reflectance) - 0.5
// steer_tps = KP*e + KD*de/dt
#define KP_SINGLE        30.0f   // starting point that felt “better”
#define KI_SINGLE         0.0f    // off
#define KD_SINGLE         3.0f    // light damping

// Keep steering requests realistic
#define STEER_CLAMP_TPS  25.0f

// ===== Speed control: slower overall + more slowdown in corners =====
// Global tame factor (affects straights & curves)
#define BASE_SPEED_SCALE     0.85f   // lower to go slower everywhere (e.g., 0.70)

// Corner slow-down based on steer magnitude (0..1)
// final_base = base * BASE_SPEED_SCALE * (1 - CURVE_SLOWDOWN * (|steer|/STEER_CLAMP_TPS)^CURVE_EXP)
#define CURVE_SLOWDOWN       0.70f   // raise (0.85–0.90) to slow more in turns
#define CURVE_EXP            1.6f    // emphasizes slowdown only on sharper turns
#define MIN_SPEED_FRACTION   0.30f   // can drop to 30% of commanded base in tight bends

// ===== Internals =====
static float pid_i = 0.0f;
static float last_err = 0.0f;
static absolute_time_t last_t;

static inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

static uint16_t adc_read_avg(uint ch, int samples) {
    adc_select_input(ch);
    uint32_t acc = 0;
    for (int i = 0; i < samples; i++) acc += adc_read();
    return (uint16_t)(acc / samples);
}

// ===== Public API =====
void ir_init(void) {
    ir_reset_pid_state();
}

void ir_reset_pid_state(void) {
    pid_i = 0.0f;
    last_err = 0.0f;
    last_t = get_absolute_time();
}

void ir_set_calibration_values(uint16_t white, uint16_t black) {
    SENSOR_WHITE = (float)white;
    SENSOR_BLACK = (float)black;

    // If your sensor reads LOWER on black, swap here:
    // if (SENSOR_BLACK < SENSOR_WHITE) { float t=SENSOR_BLACK; SENSOR_BLACK=SENSOR_WHITE; SENSOR_WHITE=t; }

    if (fabsf(SENSOR_BLACK - SENSOR_WHITE) < 100.0f) {
        printf("WARNING: Small IR range. White=%d Black=%d\n", (int)SENSOR_WHITE, (int)SENSOR_BLACK);
        SENSOR_BLACK = SENSOR_WHITE + 500.0f;
    }
    printf("IR Cal: White=%.0f Black=%.0f Range=%.0f\n",
           SENSOR_WHITE, SENSOR_BLACK, SENSOR_BLACK - SENSOR_WHITE);
}

uint16_t ir_read_single_sensor_raw(void) {
    return adc_read_avg(PIN_IR_RIGHT_ADC_CH, 12);
}

void ir_follow_line_single(float base_speed_tps,
                           float *out_target_left_tps, float *out_target_right_tps,
                           uint16_t *out_R_adc,
                           const char **out_state_name) {
    // 1) Read & normalize to [0..1]
    uint16_t R_adc = adc_read_avg(PIN_IR_RIGHT_ADC_CH, 12);
    float R_norm = (R_adc - SENSOR_WHITE) / (SENSOR_BLACK - SENSOR_WHITE);
    R_norm = clampf(R_norm, 0.0f, 1.0f);

    // 2) Right-edge PD: target is 0.5 (half white / half black)
    float err = R_norm - 0.5f;

    // timing
    absolute_time_t now = get_absolute_time();
    float dt = absolute_time_diff_us(last_t, now) / 1e6f;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;
    last_t = now;

    // PD (+I disabled)
    pid_i += err * dt;
    float d = (err - last_err) / dt;
    last_err = err;

    float steer = (KP_SINGLE * err) + (KI_SINGLE * pid_i) + (KD_SINGLE * d);
    steer = clampf(steer, -STEER_CLAMP_TPS, +STEER_CLAMP_TPS);

    // ===== Adaptive base speed: slower when steering hard =====
    float steer_frac = fabsf(steer) / STEER_CLAMP_TPS;      // 0..1
    float curve_scale = 1.0f - (CURVE_SLOWDOWN * powf(steer_frac, CURVE_EXP));
    curve_scale = clampf(curve_scale, MIN_SPEED_FRACTION, 1.0f);

    float B = base_speed_tps * BASE_SPEED_SCALE * curve_scale;

    // 3) Map steer to wheel targets; positive steer = turn RIGHT (left faster)
    float left_tps, right_tps;
#if SENSOR_ON_RIGHT
    left_tps  = B + steer;
    right_tps = B - steer;
#else
    left_tps  = B - steer;  // (kept for completeness)
    right_tps = B + steer;
#endif

#if !ALLOW_REVERSE
    // keep targets non-negative (no pivoting/reversing)
    if (left_tps  < 0.0f) left_tps  = 0.0f;
    if (right_tps < 0.0f) right_tps = 0.0f;
#endif

    // 4) Outputs
    *out_target_left_tps  = left_tps;
    *out_target_right_tps = right_tps;
    if (out_R_adc) *out_R_adc = R_adc;
    if (out_state_name) *out_state_name = "S_TRACK";
}
