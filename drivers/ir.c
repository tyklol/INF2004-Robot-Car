#include "ir.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include <math.h>
#include <stdio.h>

// ===== ROBOT CONFIG =====
// We are a strict RIGHT-edge follower
#define SENSOR_ON_RIGHT  1      // 1 = sensor on RIGHT, 0 = sensor on LEFT
#define ALLOW_REVERSE    0      // 0 = never spin wheels backward (safer)

// ---- Pin map ----
#define PIN_IR_RIGHT_ADC_GPIO 26
#define PIN_IR_RIGHT_ADC_CH   0
#define ADC_SAMPLES           12    // Number of ADC samples to average

// ---- Calibration (set at runtime) ----
static float SENSOR_WHITE = 180.0f;
static float SENSOR_BLACK = 2400.0f;

// ---- Line-Following PD Controller (Tuned) ----
// We are a PD controller. KI is disabled for stability.
// Error = 0.0 (all white) ... 0.5 (on edge) ... 1.0 (all black)
// Target = 0.5
#define PID_SETPOINT     0.5f   // Target: 50% (on the edge of the line)
#define KP_SINGLE        40.0f  // Proportional gain. High gain for fast response.
#define KI_SINGLE        0.0f   // Integral gain. 0.0 = OFF.
#define KD_SINGLE        2.0f   // Derivative gain. Dampens oscillations.

// Max steering correction (in ticks-per-second)
#define STEER_CLAMP_TPS  30.0f  // Was 25. More authority to turn.

// ---- Dynamic Speed Control ----
// This slows the robot down in sharp turns.
#define BASE_SPEED_SCALE   0.85f  // Tames overall speed (0.85 = 85% of base)
#define CURVE_SLOWDOWN     0.70f  // How much to slow down (0.7 = 70% slowdown in max turn)
#define CURVE_EXP          1.6f   // (powf(x, 1.6)) Makes slowdown non-linear (more aggressive)
#define MIN_SPEED_FRACTION 0.30f  // Min speed in a turn (30% of base)

// ===== Internals =====
static float pid_integral = 0.0f;
static float pid_last_error = 0.0f;
static absolute_time_t pid_last_time;

// --- State Names for Telemetry ---
static const char* STATE_NAME_TRACK = "S_TRACK";
static const char* STATE_NAME_LOST_W = "S_LOST_WHT"; // All White
static const char* STATE_NAME_LOST_B = "S_LOST_BLK"; // All Black

// --- Helper Functions ---
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
    // ADC is initialized in main.c
    ir_reset_pid_state();
}

void ir_reset_pid_state(void) {
    pid_integral = 0.0f;
    pid_last_error = 0.0f;
    pid_last_time = get_absolute_time(); // Set last_t to now
}

void ir_set_calibration_values(uint16_t white, uint16_t black) {
    SENSOR_WHITE = (float)white;
    SENSOR_BLACK = (float)black;

    // Safety check: if black reads *lower* than white, swap them
    if (SENSOR_BLACK < SENSOR_WHITE) {
        printf("WARNING: Black ADC < White ADC. Swapping values.\n");
        float temp = SENSOR_BLACK;
        SENSOR_BLACK = SENSOR_WHITE;
        SENSOR_WHITE = temp;
    }

    // Safety check: if range is too small, create a fake range
    if (fabsf(SENSOR_BLACK - SENSOR_WHITE) < 100.0f) {
        printf("WARNING: Small IR range. White=%d Black=%d\n", (int)SENSOR_WHITE, (int)SENSOR_BLACK);
        SENSOR_BLACK = SENSOR_WHITE + 500.0f;
    }
    
    printf("IR Cal: White=%.0f Black=%.0f Range=%.0f\n",
           SENSOR_WHITE, SENSOR_BLACK, SENSOR_BLACK - SENSOR_WHITE);
}

uint16_t ir_read_single_sensor_raw(void) {
    return adc_read_avg(PIN_IR_RIGHT_ADC_CH, ADC_SAMPLES);
}

void ir_follow_line_single(float base_speed_tps,
                           float *out_target_left_tps, float *out_target_right_tps,
                           uint16_t *out_R_adc,
                           const char **out_state_name) {
    
    // 1) Read & normalize sensor to [0.0 ... 1.0]
    uint16_t R_adc = adc_read_avg(PIN_IR_RIGHT_ADC_CH, ADC_SAMPLES);
    float R_norm = (R_adc - SENSOR_WHITE) / (SENSOR_BLACK - SENSOR_WHITE);
    R_norm = clampf(R_norm, 0.0f, 1.0f); // Clamp to 0.0 (white) or 1.0 (black)

    // 2) Calculate error for PD controller
    // Target is 0.5.
    // If R_norm = 1.0 (all black, too far left), error = +0.5
    // If R_norm = 0.0 (all white, too far right), error = -0.5
    float error = R_norm - PID_SETPOINT;

    // 3) Calculate delta-time (dt) for PID
    absolute_time_t now = get_absolute_time();
    float dt_s = absolute_time_diff_us(pid_last_time, now) / 1e6f; // in seconds
    // Handle first loop or stall
    if (dt_s <= 0.0f || dt_s > 0.1f) dt_s = 0.01f; // Use 10ms default
    pid_last_time = now;

    // 4) Calculate PD terms
    float p_term = KP_SINGLE * error;

    // We only run the I-term if KI is non-zero (it's 0.0, so this is skipped)
    if (KI_SINGLE != 0.0f) {
        pid_integral += error * dt_s;
        pid_integral = clampf(pid_integral, -10.0f, 10.0f); // Anti-windup
    } else {
        pid_integral = 0.0f;
    }
    float i_term = KI_SINGLE * pid_integral;

    float derivative = (error - pid_last_error) / dt_s;
    pid_last_error = error;
    float d_term = KD_SINGLE * derivative;

    // 5) Calculate final steering command (in tps)
    float steer = p_term + i_term + d_term;
    steer = clampf(steer, -STEER_CLAMP_TPS, +STEER_CLAMP_TPS);

    // 6) Dynamic Speed: Slow down in corners
    // steer_frac = 0.0 (straight) ... 1.0 (max turn)
    float steer_frac = fabsf(steer) / STEER_CLAMP_TPS; 
    // curve_scale = 1.0 (straight) ... (1.0 - 0.7) = 0.3 (max turn)
    float curve_scale = 1.0f - (CURVE_SLOWDOWN * powf(steer_frac, CURVE_EXP));
    curve_scale = clampf(curve_scale, MIN_SPEED_FRACTION, 1.0f);

    float final_base_speed = base_speed_tps * BASE_SPEED_SCALE * curve_scale;

    // 7) Map steer to wheel targets
    // We are on the RIGHT edge.
    // error > 0 (too black/left) -> steer is positive -> turn RIGHT
    // To turn RIGHT: Left wheel faster, Right wheel slower.
    float left_tps  = final_base_speed + steer;
    float right_tps = final_base_speed - steer;

    // 8) Apply "No Reverse" logic
    #if !ALLOW_REVERSE
    if (left_tps  < 0.0f) left_tps  = 0.0f;
    if (right_tps < 0.0f) right_tps = 0.0f;
    #endif

    // 9) Set outputs
    *out_target_left_tps  = left_tps;
    *out_target_right_tps = right_tps;
    if (out_R_adc) *out_R_adc = R_adc;

    // 10) Set state for telemetry
    if (out_state_name) {
        // Use normalized value for lost-line check
        if (R_norm < 0.1f) {
            *out_state_name = STATE_NAME_LOST_W;
        } else if (R_norm > 0.9f) {
            *out_state_name = STATE_NAME_LOST_B;
        } else {
            *out_state_name = STATE_NAME_TRACK;
        }
    }
}