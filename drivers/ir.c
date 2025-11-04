#include "ir.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include <math.h>
#include <stdio.h>

// ===== ROBOT CONFIG =====
// We are a strict LEFT-edge follower
#define SENSOR_ON_RIGHT 0      // 1 = sensor on RIGHT, 0 = sensor on LEFT
#define ALLOW_REVERSE   0      // 0 = never spin wheels backward (safer)

// ---- Pin map ----
// (Assuming sensor is now plugged into the same pin)
#define PIN_IR_LEFT_ADC_GPIO 26
#define PIN_IR_LEFT_ADC_CH   0
#define ADC_SAMPLES          12    // Number of ADC samples to average

// ---- Calibration (set at runtime) ----
static float SENSOR_WHITE = 180.0f;
static float SENSOR_BLACK = 2400.0f;

// ---- Line-Following PD Controller ----
#define PID_SETPOINT     0.5f   // Target: 50% (on the edge of the line)

// --- NEW: Tuning parameters are now variables ---
typedef struct {
    float kp;
    float ki;
    float kd;
    float steer_clamp;
    float base_scale;
    float curve_slowdown;
    float curve_exp;
    float min_speed_frac;
} ir_tuning_params_t;

// Default to a very safe, slow tune (Scenario 1)
static ir_tuning_params_t g_params = {
    .kp = 15.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .steer_clamp = 20.0f,
    .base_scale = 1.0f,
    .curve_slowdown = 0.5f,
    .curve_exp = 1.2f,
    .min_speed_frac = 0.4f
};


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

/**
 * @brief NEW: Public function to update the static tuning params.
 */
void ir_set_tuning_params(float kp, float ki, float kd, 
                          float steer_clamp, float base_scale, 
                          float curve_slowdown, float curve_exp, 
                          float min_speed_frac) 
{
    g_params.kp = kp;
    g_params.ki = ki;
    g_params.kd = kd;
    g_params.steer_clamp = steer_clamp;
    g_params.base_scale = base_scale;
    g_params.curve_slowdown = curve_slowdown;
    g_params.curve_exp = curve_exp;
    g_params.min_speed_frac = min_speed_frac;
}

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
    // Read the LEFT sensor now
    return adc_read_avg(PIN_IR_LEFT_ADC_CH, ADC_SAMPLES);
}

void ir_follow_line_single(float base_speed_tps,
                           float *out_target_left_tps, float *out_target_right_tps,
                           uint16_t *out_R_adc, // <-- NOTE: Kept name for API compatibility
                           const char **out_state_name) {
    
    // 1) Read & normalize sensor to [0.0 ... 1.0]
    uint16_t L_adc = adc_read_avg(PIN_IR_LEFT_ADC_CH, ADC_SAMPLES);
    float norm = (L_adc - SENSOR_WHITE) / (SENSOR_BLACK - SENSOR_WHITE);
    norm = clampf(norm, 0.0f, 1.0f); // Clamp to 0.0 (white) or 1.0 (black)

    // 2) Calculate error for PD controller
    //    RIGHT sensor: error = norm - SETPOINT (if > 0.5, on black, turn right)
    //    LEFT sensor:  error = SETPOINT - norm (if > 0.5, on black, turn left)
#if SENSOR_ON_RIGHT
    float error = norm - PID_SETPOINT;
#else
    float error = PID_SETPOINT - norm; // Inverted for LEFT sensor
#endif

    // 3) Calculate delta-time (dt) for PID
    absolute_time_t now = get_absolute_time();
    float dt_s = absolute_time_diff_us(pid_last_time, now) / 1e6f; // in seconds
    if (dt_s <= 0.0f || dt_s > 0.1f) dt_s = 0.01f; // Use 10ms default
    pid_last_time = now;

    // 4) Calculate PD terms (USING g_params NOW)
    float p_term = g_params.kp * error;

    if (g_params.ki != 0.0f) {
        pid_integral += error * dt_s;
        pid_integral = clampf(pid_integral, -10.0f, 10.0f); // Anti-windup
    } else {
        pid_integral = 0.0f;
    }
    float i_term = g_params.ki * pid_integral;

    float derivative = (error - pid_last_error) / dt_s;
    pid_last_error = error;
    float d_term = g_params.kd * derivative;

    // 5) Calculate final steering command (in tps)
    float steer = p_term + i_term + d_term;
    steer = clampf(steer, -g_params.steer_clamp, +g_params.steer_clamp);

    // 6) Dynamic Speed: Slow down in corners (USING g_params NOW)
    float steer_frac = fabsf(steer) / g_params.steer_clamp; 
    float curve_scale = 1.0f - (g_params.curve_slowdown * powf(steer_frac, g_params.curve_exp));
    curve_scale = clampf(curve_scale, g_params.min_speed_frac, 1.0f);

    float final_base_speed = base_speed_tps * g_params.base_scale * curve_scale;

    // 7) Map steer to wheel targets
    //    Positive steer = turn RIGHT (Left wheel faster, Right wheel slower)
    //    Negative steer = turn LEFT (Left wheel slower, Right wheel faster)
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
    
    // Pass the LEFT sensor value to the output pointer.
    // The pointer is still named 'out_R_adc' to avoid breaking 
    // the function's API (i.e., changing the .h file).
    if (out_R_adc) *out_R_adc = L_adc;

    // 10) Set state for telemetry
    if (out_state_name) {
        if (norm < 0.1f) {
            *out_state_name = STATE_NAME_LOST_W;
        } else if (norm > 0.9f) {
            *out_state_name = STATE_NAME_LOST_B;
        } else {
            *out_state_name = STATE_NAME_TRACK;
        }
    }
}