#include "./ir.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include <math.h>

#define PIN_IR_RIGHT_ADC_GPIO   26
#define PIN_IR_RIGHT_ADC_CH     0

// --- Globals ---
static uint16_t g_calib_white = 0;
static uint16_t g_calib_black = 4095;

// --- STRONG SETTINGS (For Curves) ---
// Passed in from main.c
static float g_kp_strong = 45.0f;
static float g_ki_strong = 0.0f;
static float g_kd_strong = 8.0f;       

// --- WEAK SETTINGS (For Straights) ---
// UPDATED: Kp Lowered (8.0), Kd Tripled (6.0)
// This makes it very "heavy" and stable on straights.
static float g_kp_weak = 8.0f;  
static float g_ki_weak = 0.0f;
static float g_kd_weak = 6.0f; 

static float g_clamp = 60.0f;   
static float g_curve_slowdown = 0.5f; 
static float g_min_speed_frac = 0.4f;

// State
static float prev_error = 0.0f;
static float integral = 0.0f;

// Hysteresis State
static bool g_in_strong_mode = false;
static uint32_t g_last_switch_time = 0;

void ir_init(void) {
    adc_init();
    adc_gpio_init(PIN_IR_RIGHT_ADC_GPIO);
    adc_select_input(PIN_IR_RIGHT_ADC_CH);
}

void ir_set_calibration_values(uint16_t white_val, uint16_t black_val) {
    g_calib_white = white_val;
    g_calib_black = black_val;
}

void ir_set_tuning_params(float kp, float ki, float kd, 
                          float steer_clamp, float alpha, 
                          float curve_slowdown, float curve_exp,
                          float min_speed_frac) {
    g_kp_strong = kp; 
    g_ki_strong = ki; 
    g_kd_strong = kd;
    g_clamp = steer_clamp;
    g_curve_slowdown = curve_slowdown;
    g_min_speed_frac = min_speed_frac;
}

void ir_reset_pid_state(void) {
    prev_error = 0.0f;
    integral = 0.0f;
    g_in_strong_mode = false;
}

uint16_t ir_read_smooth(void) {
    uint32_t sum = 0;
    adc_select_input(PIN_IR_RIGHT_ADC_CH);
    for(int i=0; i<8; i++) sum += adc_read();
    return (uint16_t)(sum / 8);
}

void ir_follow_line_single(float base_speed, 
                           float *out_left, float *out_right, 
                           uint16_t *out_raw_sensor,
                           const char **out_state_str,
                           uint32_t *out_lost_time) {
    
    uint16_t raw = ir_read_smooth();
    if(out_raw_sensor) *out_raw_sensor = raw;

    float range = (float)(g_calib_black - g_calib_white);
    if (range < 100) range = 100;

    float val = (float)raw;
    if (val < g_calib_white) val = g_calib_white;
    if (val > g_calib_black) val = g_calib_black;

    // Normalized: 0.0 (White) ... 0.5 (Edge) ... 1.0 (Black)
    float normalized = (val - g_calib_white) / range;
    float error = normalized - 0.5f;

    // --- GAIN SCHEDULING WITH HYSTERESIS ---
    float use_kp, use_ki, use_kd;
    float abs_err = fabsf(error);
    uint32_t now = to_ms_since_boot(get_absolute_time());

    // Thresholds
    float trigger_strong = 0.12f; // Trigger strong earlier (was 0.15)
    float reset_weak = 0.08f;     // Go back to weak only when very centered

    if (g_in_strong_mode) {
        // We are in STRONG mode. Stay here until error is tiny AND time has passed.
        if (abs_err < reset_weak && (now - g_last_switch_time > 100)) {
            g_in_strong_mode = false;
            g_last_switch_time = now;
        }
    } else {
        // We are in WEAK mode. Switch if error gets big.
        if (abs_err > trigger_strong) {
            g_in_strong_mode = true;
            g_last_switch_time = now;
        }
    }

    if (g_in_strong_mode) {
        use_kp = g_kp_strong;
        use_ki = g_ki_strong;
        use_kd = g_kd_strong;
        *out_state_str = "CURVE(STRONG)";
    } else {
        use_kp = g_kp_weak;
        use_ki = g_ki_weak;
        use_kd = g_kd_weak;
        *out_state_str = "STRT(WEAK)";
    }

    // PID Calculation
    float p_term = error * use_kp;
    integral += error;
    float i_term = integral * use_ki;
    float d_term = (error - prev_error) * use_kd;
    prev_error = error;

    float steering = p_term + i_term + d_term;

    // Clamp
    if (steering > g_clamp) steering = g_clamp;
    if (steering < -g_clamp) steering = -g_clamp;

    // Curve Slowdown
    float turn_severity = fabsf(steering) / g_clamp; 
    float speed_factor = 1.0f - (turn_severity * g_curve_slowdown);
    if (speed_factor < g_min_speed_frac) speed_factor = g_min_speed_frac;

    float final_speed = base_speed * speed_factor;

    *out_left = final_speed + steering;
    *out_right = final_speed - steering;

    // Lost line logic
    static uint32_t lost_start = 0;
    if (normalized < 0.05f || normalized > 0.95f) {
        if (lost_start == 0) lost_start = to_ms_since_boot(get_absolute_time());
        *out_lost_time = to_ms_since_boot(get_absolute_time()) - lost_start;
    } else {
        lost_start = 0;
        *out_lost_time = 0;
    }
}