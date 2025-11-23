#include "ir.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include <math.h>
#include <stdio.h>

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

void ir_set_tuning_params(float kp, float ki, float kd, 
                          float steer_clamp, float base_scale) 
{
    g_params.kp = kp;
    g_params.ki = ki;
    g_params.kd = kd;
    g_params.steer_clamp = steer_clamp;
    g_params.base_scale = base_scale;
}

void ir_init(void) {
    ir_reset_pid_state();
}

void ir_reset_pid_state(void) {
    pid_integral = 0.0f;
    pid_last_error = 0.0f;
    pid_last_time = get_absolute_time();
}

void ir_set_calibration_values(uint16_t white, uint16_t black) {
    SENSOR_WHITE = (float)white;
    SENSOR_BLACK = (float)black;
    // Safety check to prevent division by zero
    if (fabsf(SENSOR_BLACK - SENSOR_WHITE) < 100.0f) {
        SENSOR_BLACK = SENSOR_WHITE + 500.0f;
    }
    printf("IR Calibrated: White=%.0f Black=%.0f\n", SENSOR_WHITE, SENSOR_BLACK);
}

uint16_t ir_read_single_sensor_raw(void) {
    return adc_read_avg(PIN_IR_RIGHT_ADC_CH, ADC_SAMPLES);
}

/**
 * @brief Simplified Single Sensor Line Follower (Left Edge Strategy)
 */
void ir_follow_line_single(float base_speed_tps,
                           float *out_target_left_tps, float *out_target_right_tps,
                           uint16_t *out_R_adc) {
    
    // 1. Read & Normalize Sensor
    uint16_t R_adc = adc_read_avg(PIN_IR_RIGHT_ADC_CH, ADC_SAMPLES);
    
    // Normalize to 0.0 (White) to 1.0 (Black)
    float R_norm = (R_adc - SENSOR_WHITE) / (SENSOR_BLACK - SENSOR_WHITE);
    R_norm = clampf(R_norm, 0.0f, 1.0f);

    // 2. Calculate Delta Time
    absolute_time_t now = get_absolute_time();
    float dt_s = absolute_time_diff_us(pid_last_time, now) / 1e6f;
    if (dt_s <= 0.0f || dt_s > 0.1f) dt_s = 0.01f;
    pid_last_time = now;

    // 3. Calculate Error (LEFT EDGE STRATEGY)
    // Target is 0.5.
    // White (0.0) -> Error = +0.5 (Turn Right)
    // Black (1.0) -> Error = -0.5 (Turn Left)
    float error = PID_SETPOINT - R_norm;

    // --- FIX: ASYMMETRIC GAIN ---
    // If Error is NEGATIVE, we are seeing BLACK (Scenario B).
    // We risk crossing the line. We must react HARDER here.
    float current_kp = g_params.kp;
    if (error < 0) {
        current_kp = g_params.kp * 2.5f; // 2.5x stronger reaction to Black
    }
    // ----------------------------

    // 4. PID Calculation
    float p_term = current_kp * error;

    pid_integral += error * dt_s;
    pid_integral = clampf(pid_integral, -10.0f, 10.0f);
    float i_term = g_params.ki * pid_integral;

    float derivative = (error - pid_last_error) / dt_s;
    pid_last_error = error;
    float d_term = g_params.kd * derivative;

    // Final Output
    float output = p_term + i_term + d_term;
    
    // Output limiting
    output = clampf(output, -g_params.steer_clamp, +g_params.steer_clamp);

    // 5. Motor Mixing
    // Error Positive (White) -> Turn Right (Left + Out, Right - Out)
    // Error Negative (Black) -> Turn Left  (Left - Out, Right + Out) -> Wait...
    
    // Let's re-verify the mixing signs for Negative Error:
    // Error = -0.5. Output is Negative (e.g., -20).
    // Left = Base + (-20) = Slows Down.
    // Right = Base - (-20) = Base + 20 = Speeds Up.
    // Result: Right Wheel Fast, Left Wheel Slow -> Turn Left. CORRECT.
    
    float left_tps  = (base_speed_tps * g_params.base_scale) + output;
    float right_tps = (base_speed_tps * g_params.base_scale) - output;

    // 6. Outputs
    if (left_tps < 0) left_tps = 0;
    if (right_tps < 0) right_tps = 0;

    *out_target_left_tps  = left_tps;
    *out_target_right_tps = right_tps;
    if (out_R_adc) *out_R_adc = R_adc;
}