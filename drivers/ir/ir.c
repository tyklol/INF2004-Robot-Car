#include "ir.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include <math.h>
#include <stdio.h>

ir_state_t g_current_state = TRACK_RIGHT_EDGE;

// Define static vars for calibration (exposed via extern in header if needed)
float SENSOR_WHITE = 180.0f;
float SENSOR_BLACK = 2400.0f;

// State timers
static uint32_t g_timer_white_ms = 0;
static uint32_t g_timer_black_ms = 0;
static ir_tuning_params_t g_params_internal = {/* defaults */}; // We use the struct from ir.h generally

// State Names
static const char *STATE_NAME_TRACK_R = "S_TRACK_R";
static const char *STATE_NAME_TRACK_L = "S_TRACK_L";
static const char *STATE_NAME_SW_W = "SW_WHITE"; // Switch triggered by white
static const char *STATE_NAME_SW_B = "SW_BLACK"; // Switch triggered by black

// Internal PID state
static float pid_integral = 0.0f;
static float pid_last_error = 0.0f;
static absolute_time_t pid_last_time;

// --- Helper Functions ---
static inline float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static uint16_t adc_read_avg(uint ch, int samples)
{
    adc_select_input(ch);
    uint32_t acc = 0;
    for (int i = 0; i < samples; i++)
        acc += adc_read();
    return (uint16_t)(acc / samples);
}
static float ir_compute_steer(float R_norm, int g_current_state, float dt_s, float yaw_rate_dps)
{
    // 4) Calculate error based on mode
    float error;
    if (g_current_state == TRACK_RIGHT_EDGE)
    {
        error = R_norm - PID_SETPOINT;
    }
    else
    { // TRACK_LEFT_EDGE
        error = PID_SETPOINT - R_norm;
    }

    // 5) Calculate PD terms
    // (unchanged)
    float p_term = g_params.kp * error;
    if (g_params.ki != 0.0f)
    {
        pid_integral += error * dt_s;
        pid_integral = clampf(pid_integral, -10.0f, 10.0f);
    }
    else
    {
        pid_integral = 0.0f;
    }
    float i_term = g_params.ki * pid_integral;
    float derivative = (error - pid_last_error) / dt_s;
    pid_last_error = error;
    float d_term = g_params.kd * derivative;

    // 6) AGGRESSIVE CURVE-EXIT CORRECTION with IMU damping
    float rotation_damping = 0.0f;
    const float IMU_DAMPING_GAIN = 0.30f; // Increased for stronger damping mid-curve
    const float YAW_RATE_THRESHOLD = 30.0f;
    const float CURVE_EXIT_THRESHOLD = 0.08f; // LOWERED - exit curve sooner

    // Detect if we're exiting a curve (error approaching zero)
    bool near_straight_line = (fabsf(error) < CURVE_EXIT_THRESHOLD);

    if (fabsf(yaw_rate_dps) > YAW_RATE_THRESHOLD)
    {
        if (near_straight_line)
        {
            // CURVE EXIT: Apply COUNTER-rotation boost to force straightening
            // If still rotating but error is small, we need to fight the rotation
            float exit_boost_gain = 0.40f; // Strong correction
            rotation_damping = -exit_boost_gain * yaw_rate_dps;
        }
        else
        {
            // MID-CURVE: Apply normal damping, but with gradual rampdown
            float excess_rotation = fabsf(yaw_rate_dps) - YAW_RATE_THRESHOLD;

            // Gradually reduce damping as we approach straight line
            float damping_scale = clampf((fabsf(error) - CURVE_EXIT_THRESHOLD) / 0.25f, 0.0f, 1.0f);

            rotation_damping = -IMU_DAMPING_GAIN * excess_rotation * damping_scale * (yaw_rate_dps > 0 ? 1.0f : -1.0f);
        }
    }

    // 7) Calculate final steering with IMU correction
    float steer = p_term + i_term + d_term + rotation_damping;
    steer = clampf(steer, -g_params.steer_clamp, +g_params.steer_clamp);

    return steer;
}

// ===== Public API =====

// IMPORTANT: We need to define the storage for g_params
ir_tuning_params_t g_params = {
    .kp = 15.0f, .ki = 0.0f, .kd = 0.0f, .steer_clamp = 20.0f, .base_scale = 1.0f, .curve_slowdown = 0.5f, .curve_exp = 1.2f, .min_speed_frac = 0.4f, .timeout_R_to_L_ms = 320, .timeout_L_to_R_ms = 635};

void ir_set_tuning_params(float kp, float ki, float kd, float steer_clamp, float base_scale,
                          float curve_slowdown, float curve_exp, float min_speed_frac,
                          uint32_t timeout_white, uint32_t timeout_black)
{
    g_params.kp = kp;
    g_params.ki = ki;
    g_params.kd = kd;
    g_params.steer_clamp = steer_clamp;
    g_params.base_scale = base_scale;
    g_params.curve_slowdown = curve_slowdown;
    g_params.curve_exp = curve_exp;
    g_params.min_speed_frac = min_speed_frac;
    g_params.timeout_R_to_L_ms = timeout_white;
    g_params.timeout_L_to_R_ms = timeout_black;
}

void ir_init(void) { 
    adc_init();
    adc_gpio_init(PIN_IR_RIGHT_ADC_GPIO);
    ir_reset_robot_state(); 
}

void ir_reset_pid_state(void)
{
    pid_integral = 0.0f;
    pid_last_error = 0.0f;
    pid_last_time = get_absolute_time();
}

void ir_reset_robot_state(void)
{
    ir_reset_pid_state();
    g_current_state = TRACK_RIGHT_EDGE;
    g_timer_white_ms = 0;
    g_timer_black_ms = 0;
}

void ir_force_state(int state)
{
    g_current_state = (state == IR_STATE_TRACK_LEFT) ? TRACK_LEFT_EDGE : TRACK_RIGHT_EDGE;
    g_timer_white_ms = 0;
    g_timer_black_ms = 0;
    ir_reset_pid_state();
}

void ir_set_calibration_values(uint16_t white, uint16_t black)
{
    SENSOR_WHITE = (float)white;
    SENSOR_BLACK = (float)black;
    // Swap if inverted
    if (SENSOR_BLACK < SENSOR_WHITE)
    {
        float temp = SENSOR_BLACK;
        SENSOR_BLACK = SENSOR_WHITE;
        SENSOR_WHITE = temp;
    }
    // Safety gap
    if (fabsf(SENSOR_BLACK - SENSOR_WHITE) < 100.0f)
        SENSOR_BLACK = SENSOR_WHITE + 500.0f;
}

uint16_t ir_read_single_sensor_raw(void)
{
    return adc_read_avg(PIN_IR_RIGHT_ADC_CH, ADC_SAMPLES);
}

// Paste your full ir_follow_line_single function here
// Ensure it calls ir_compute_steer
void ir_follow_line_single(float base_speed_tps,
                           float *out_target_left_tps, float *out_target_right_tps,
                           uint16_t *out_R_adc,
                           const char **out_state_name,
                           uint32_t *out_time_lost_ms,
                           float yaw_rate_dps)
{

    uint16_t R_adc = adc_read_avg(PIN_IR_RIGHT_ADC_CH, ADC_SAMPLES);
    float R_norm = (R_adc - SENSOR_WHITE) / (SENSOR_BLACK - SENSOR_WHITE);
    R_norm = clampf(R_norm, 0.0f, 1.0f);

    absolute_time_t now = get_absolute_time();
    float dt_s = absolute_time_diff_us(pid_last_time, now) / 1e6f;
    if (dt_s <= 0.0f || dt_s > 0.1f)
        dt_s = 0.01f;
    pid_last_time = now;
    uint32_t dt_ms = (uint32_t)(dt_s * 1000.0f);

    bool did_switch = false;

    // Check timeouts using g_params.timeout...
    if (R_norm < 0.10f)
    {
        g_timer_white_ms += dt_ms;
        if (g_timer_white_ms > g_params.timeout_R_to_L_ms)
        { // Using timeout param
            did_switch = true;
            *out_state_name = STATE_NAME_SW_W;
        }
    }
    else
    {
        g_timer_white_ms = 0;
    }

    if (R_norm > 0.80f)
    {
        g_timer_black_ms += dt_ms;
        if (g_timer_black_ms > g_params.timeout_L_to_R_ms)
        {
            did_switch = true;
            *out_state_name = STATE_NAME_SW_B;
        }
    }
    else
    {
        g_timer_black_ms = 0;
    }

    // Compute Steer
    float steer = ir_compute_steer(R_norm, g_current_state, dt_s, yaw_rate_dps);

    // Speed Scaling
    float steer_frac = fabsf(steer) / g_params.steer_clamp;
    float curve_scale = 1.0f - (g_params.curve_slowdown * powf(steer_frac, g_params.curve_exp));
    curve_scale = clampf(curve_scale, g_params.min_speed_frac, 1.0f);
    float final_base_speed = base_speed_tps * g_params.base_scale * curve_scale;

    float left_tps = final_base_speed + steer;
    float right_tps = final_base_speed - steer;
    if (left_tps < 0)
        left_tps = 0;
    if (right_tps < 0)
        right_tps = 0;

    *out_target_left_tps = left_tps;
    *out_target_right_tps = right_tps;
    if (out_R_adc)
        *out_R_adc = R_adc;
    if (out_time_lost_ms)
        *out_time_lost_ms = (g_timer_white_ms > g_timer_black_ms) ? g_timer_white_ms : g_timer_black_ms;

    if (out_state_name && !did_switch)
    {
        *out_state_name = (g_current_state == TRACK_RIGHT_EDGE) ? STATE_NAME_TRACK_R : STATE_NAME_TRACK_L;
    }
}