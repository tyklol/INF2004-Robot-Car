#include "ir.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "./motor.h" // Using your requested include path
#include <math.h>

// ------------------------- PIN MAP -------------------------
#define PIN_IR_LEFT_ADC_GPIO    28    // GP28 = ADC2
#define PIN_IR_LEFT_ADC_CH       2
#define PIN_IR_RIGHT_ADC_GPIO   26    // GP26 = ADC0
#define PIN_IR_RIGHT_ADC_CH      0
// -----------------------------------------------------------

// ---------------------- SENSOR CALIBRATION (FROM YOUR DATA) -----------------
#define L_SENSOR_WHITE          170.0f
#define L_SENSOR_BLACK          2800.0f
#define R_SENSOR_WHITE          180.0f
#define R_SENSOR_BLACK          2400.0f
// -------------------------------------------------------------------

// ---------------------- DUAL SENSOR TUNABLES (ADJUSTED) -----------------
#define KP_TURN_DOUBLE          0.35f   // Slightly lower gain for smoothness WAS 0.40f
#define ADC_SMOOTH_SAMPLES      12
#define NORM_WHITE_THRESHOLD    0.2f    // 20% = "fully white"
// --- FIX for Tilt/Intersection --- Lowered significantly
#define NORM_BLACK_THRESHOLD    0.65f   // WAS 0.8f. Now requires only 65% blackness.
// -------------------------------------------------------------------

// --- "PERFECTED" SINGLE SENSOR PID TUNABLES (RIGHT EDGE) ---
#define NORM_SETPOINT_SINGLE    0.5f
#define KP_SINGLE               0.30f
#define KI_SINGLE               0.15f
#define KD_SINGLE               0.05f
#define NORM_WHITE_THRESH_SINGLE NORM_WHITE_THRESHOLD
#define NORM_BLACK_THRESH_SINGLE 0.75f // Single sensor can use a slightly higher black threshold
// -----------------------------------------------------------

// --- Dual-Sensor "Memory" ---
static float last_seen_error_dual = 0.0f;
static const char* STATE_NAME_TRACK = "TRACKING";
static const char* STATE_NAME_INTER = "INTERSEC";
static const char* STATE_NAME_LOST  = "LOST_LINE";

// --- Single-Sensor "Memory" ---
static const char* STATE_NAME_S_TRACK = "S_TRACK";
static const char* STATE_NAME_S_LOST_W = "S_LOST_WHT";
static const char* STATE_NAME_S_LOST_B = "S_LOST_BLK";
// --- PID Internal State Variables ---
static float pid_integral = 0.0f;
static float pid_last_error = 0.0f;
static absolute_time_t pid_last_time;
static bool pid_first_run = true;


// Helper function to clamp a float value
static float clampf(float x, float lo, float hi){
    return x < lo ? lo : (x > hi ? hi : x);
}

// Read ADC channel with averaging
static uint16_t adc_read_avg(uint ch, int samples) {
    adc_select_input(ch);
    uint32_t acc=0;
    for (int i=0; i<samples; i++) acc += adc_read();
    return (uint16_t)(acc / samples);
}

// --- Public Functions ---

void ir_init(void) {
    adc_init();
    adc_gpio_init(PIN_IR_LEFT_ADC_GPIO);  // GP28
    adc_gpio_init(PIN_IR_RIGHT_ADC_GPIO); // GP26
    ir_reset_pid_state();
}

/**
 * @brief Resets the PID memory.
 */
void ir_reset_pid_state(void) {
    pid_integral = 0.0f;
    pid_last_error = 0.0f;
    pid_last_time = get_absolute_time();
    pid_first_run = true;
    last_seen_error_dual = 0.0f;
}


/**
 * @brief Dual-sensor logic (Uses new thresholds)
 */
void ir_follow_line_double(float base_speed_percent,
                           float *out_left_power, float *out_right_power,
                           uint16_t *out_L_adc, uint16_t *out_R_adc,
                           const char **out_state_name) {

    // 1. Read sensors
    uint16_t L_adc = adc_read_avg(PIN_IR_LEFT_ADC_CH,  ADC_SMOOTH_SAMPLES);
    uint16_t R_adc = adc_read_avg(PIN_IR_RIGHT_ADC_CH, ADC_SMOOTH_SAMPLES);

    // 2. NORMALIZE readings to 0.0 (White) - 1.0 (Black)
    float L_norm = (L_adc - L_SENSOR_WHITE) / (L_SENSOR_BLACK - L_SENSOR_WHITE);
    float R_norm = (R_adc - R_SENSOR_WHITE) / (R_SENSOR_BLACK - R_SENSOR_WHITE);
    L_norm = clampf(L_norm, 0.0f, 1.0f);
    R_norm = clampf(R_norm, 0.0f, 1.0f);

    // 3. Determine state (uses NEW NORM_BLACK_THRESHOLD of 0.65f)
    bool is_left_white = (L_norm < NORM_WHITE_THRESHOLD);
    bool is_right_white = (R_norm < NORM_WHITE_THRESHOLD);
    bool is_left_black = (L_norm > NORM_BLACK_THRESHOLD); // Easier to trigger black
    bool is_right_black = (R_norm > NORM_BLACK_THRESHOLD); // Easier to trigger black

    float steer;
    float err = R_norm - L_norm;

    if (is_left_black && is_right_black) {
        // --- STATE 1: INTERSECTION ---
        // This state should now trigger more reliably even when tilted.
        steer = 0.0f;
        last_seen_error_dual = 0.0f;
        if (out_state_name) *out_state_name = STATE_NAME_INTER;

    } else if (is_left_white && is_right_white) {
        // --- STATE 2: LOST LINE ---
        if (out_state_name) *out_state_name = STATE_NAME_LOST;
        if (last_seen_error_dual > 0.1f) {
            steer = -KP_TURN_DOUBLE; // Spin RIGHT
        } else if (last_seen_error_dual < -0.1f) {
            steer = KP_TURN_DOUBLE; // Spin LEFT
        } else {
            steer = 0.0f;
        }
    } else {
        // --- STATE 3: ON-LINE TRACKING ---
        steer = KP_TURN_DOUBLE * err;
        last_seen_error_dual = err;
        if (out_state_name) *out_state_name = STATE_NAME_TRACK;
    }

    // --- 4. Calculate final motor speeds ---
    float current_base_duty = base_speed_percent / 100.0f;
    float left_duty  = clampf(current_base_duty - steer, 0.f, 1.f);
    float right_duty = clampf(current_base_duty + steer, 0.f, 1.f);
    float left_percent = left_duty * 100.0f;
    float right_percent = right_duty * 100.0f;

    // --- 5. Set Motors and report values ---
    motor_set_speed(MOTOR_LEFT,  left_percent);
    motor_set_speed(MOTOR_RIGHT, right_percent);

    if (out_left_power)  *out_left_power  = left_percent;
    if (out_right_power) *out_right_power = right_percent;
    if (out_L_adc) *out_L_adc = L_adc;
    if (out_R_adc) *out_R_adc = R_adc;
}


/**
 * @brief "PERFECTED" Single-sensor logic for RIGHT EDGE TRACKING (Uses new thresholds)
 */
void ir_follow_line_single(float base_speed_percent,
                           float *out_left_power, float *out_right_power,
                           uint16_t *out_L_adc,
                           const char **out_state_name) {

    // 1. Read sensor and normalize
    uint16_t L_adc = adc_read_avg(PIN_IR_LEFT_ADC_CH, ADC_SMOOTH_SAMPLES);
    float L_norm = (L_adc - L_SENSOR_WHITE) / (L_SENSOR_BLACK - L_SENSOR_WHITE);
    L_norm = clampf(L_norm, 0.0f, 1.0f);

    // 2. Determine state
    bool is_white = (L_norm < NORM_WHITE_THRESH_SINGLE); // Uses 0.2f
    bool is_black = (L_norm > NORM_BLACK_THRESH_SINGLE); // Uses 0.75f
    float steer;

    // --- PID Time Calculation ---
    absolute_time_t now = get_absolute_time();
    float dt = absolute_time_diff_us(pid_last_time, now) / 1000000.0f;
    pid_last_time = now;
    if (pid_first_run || dt <= 0.0f) {
        dt = 0.001f;
        pid_first_run = false;
    }

    if (is_white) {
        // --- STATE 1: LOST LINE (ALL WHITE) ---
        if (out_state_name) *out_state_name = STATE_NAME_S_LOST_W;
        steer = -KP_SINGLE * 1.2f; // Spin RIGHT
        pid_integral = 0.0f;
        pid_last_error = 0.0f;

    } else if (is_black) {
        // --- STATE 2: LOST LINE (ALL BLACK) ---
        if (out_state_name) *out_state_name = STATE_NAME_S_LOST_B;
        steer = KP_SINGLE * 1.2f; // Spin LEFT
        pid_integral = 0.0f;
        pid_last_error = 0.0f;

    } else {
        // --- STATE 3: ON THE EDGE (PID TRACKING) ---
        if (out_state_name) *out_state_name = STATE_NAME_S_TRACK;
        float error = L_norm - NORM_SETPOINT_SINGLE;
        pid_integral += error * dt;
        pid_integral = clampf(pid_integral, -0.5f, 0.5f);
        float derivative = (error - pid_last_error) / dt;
        steer = (KP_SINGLE * error) + (KI_SINGLE * pid_integral) + (KD_SINGLE * derivative);
        pid_last_error = error;
    }

    // --- 4. Calculate final motor speeds ---
    float current_base_duty = base_speed_percent / 100.0f;
    float left_duty  = clampf(current_base_duty + steer, 0.f, 1.f); // Right Edge: +steer
    float right_duty = clampf(current_base_duty - steer, 0.f, 1.f); // Right Edge: -steer
    float left_percent = left_duty * 100.0f;
    float right_percent = right_duty * 100.0f;

    // --- 5. Set Motors and report values ---
    motor_set_speed(MOTOR_LEFT,  left_percent);
    motor_set_speed(MOTOR_RIGHT, right_percent);

    if (out_left_power)  *out_left_power  = left_percent;
    if (out_right_power) *out_right_power = right_percent;
    if (out_L_adc) *out_L_adc = L_adc;
}