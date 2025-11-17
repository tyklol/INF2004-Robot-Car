#include "ir.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include <math.h>
#include <stdio.h>

// ===== ROBOT CONFIG =====
#define SENSOR_ON_RIGHT  1
#define ALLOW_REVERSE    0

// ---- Pin map ----
#define PIN_IR_RIGHT_ADC_GPIO 26
#define PIN_IR_RIGHT_ADC_CH   0
#define ADC_SAMPLES           12

// ---- Calibration (set at runtime) ----
static float SENSOR_WHITE = 180.0f;
static float SENSOR_BLACK = 2400.0f;
// --- State Machine ---
typedef enum {
    TRACK_RIGHT_EDGE,
    TRACK_LEFT_EDGE
} ir_state_t;
// ---- Line-Following PD Controller ----
#define PID_SETPOINT     0.5f

// --- MODIFIED: Tuning parameters struct ---
typedef struct {
    float kp;
    float ki;
    float kd;
    float steer_clamp;
    float base_scale;
    float curve_slowdown;
    float curve_exp;
    float min_speed_frac;
    uint32_t timeout_R_to_L_ms; // --- NEW ---
    uint32_t timeout_L_to_R_ms; // --- NEW ---
} ir_tuning_params_t;

// Default to your tune (timeouts are placeholders)
static ir_tuning_params_t g_params = {
    .kp = 15.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .steer_clamp = 20.0f,
    .base_scale = 1.0f,
    .curve_slowdown = 0.5f,
    .curve_exp = 1.2f,
    .min_speed_frac = 0.4f,
    .timeout_R_to_L_ms = 320, // --- NEW ---
    .timeout_L_to_R_ms = 635  // --- NEW ---
};


// ===== Internals =====
static float pid_integral = 0.0f;
static float pid_last_error = 0.0f;
static absolute_time_t pid_last_time;

// --- State Machine ---
// (typedef moved to ir.h)

static ir_state_t g_current_state = TRACK_RIGHT_EDGE; // Start state
static uint32_t g_time_lost_ms = 0;

// --- State Names for Telemetry ---
static const char* STATE_NAME_TRACK_R = "S_TRACK_R";
static const char* STATE_NAME_TRACK_L = "S_TRACK_L";
static const char* STATE_NAME_LOST_W_R = "S_LOST_W_R";
static const char* STATE_NAME_LOST_W_L = "S_LOST_W_L";
static const char* STATE_NAME_LOST_B = "S_LOST_B";

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
 * @brief MODIFIED: Public function to update the static tuning params.
 */
void ir_set_tuning_params(float kp, float ki, float kd, 
                          float steer_clamp, float base_scale, 
                          float curve_slowdown, float curve_exp, 
                          float min_speed_frac,
                          uint32_t timeout_R_to_L, // --- NEW ---
                          uint32_t timeout_L_to_R  // --- NEW ---
                         ) 
{
    g_params.kp = kp;
    g_params.ki = ki;
    g_params.kd = kd;
    g_params.steer_clamp = steer_clamp;
    g_params.base_scale = base_scale;
    g_params.curve_slowdown = curve_slowdown;
    g_params.curve_exp = curve_exp;
    g_params.min_speed_frac = min_speed_frac;
    g_params.timeout_R_to_L_ms = timeout_R_to_L; // --- NEW ---
    g_params.timeout_L_to_R_ms = timeout_L_to_R; // --- NEW ---
}

void ir_init(void) {
    ir_reset_robot_state();
}

void ir_reset_pid_state(void) {
    pid_integral = 0.0f;
    pid_last_error = 0.0f;
    pid_last_time = get_absolute_time();
}

/**
 * @brief Resets the full robot state machine to default
 */
void ir_reset_robot_state(void) {
    ir_reset_pid_state();
    g_current_state = TRACK_RIGHT_EDGE; // Default to right edge
    g_time_lost_ms = 0;
}

/**
 * @brief NEW: Forces the state machine into a specific state
 */
void ir_force_state(int state) {
    if (state == IR_STATE_TRACK_LEFT) {
        g_current_state = TRACK_LEFT_EDGE;
    } else {
        g_current_state = TRACK_RIGHT_EDGE;
    }
    g_time_lost_ms = 0;
    ir_reset_pid_state(); // Reset PID on a forced switch
}


void ir_set_calibration_values(uint16_t white, uint16_t black) {
    // (unchanged)
    SENSOR_WHITE = (float)white;
    SENSOR_BLACK = (float)black;
    if (SENSOR_BLACK < SENSOR_WHITE) {
        printf("WARNING: Black ADC < White ADC. Swapping values.\n");
        float temp = SENSOR_BLACK;
        SENSOR_BLACK = SENSOR_WHITE;
        SENSOR_WHITE = temp;
    }
    if (fabsf(SENSOR_BLACK - SENSOR_WHITE) < 100.0f) {
        printf("WARNING: Small IR range. White=%d Black=%d\n", (int)SENSOR_WHITE, (int)SENSOR_BLACK);
        SENSOR_BLACK = SENSOR_WHITE + 500.0f;
    }
    printf("IR Cal: White=%.0f Black=%.0f Range=%.0f\n",
           SENSOR_WHITE, SENSOR_BLACK, SENSOR_BLACK - SENSOR_WHITE);
}

uint16_t ir_read_single_sensor_raw(void) {
    // (unchanged)
    return adc_read_avg(PIN_IR_RIGHT_ADC_CH, ADC_SAMPLES);
}

// --- MODIFIED --- Function signature
void ir_follow_line_single(float base_speed_tps,
                           float *out_target_left_tps, float *out_target_right_tps,
                           uint16_t *out_R_adc,
                           const char **out_state_name,
                           uint32_t *out_time_lost_ms // --- NEW ---
                          ) {
    
    // 1) Read & normalize sensor
    // (unchanged)
    uint16_t R_adc = adc_read_avg(PIN_IR_RIGHT_ADC_CH, ADC_SAMPLES);
    float R_norm = (R_adc - SENSOR_WHITE) / (SENSOR_BLACK - SENSOR_WHITE);
    R_norm = clampf(R_norm, 0.0f, 1.0f);

    // 2) Calculate delta-time (dt)
    // (unchanged)
    absolute_time_t now = get_absolute_time();
    float dt_s = absolute_time_diff_us(pid_last_time, now) / 1e6f;
    if (dt_s <= 0.0f || dt_s > 0.1f) dt_s = 0.01f;
    pid_last_time = now;
    uint32_t dt_ms = (uint32_t)(dt_s * 1000.0f);

    // 3) --- MODIFIED: STATE MACHINE LOGIC ---
    bool line_is_lost_white = (R_norm < 0.1f);

    if (line_is_lost_white) {
        g_time_lost_ms += dt_ms;

        // --- NEW: Select the correct timeout based on the *current* state ---
        uint32_t current_timeout_threshold = 
            (g_current_state == TRACK_RIGHT_EDGE) ? 
            g_params.timeout_R_to_L_ms :  // Use R->L timeout
            g_params.timeout_L_to_R_ms;   // Use L->R timeout

        // Check for timeout
        if (g_time_lost_ms > current_timeout_threshold) {
            // --- TIMEOUT TRIGGER! ---
            if (g_current_state == TRACK_RIGHT_EDGE) {
                g_current_state = TRACK_LEFT_EDGE;
            } else {
                g_current_state = TRACK_RIGHT_EDGE;
            }
            g_time_lost_ms = 0;
            ir_reset_pid_state(); 
        }
    } else {
        g_time_lost_ms = 0;
    }

    // 4) Calculate error based on state
    // (unchanged)
    float error;
    if (g_current_state == TRACK_RIGHT_EDGE) {
        error = R_norm - PID_SETPOINT;
    } else { // TRACK_LEFT_EDGE
        error = PID_SETPOINT - R_norm;
    }

    // 5) Calculate PD terms
    // (unchanged)
    float p_term = g_params.kp * error;
    if (g_params.ki != 0.0f) {
        pid_integral += error * dt_s;
        pid_integral = clampf(pid_integral, -10.0f, 10.0f);
    } else {
        pid_integral = 0.0f;
    }
    float i_term = g_params.ki * pid_integral;
    float derivative = (error - pid_last_error) / dt_s;
    pid_last_error = error;
    float d_term = g_params.kd * derivative;

    // 6) Calculate final steering command
    // (unchanged)
    float steer = p_term + i_term + d_term;
    steer = clampf(steer, -g_params.steer_clamp, +g_params.steer_clamp);

    // 7) Dynamic Speed
    // (unchanged)
    float steer_frac = fabsf(steer) / g_params.steer_clamp; 
    float curve_scale = 1.0f - (g_params.curve_slowdown * powf(steer_frac, g_params.curve_exp));
    curve_scale = clampf(curve_scale, g_params.min_speed_frac, 1.0f);
    float final_base_speed = base_speed_tps * g_params.base_scale * curve_scale;

    // 8) Map steer to wheel targets
    // (unchanged)
    float left_tps  = final_base_speed + steer;
    float right_tps = final_base_speed - steer;

    // 9) Apply "No Reverse" logic
    // (unchanged)
    #if !ALLOW_REVERSE
    if (left_tps  < 0.0f) left_tps  = 0.0f;
    if (right_tps < 0.0f) right_tps = 0.0f;
    #endif

    // 10) Set outputs
    // (unchanged)
    *out_target_left_tps  = left_tps;
    *out_target_right_tps = right_tps;
    if (out_R_adc) *out_R_adc = R_adc;

    // 11) Set state for telemetry
    // (unchanged)
    if (out_state_name) {
        if (line_is_lost_white) {
            *out_state_name = (g_current_state == TRACK_RIGHT_EDGE) ? STATE_NAME_LOST_W_R : STATE_NAME_LOST_W_L;
        } else if (R_norm > 0.9f) {
            *out_state_name = STATE_NAME_LOST_B;
        } else {
            *out_state_name = (g_current_state == TRACK_RIGHT_EDGE) ? STATE_NAME_TRACK_R : STATE_NAME_TRACK_L;
        }
    }

    // 12) --- NEW: Output timer value for tuning ---
    if (out_time_lost_ms) {
        *out_time_lost_ms = g_time_lost_ms;
    }
}