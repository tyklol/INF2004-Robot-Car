// ir.h

#ifndef IR_H
#define IR_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/adc.h"

// --- State Machine ---
typedef enum {
    TRACK_RIGHT_EDGE,
    TRACK_LEFT_EDGE
} ir_state_t;

// --- State Machine ---
#define IR_STATE_TRACK_RIGHT 0
#define IR_STATE_TRACK_LEFT  1

// ===== ROBOT CONFIG =====
#define ALLOW_REVERSE    0

// ---- Pin map ----
#define PIN_IR_RIGHT_ADC_GPIO 26
#define PIN_IR_RIGHT_ADC_CH   0
#define ADC_SAMPLES           12

// ---- Calibration (set at runtime) ----
static float SENSOR_WHITE = 180.0f;
static float SENSOR_BLACK = 2400.0f;

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

extern ir_state_t g_current_state; // Start state
static uint32_t g_time_lost_ms = 0;

// --- State Names for Telemetry ---
static const char* STATE_NAME_TRACK_R = "S_TRACK_R";
static const char* STATE_NAME_TRACK_L = "S_TRACK_L";
static const char* STATE_NAME_LOST_W_R = "S_LOST_W_R";
static const char* STATE_NAME_LOST_W_L = "S_LOST_W_L";
static const char* STATE_NAME_LOST_B = "S_LOST_B";

void ir_init(void);

// --- MODIFIED: Accepts both timeout values ---
void ir_set_tuning_params(float kp, float ki, float kd, 
                          float steer_clamp, float base_scale, 
                          float curve_slowdown, float curve_exp, 
                          float min_speed_frac,
                          uint32_t timeout_R_to_L_ms, // Timeout for R -> L switch
                          uint32_t timeout_L_to_R_ms  // Timeout for L -> R switch
                         );

void ir_reset_pid_state(void);
void ir_reset_robot_state(void);
void ir_force_state(int state); // --- NEW ---

void ir_set_calibration_values(uint16_t white, uint16_t black);

uint16_t ir_read_single_sensor_raw(void);

// --- MODIFIED: Added 'out_time_lost_ms' back for tuning ---
void ir_follow_line_single(float base_speed_tps,
                           float *out_target_left_tps, float *out_target_right_tps,
                           uint16_t *out_R_adc,
                           const char **out_state_name,
                           uint32_t *out_time_lost_ms,
                          float yaw_rate_dps);

#endif // IR_H