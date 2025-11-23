#ifndef IR_DRIVER_H
#define IR_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "sys_config.h" // Gets PIN_IR_RIGHT_ADC_CH, etc.

// --- State Machine ---
typedef enum {
    TRACK_RIGHT_EDGE,
    TRACK_LEFT_EDGE
} ir_state_t;

// --- State Constants ---
#define IR_STATE_TRACK_RIGHT 0
#define IR_STATE_TRACK_LEFT  1

// ===== ROBOT CONFIG =====
#define ALLOW_REVERSE    0
#define ADC_SAMPLES      12
#define PID_SETPOINT     0.5f

// --- Calibration ---
extern float SENSOR_WHITE;
extern float SENSOR_BLACK;

// --- Tuning Parameters ---
typedef struct {
    float kp;
    float ki;
    float kd;
    float steer_clamp;
    float base_scale;
    float curve_slowdown;
    float curve_exp;
    float min_speed_frac;
    uint32_t timeout_R_to_L_ms;
    uint32_t timeout_L_to_R_ms;
} ir_tuning_params_t;

extern ir_tuning_params_t g_params;
extern ir_state_t g_current_state; 

void ir_init(void);
void ir_reset_pid_state(void);
void ir_reset_robot_state(void);
void ir_force_state(int state);
void ir_set_calibration_values(uint16_t white, uint16_t black);
uint16_t ir_read_single_sensor_raw(void);
void ir_set_tuning_params(float kp, float ki, float kd, float steer_clamp, float base_scale, 
                          float curve_slowdown, float curve_exp, float min_speed_frac,
                          uint32_t timeout_white, uint32_t timeout_black);

void ir_follow_line_single(float base_speed_tps,
                           float *out_target_left_tps, float *out_target_right_tps,
                           uint16_t *out_R_adc,
                           const char **out_state_name,
                           uint32_t *out_time_lost_ms,
                           float yaw_rate_dps);

#endif // IR_DRIVER_H