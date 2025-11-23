#ifndef IR_H
#define IR_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/adc.h"

// ---- Pin map ----
#define PIN_IR_RIGHT_ADC_GPIO 26
#define PIN_IR_RIGHT_ADC_CH   0
#define ADC_SAMPLES           12

// ---- Calibration ----
static float SENSOR_WHITE = 140.0f;  // Default safety values
static float SENSOR_BLACK = 2000.0f;

// ---- PID Constants ----
#define PID_SETPOINT     0.5f // We aim for the "Grey" middle

typedef struct {
    float kp;
    float ki;
    float kd;
    float steer_clamp;
    float base_scale;
} ir_tuning_params_t;

// Default Tuning (Adjust these!)
static ir_tuning_params_t g_params = {
    .kp = 30.0f,        // Start here. If it doesn't turn enough, increase.
    .ki = 0.0f,         // Keep 0 initially.
    .kd = 2.0f,         // If it wobbles, increase this.
    .steer_clamp = 30.0f, // Max turning correction
    .base_scale = 1.0f
};

// Internal PID State
static float pid_integral = 0.0f;
static float pid_last_error = 0.0f;
static absolute_time_t pid_last_time;

// Function Prototypes
void ir_init(void);
void ir_set_tuning_params(float kp, float ki, float kd, float steer_clamp, float base_scale);
void ir_reset_pid_state(void);
void ir_set_calibration_values(uint16_t white, uint16_t black);
uint16_t ir_read_single_sensor_raw(void);

void ir_follow_line_single(float base_speed_tps,
                           float *out_target_left_tps, float *out_target_right_tps,
                           uint16_t *out_R_adc);

#endif // IR_H