#ifndef IR_DRIVER_H
#define IR_DRIVER_H

#include "pico/stdlib.h"

void ir_init(void);
void ir_reset_pid_state(void);
void ir_set_calibration_values(uint16_t white, uint16_t black);
uint16_t ir_read_single_sensor_raw(void);

void ir_follow_line_single(float base_speed_tps,
                           float *out_target_left_tps, float *out_target_right_tps,
                           uint16_t *out_R_adc,
                           const char **out_state_name);

/**
 * @brief NEW: Sets the IR PID tuning parameters at runtime.
 */
void ir_set_tuning_params(float kp, float ki, float kd, 
                          float steer_clamp, float base_scale, 
                          float curve_slowdown, float curve_exp, 
                          float min_speed_frac);

#endif // IR_DRIVER_H