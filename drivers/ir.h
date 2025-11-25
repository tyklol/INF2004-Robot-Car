#ifndef IR_H
#define IR_H

#include <stdint.h>
#include <stdbool.h>

void ir_init(void);

void ir_set_calibration_values(uint16_t white_val, uint16_t black_val);

void ir_set_tuning_params(float kp, float ki, float kd, 
                          float steer_clamp, float alpha, 
                          float curve_slowdown, float curve_exp,
                          float min_speed_frac);

void ir_reset_pid_state(void);

// Returns "LOCKED" when in the deadband (going straight)
void ir_follow_line_single(float base_speed, 
                           float *out_left, float *out_right, 
                           uint16_t *out_raw_sensor,
                           const char **out_state_str,
                           uint32_t *out_lost_time);

#endif