// ir.h

#ifndef IR_H
#define IR_H

#include <stdint.h>

// --- NEW: Public state definitions ---
#define IR_STATE_TRACK_RIGHT 0
#define IR_STATE_TRACK_LEFT  1

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
                           uint32_t *out_time_lost_ms // --- NEW ---
                          );

#endif // IR_H