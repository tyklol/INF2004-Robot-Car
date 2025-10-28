#ifndef DRIVERS_IR_H_
#define DRIVERS_IR_H_

#include "pico/stdlib.h" // For uint16_t

/**
 * @brief Initializes the ADC pins for the IR line sensors.
 */
void ir_init(void);

/**
 * @brief Resets the internal memory (Integral, last error) of the PID controllers.
 * Call this when changing modes to prevent old data from causing a spin.
 */
void ir_reset_pid_state(void);

/**
 * @brief Runs a dual-sensor proportional-control line following step.
 * (This function is unchanged)
 */
void ir_follow_line_double(float base_speed_percent, 
                           float *out_left_power, float *out_right_power, 
                           uint16_t *out_L_adc, uint16_t *out_R_adc,
                           const char **out_state_name);

/**
 * @brief Runs a "perfected" single-sensor line following step with full PID.
 *
 * @param base_speed_percent The nominal speed (0-100) to run at.
 * @param out_left_power     Pointer to a float to store the calculated left power.
 * @param out_right_power    Pointer to a float to store the calculated right power.
 * @param out_L_adc          Pointer to store the raw Left ADC value.
 * @param out_state_name     Pointer to a char* to store the name of the current state.
 */
void ir_follow_line_single(float base_speed_percent, 
                           float *out_left_power, float *out_right_power, 
                           uint16_t *out_L_adc,
                           const char **out_state_name);

#endif // DRIVERS_IR_H_