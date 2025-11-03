#ifndef _IR_H_
#define _IR_H_

#include <stdint.h>

/**
 * @brief Initializes the IR driver.
 */
void ir_init(void);

/**
 * @brief Resets the internal state of the line-following PID controller.
 * Call this when changing modes or stopping.
 */
void ir_reset_pid_state(void);

/**
 * @brief Sets the new white and black calibration values for the sensor.
 * @param white Raw ADC value for the white surface.
 * @param black Raw ADC value for the black surface.
 */
void ir_set_calibration_values(uint16_t white, uint16_t black);

/**
 * @brief Reads the raw, averaged ADC value from the single line-following sensor.
 * Used for the calibration process.
 * @return Raw ADC value (0-4095).
 */
uint16_t ir_read_single_sensor_raw(void);

/**
 * @brief Runs the single-sensor line-following PID logic.
 * This is a "cascade" controller: it calculates the *target speeds* (in tps)
 * for the motors, which are then passed to the motor speed PIDs in main.c.
 *
 * @param base_speed_tps The base speed (in ticks-per-second) for the robot.
 * @param out_target_left_tps Pointer to store the calculated left wheel target speed.
 * @param out_target_right_tps Pointer to store the calculated right wheel target speed.
 * @param out_R_adc Pointer to store the raw ADC value read (for debugging).
 * @param out_state_name Pointer to store the current state name (for debugging).
 */
void ir_follow_line_single(float base_speed_tps,
                           float *out_target_left_tps, float *out_target_right_tps,
                           uint16_t *out_R_adc,
                           const char **out_state_name);

#endif // _IR_H_