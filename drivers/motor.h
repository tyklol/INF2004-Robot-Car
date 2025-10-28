#ifndef DRIVERS_MOTOR_H_
#define DRIVERS_MOTOR_H_

// Define identifiers for the left and right motors
#define MOTOR_LEFT  1
#define MOTOR_RIGHT 2

/**
 * @brief Initializes the GPIO pins and PWM slices for both motors.
 * This must be called once at startup.
 */
void motor_init(void);

/**
 * @brief Sets the speed and direction for a specific motor.
 *
 * @param motor_id Which motor to control (use MOTOR_LEFT or MOTOR_RIGHT).
 * @param speed    The speed to set, from -100.0 (full reverse) to 
 * 100.0 (full forward). A value of 0.0 will stop (coast)
 * the motor.
 */
void motor_set_speed(int motor_id, float speed);

#endif // DRIVERS_MOTOR_H_