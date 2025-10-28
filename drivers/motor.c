#include "motor.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <math.h> 

// --- Pin Definitions (Matching your new wiring) ---
// Motor 1 (Left) - (GP9 / M2B, GP8 / M2A)
#define MOTOR_LEFT_FWD  9
#define MOTOR_LEFT_REV  8

// Motor 2 (Right) - (GP10 / M1A, GP11 / M1B)
#define MOTOR_RIGHT_FWD  10
#define MOTOR_RIGHT_REV  11

// We store the slice numbers globally within this file
static uint slice_num_left;
static uint slice_num_right;

/**
 * @brief Private helper function to control one motor.
 */
static void _set_single_motor_speed(uint pin_fwd, uint pin_rev, float speed_float) {
    // Clamp speed_float to the -1.0 to 1.0 range
    if (speed_float > 1.0f)  speed_float = 1.0f;
    if (speed_float < -1.0f) speed_float = -1.0f;

    // Calculate PWM duty cycle (0 to 65535)
    uint16_t pwm_level = (uint16_t)(fabsf(speed_float) * 65535.0f);

    if (speed_float > 0.05f) { // Forward
        gpio_set_function(pin_rev, GPIO_FUNC_SIO);
        gpio_set_dir(pin_rev, GPIO_OUT);
        gpio_put(pin_rev, 0);
        gpio_set_function(pin_fwd, GPIO_FUNC_PWM);
        pwm_set_gpio_level(pin_fwd, pwm_level);
    
    } else if (speed_float < -0.05f) { // Reverse
        gpio_set_function(pin_fwd, GPIO_FUNC_SIO);
        gpio_set_dir(pin_fwd, GPIO_OUT);
        gpio_put(pin_fwd, 0);
        gpio_set_function(pin_rev, GPIO_FUNC_PWM);
        pwm_set_gpio_level(pin_rev, pwm_level);

    } else { // Stop (Coast)
        gpio_set_function(pin_fwd, GPIO_FUNC_SIO);
        gpio_set_function(pin_rev, GPIO_FUNC_SIO);
        gpio_set_dir(pin_fwd, GPIO_OUT);
        gpio_set_dir(pin_rev, GPIO_OUT);
        gpio_put(pin_fwd, 0);
        gpio_put(pin_rev, 0);
    }
}

// --- Public Functions (defined in motor.h) ---

void motor_init(void) {
    slice_num_left  = pwm_gpio_to_slice_num(MOTOR_LEFT_FWD);
    slice_num_right = pwm_gpio_to_slice_num(MOTOR_RIGHT_FWD);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.f);
    
    pwm_init(slice_num_left, &config, true);
    pwm_init(slice_num_right, &config, true);

    motor_set_speed(MOTOR_LEFT, 0.0f);
    motor_set_speed(MOTOR_RIGHT, 0.0f);
}

void motor_set_speed(int motor_id, float speed) {
    float speed_float = speed / 100.0f;

    if (motor_id == MOTOR_LEFT) {
        _set_single_motor_speed(MOTOR_LEFT_FWD, MOTOR_LEFT_REV, speed_float);
    } else if (motor_id == MOTOR_RIGHT) {
        _set_single_motor_speed(MOTOR_RIGHT_FWD, MOTOR_RIGHT_REV, speed_float);
    }
}