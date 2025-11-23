#include "motor.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <math.h> 

static uint slice_num_left;
static uint slice_num_right;

static void _set_single_motor_speed(uint pin_fwd, uint pin_rev, float speed_float) {
    if (speed_float > 1.0f)  speed_float = 1.0f;
    if (speed_float < -1.0f) speed_float = -1.0f;
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
    } else { // Stop
        gpio_set_function(pin_fwd, GPIO_FUNC_SIO);
        gpio_set_function(pin_rev, GPIO_FUNC_SIO);
        gpio_set_dir(pin_fwd, GPIO_OUT);
        gpio_set_dir(pin_rev, GPIO_OUT);
        gpio_put(pin_fwd, 0);
        gpio_put(pin_rev, 0);
    }
}

void motor_init(void) {
    // Pins are defined in sys_config.h, included via motor.h
    slice_num_left  = pwm_gpio_to_slice_num(MOTOR_LEFT_FWD_PIN);
    slice_num_right = pwm_gpio_to_slice_num(MOTOR_RIGHT_FWD_PIN);

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
        _set_single_motor_speed(MOTOR_LEFT_FWD_PIN, MOTOR_LEFT_REV_PIN, speed_float);
    } else if (motor_id == MOTOR_RIGHT) {
        _set_single_motor_speed(MOTOR_RIGHT_FWD_PIN, MOTOR_RIGHT_REV_PIN, speed_float);
    }
}