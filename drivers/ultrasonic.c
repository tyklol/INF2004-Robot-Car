/*
 * Obstacle Scanner with Raspberry Pi Pico W
 * * Detects obstacle distance using HC-SR04 ultrasonic sensor.
 * If distance is < 10 cm, uses a servo motor to scan
 * for the obstacle's edges and calculate its width.
 */

#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "math.h" // For M_PI and tan()

// --- Pin Definitions ---
#define SERVO_PIN 15 // Must be a PWM-capable pin (e.g., GP15)
#define TRIG_PIN 4
#define ECHO_PIN 5

// --- Constants ---
#define THRESHOLD_DISTANCE 20.0 // (in cm) Scan if closer than this
#define SOUND_SPEED_DIVISOR 58.0 // For cm (34300 cm/s -> 1 / (34300 / 2 / 1000000) ~= 58.3)
#define PWM_PERIOD_US 20000      // 20ms, which is 50Hz
#define SERVO_MIN_PULSE_US 900   // Corresponds to 0 degrees
#define SERVO_MAX_PULSE_US 2100  // Corresponds to 180 degrees
#define TIMEOUT_US 30000         // 30ms timeout for ultrasonic sensor

// --- Function Prototypes ---
void servo_init();
void ultrasonic_init();
void set_servo_angle(float angle);
float get_distance();
float calculate_width(float center_distance, float left_angle, float right_angle);
float scan_for_width(float center_distance);


/**
 * @brief Main program entry point
 */
float main_ultrasonic() {
    // Initialize stdio for serial output
    // Start at center position
        // 1. Check distance straight ahead
        float distance = get_distance();
        printf("Center distance: %.2f cm\n", distance);
        
        // 2. If object is close, scan for it
        if (distance < THRESHOLD_DISTANCE && distance > 0) {
            printf("Obstacle detected! Scanning for width...\n");
            float width = scan_for_width(distance);
            
            printf("\n--------------------------------------\n");
            printf(">>> OBSTACLE WIDTH: %.2f cm <<<\n", width);
            printf("--------------------------------------\n\n");
            
            // Wait 3 seconds before checking again
            sleep_ms(3000);
        }

        // Wait a bit before next check
        sleep_ms(200);
}

/**
 * @brief Initializes the PWM for the servo motor
 */
void servo_init() {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    uint chan = pwm_gpio_to_channel(SERVO_PIN);

    pwm_config config = pwm_get_default_config();
    
    // Set clock divider to 125 (125MHz / 125 = 1MHz, 1us per tick)
    pwm_config_set_clkdiv(&config, 125.0);
    
    // Set wrap value to 20000 (1MHz / 20000 = 50Hz, 20ms period)
    pwm_config_set_wrap(&config, PWM_PERIOD_US);

    pwm_init(slice_num, &config, true);
}

/**
 * @brief Initializes the GPIO pins for the ultrasonic sensor
 */
void ultrasonic_init() {
    // Trig pin is an output
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0); // Start low

    // Echo pin is an input
    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
}

/**
 * @brief Sets the servo to a specific angle (0-180)
 * * @param angle The angle to set (0.0 to 180.0)
 */
void set_servo_angle(float angle) {
    // Clamp angle to valid range
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // Map angle (0-180) to pulse width (500-2500us)
    float pulse_width_us = (angle / 180.0) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) + SERVO_MIN_PULSE_US;
    
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    uint chan = pwm_gpio_to_channel(SERVO_PIN);
    
    // Set the PWM duty cycle (pulse width)
    pwm_set_chan_level(slice_num, chan, (uint16_t)pulse_width_us);
}

/**
 * @brief Measures distance using the HC-SR04 sensor
 * * @return float The measured distance in cm, or -1.0 on timeout
 */
float get_distance() {
    // 1. Send the 10us trigger pulse
    gpio_put(TRIG_PIN, 0);
    sleep_us(2);
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);

    absolute_time_t start_time, end_time;
    absolute_time_t timeout_start = get_absolute_time();

    // 2. Wait for the echo pulse to start (go HIGH)
    while (gpio_get(ECHO_PIN) == 0) {
        if (absolute_time_diff_us(timeout_start, get_absolute_time()) > TIMEOUT_US) {
            return -1.0; // Return -1 on timeout
        }
        tight_loop_contents();
    }
    start_time = get_absolute_time();

    // 3. Wait for the echo pulse to end (go LOW)
    timeout_start = get_absolute_time(); // Reset timeout
    while (gpio_get(ECHO_PIN) == 1) {
        if (absolute_time_diff_us(timeout_start, get_absolute_time()) > TIMEOUT_US) {
            return -1.0; // Return -1 on timeout
        }
        tight_loop_contents();
    }
    end_time = get_absolute_time();

    // 4. Calculate duration and convert to distance
    uint64_t pulse_duration_us = absolute_time_diff_us(start_time, end_time);
    return (float)pulse_duration_us / SOUND_SPEED_DIVISOR;
}

/**
 * @brief Calculates object width based on trigonometry
 * * @param center_distance The distance to the object at 90 degrees
 * @param left_angle The angle where the left edge was found
 * @param right_angle The angle where the right edge was found
 * @return float The calculated width of the object in cm
 */
float calculate_width(float center_distance, float left_angle, float right_angle) {
    // This calculation assumes the object is a flat plane
    // perpendicular to the 90-degree center line.
    // It uses two right-angled triangles with a common adjacent side (center_distance).

    // 1. Calculate the relative angles from the 90-degree center
    float rel_angle_left = 90.0 - left_angle;
    float rel_angle_right = right_angle - 90.0;

    // 2. Convert these angles from degrees to radians for tan()
    float rad_left = rel_angle_left * (M_PI / 180.0);
    float rad_right = rel_angle_right * (M_PI / 180.0);

    // 3. Calculate the half-widths using tan(angle) = opposite / adjacent
    // opposite = adjacent * tan(angle)
    float width_left = center_distance * tan(rad_left);
    float width_right = center_distance * tan(rad_right);

    // 4. Return the total width
    return width_left + width_right;
}

/**
 * @brief Scans left and right to find the edges of an object
 * * @param center_distance The known distance to the object
 * @return float The calculated width of the object
 */
float scan_for_width(float center_distance) {
    float left_angle = 90.0;
    float right_angle = 90.0;
    float current_distance;
    
    // --- 1. Scan Left (90 down to 0) ---
    printf("  Scanning left...\n");
    for (float angle = 90; angle >= 0; angle -= 2) { // Scan in 2-degree steps
        set_servo_angle(angle);
        sleep_ms(40); // Give servo time to move
        current_distance = get_distance();
        printf("    Angle: %.1f, Dist: %.2f\n", angle, current_distance);

        // If we lost the object (distance > threshold) or got a bad reading
        // The "+ 5.0" is a margin of error
        if (current_distance > (center_distance + 5.0) || current_distance < 0) {
            left_angle = angle + 2; // The last angle we *did* see the object
            break; 
        }
        if (angle == 0) { // Hit the limit
            left_angle = 0;
        }
    }
    printf("  Left edge found at approx: %.1f deg\n", left_angle);

    // --- 2. Scan Right (90 up to 180) ---
    printf("  Scanning right...\n");
    for (float angle = 90; angle <= 180; angle += 2) {
        set_servo_angle(angle);
        sleep_ms(40);
        current_distance = get_distance();
        printf("    Angle: %.1f, Dist: %.2f\n", angle, current_distance);
        
        if (current_distance > (center_distance + 5.0) || current_distance < 0) {
            right_angle = angle - 2; // The last angle we *did* see the object
            break;
        }
         if (angle == 180) { // Hit the limit
            right_angle = 180;
        }
    }
    printf("  Right edge found at approx: %.1f deg\n", right_angle);

    // --- 3. Reset to center ---
    set_servo_angle(90);
    printf("  Scan complete. Resetting to center.\n");
    sleep_ms(100);

    // --- 4. Calculate and return width ---
    return calculate_width(center_distance, left_angle, right_angle);
}