/**
 * @file main.c
 * @brief Main program for INF2004 Robot Car using Raspberry Pi Pico W
 * 
 * This file contains the main control logic for the robot car.
 * The robot car uses PWM for motor control and GPIO for basic I/O operations.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

// Motor control pins
#define MOTOR_LEFT_FORWARD_PIN  2
#define MOTOR_LEFT_BACKWARD_PIN 3
#define MOTOR_RIGHT_FORWARD_PIN 4
#define MOTOR_RIGHT_BACKWARD_PIN 5

// LED pin (built-in LED on Pico W is controlled via CYW43)
#define LED_BLINK_INTERVAL_MS 500

/**
 * @brief Initialize GPIO pins for motor control
 */
void init_motors(void) {
    // Initialize motor pins as outputs
    gpio_init(MOTOR_LEFT_FORWARD_PIN);
    gpio_set_dir(MOTOR_LEFT_FORWARD_PIN, GPIO_OUT);
    
    gpio_init(MOTOR_LEFT_BACKWARD_PIN);
    gpio_set_dir(MOTOR_LEFT_BACKWARD_PIN, GPIO_OUT);
    
    gpio_init(MOTOR_RIGHT_FORWARD_PIN);
    gpio_set_dir(MOTOR_RIGHT_FORWARD_PIN, GPIO_OUT);
    
    gpio_init(MOTOR_RIGHT_BACKWARD_PIN);
    gpio_set_dir(MOTOR_RIGHT_BACKWARD_PIN, GPIO_OUT);
    
    // Set all motor pins to low initially
    gpio_put(MOTOR_LEFT_FORWARD_PIN, 0);
    gpio_put(MOTOR_LEFT_BACKWARD_PIN, 0);
    gpio_put(MOTOR_RIGHT_FORWARD_PIN, 0);
    gpio_put(MOTOR_RIGHT_BACKWARD_PIN, 0);
}

/**
 * @brief Move the robot car forward
 */
void move_forward(void) {
    gpio_put(MOTOR_LEFT_FORWARD_PIN, 1);
    gpio_put(MOTOR_LEFT_BACKWARD_PIN, 0);
    gpio_put(MOTOR_RIGHT_FORWARD_PIN, 1);
    gpio_put(MOTOR_RIGHT_BACKWARD_PIN, 0);
}

/**
 * @brief Move the robot car backward
 */
void move_backward(void) {
    gpio_put(MOTOR_LEFT_FORWARD_PIN, 0);
    gpio_put(MOTOR_LEFT_BACKWARD_PIN, 1);
    gpio_put(MOTOR_RIGHT_FORWARD_PIN, 0);
    gpio_put(MOTOR_RIGHT_BACKWARD_PIN, 1);
}

/**
 * @brief Stop the robot car
 */
void stop_motors(void) {
    gpio_put(MOTOR_LEFT_FORWARD_PIN, 0);
    gpio_put(MOTOR_LEFT_BACKWARD_PIN, 0);
    gpio_put(MOTOR_RIGHT_FORWARD_PIN, 0);
    gpio_put(MOTOR_RIGHT_BACKWARD_PIN, 0);
}

/**
 * @brief Turn the robot car left
 */
void turn_left(void) {
    gpio_put(MOTOR_LEFT_FORWARD_PIN, 0);
    gpio_put(MOTOR_LEFT_BACKWARD_PIN, 1);
    gpio_put(MOTOR_RIGHT_FORWARD_PIN, 1);
    gpio_put(MOTOR_RIGHT_BACKWARD_PIN, 0);
}

/**
 * @brief Turn the robot car right
 */
void turn_right(void) {
    gpio_put(MOTOR_LEFT_FORWARD_PIN, 1);
    gpio_put(MOTOR_LEFT_BACKWARD_PIN, 0);
    gpio_put(MOTOR_RIGHT_FORWARD_PIN, 0);
    gpio_put(MOTOR_RIGHT_BACKWARD_PIN, 1);
}

/**
 * @brief Main function
 * @return int Exit status
 */
int main(void) {
    // Initialize standard I/O
    stdio_init_all();
    
    // Initialize the CYW43 chip (for Wi-Fi and LED control on Pico W)
    if (cyw43_arch_init()) {
        printf("Failed to initialize CYW43\n");
        return 1;
    }
    
    printf("INF2004 Robot Car Initialized\n");
    printf("Raspberry Pi Pico W Controller\n");
    
    // Initialize motors
    init_motors();
    
    // Main control loop
    while (true) {
        // Blink the onboard LED to show the program is running
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(LED_BLINK_INTERVAL_MS);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(LED_BLINK_INTERVAL_MS);
        
        // Example movement pattern
        printf("Moving forward\n");
        move_forward();
        sleep_ms(2000);
        
        printf("Stopping\n");
        stop_motors();
        sleep_ms(1000);
        
        printf("Turning right\n");
        turn_right();
        sleep_ms(1000);
        
        printf("Stopping\n");
        stop_motors();
        sleep_ms(1000);
        
        printf("Moving backward\n");
        move_backward();
        sleep_ms(2000);
        
        printf("Stopping\n");
        stop_motors();
        sleep_ms(1000);
        
        printf("Turning left\n");
        turn_left();
        sleep_ms(1000);
        
        printf("Stopping\n");
        stop_motors();
        sleep_ms(1000);
    }
    
    // Cleanup (never reached in this example)
    cyw43_arch_deinit();
    
    return 0;
}
