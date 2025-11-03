#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>

// --- Driver Headers ---
#include "drivers/motor.h" 
#include "drivers/encoder.h" 

/*
 * NOTE: Your motor.h must define these
 */
#ifndef MOTOR_LEFT
#define MOTOR_LEFT 0
#endif
#ifndef MOTOR_RIGHT
#define MOTOR_RIGHT 1
#endif

#define TEST_MOTOR_SPEED 50.0f  // Run motors at 50% power
#define TEST_DURATION_MS 2000   // Run for 2 seconds
#define POLL_LOOP_DELAY_MS 10   // Poll encoders at 100Hz

/**
 * @brief Pauses between tests and prints a separator
 */
static void next_test_pause(void) {
    printf("\n... (pausing for 2 seconds) ...\n\n");
    sleep_ms(2000);
}

/**
 * @brief This is the core of the test.
 * It runs the motors and polls the encoders at the same time.
 */
static void run_and_poll(int motor_l_speed, int motor_r_speed) {
    
    // Get the end time for the test
    absolute_time_t end_time = make_timeout_time_ms(TEST_DURATION_MS);

    // Start the motor(s)
    motor_set_speed(MOTOR_LEFT, motor_l_speed);
    motor_set_speed(MOTOR_RIGHT, motor_r_speed);

    // Loop for the duration of the test
    while (absolute_time_diff_us(get_absolute_time(), end_time) > 0) {
        
        // --- This is the function under test ---
        // We must poll it continuously.
        int32_t delta_l, delta_r;
        encoder_update_and_get_ticks(&delta_l, &delta_r);
        
        sleep_ms(POLL_LOOP_DELAY_MS); // Simulate 100Hz control loop
    }
    
    // Stop motors
    motor_set_speed(MOTOR_LEFT, 0);
    motor_set_speed(MOTOR_RIGHT, 0);

    // Poll one last time to catch any final ticks
    int32_t delta_l, delta_r;
    encoder_update_and_get_ticks(&delta_l, &delta_r);
}


/**
 * @brief Main test harness
 */
int main() {
    stdio_init_all();
    sleep_ms(3000); // Wait for serial monitor to connect

    printf("--- Automated Encoder Integration Test ---\n");
    printf("--- (Robot MUST be on blocks!) ---\n");
    
    // 1. Initialize all drivers
    motor_init();
    encoder_init();
    
    printf("\nPress ENTER in the serial monitor to start all tests...\n");
    getchar(); // Wait for a single key press

    // --- Variables for results ---
    int32_t total_l, total_r;
    bool pass;

    // =======================================================
    // --- Test 1: Verify encoder_init ---
    // =======================================================
    printf("=== [Test E-UT-01: Verify encoder_init] ===\n");
    printf("Expected: GP0 and GP7 are GPIO inputs with pull-ups.\n");
    
    uint fn_l = gpio_get_function(PIN_ENC_L);
    uint fn_r = gpio_get_function(PIN_ENC_R);
    bool pull_l = gpio_is_pulled_up(PIN_ENC_L);
    bool pull_r = gpio_is_pulled_up(PIN_ENC_R);

    printf("--- Actual State ---\n");
    printf("  GP%u (Left):  Function=%u, Is_Pulled_Up=%d\n", PIN_ENC_L, fn_l, pull_l);
    printf("  GP%u (Right): Function=%u, Is_Pulled_Up=%d\n", PIN_ENC_R, fn_r, pull_r);
    
    pass = (fn_l == GPIO_FUNC_SIO && pull_l && fn_r == GPIO_FUNC_SIO && pull_r);
    printf("Result: %s\n", pass ? "Pass" : "Fail");
    next_test_pause();


    // =======================================================
    // --- Test 2: Left Motor Ticks ---
    // =======================================================
    printf("=== [Test E-UT-02: Verify Left Encoder Ticks] ===\n");
    printf("... Spinning LEFT motor for %d ms ...\n", TEST_DURATION_MS);
    
    encoder_reset_total_ticks();
    run_and_poll(TEST_MOTOR_SPEED, 0); // Left ON, Right OFF
    
    total_l = encoder_get_total_ticks_left();
    total_r = encoder_get_total_ticks_right();

    printf("--- Actual State ---\n");
    printf("  Left Ticks: %ld\n", total_l);
    printf("  Right Ticks: %ld\n", total_r);
    printf("Expected: Left Ticks > 0, Right Ticks = 0\n");

    pass = (total_l > 0 && total_r == 0);
    printf("Result: %s\n", pass ? "Pass" : "Fail");
    next_test_pause();

    // =======================================================
    // --- Test 3: Right Motor Ticks ---
    // =======================================================
    printf("=== [Test E-UT-03: Verify Right Encoder Ticks] ===\n");
    printf("... Spinning RIGHT motor for %d ms ...\n", TEST_DURATION_MS);

    encoder_reset_total_ticks();
    run_and_poll(0, TEST_MOTOR_SPEED); // Left OFF, Right ON

    total_l = encoder_get_total_ticks_left();
    total_r = encoder_get_total_ticks_right();

    printf("--- Actual State ---\n");
    printf("  Left Ticks: %ld\n", total_l);
    printf("  Right Ticks: %ld\n", total_r);
    printf("Expected: Left Ticks = 0, Right Ticks > 0\n");

    pass = (total_l == 0 && total_r > 0);
    printf("Result: %s\n", pass ? "Pass" : "Fail");
    next_test_pause();

    // =======================================================
    // --- Test 4: Reset Function ---
    // =======================================================
    printf("=== [Test E-UT-04: Verify encoder_reset_total_ticks] ===\n");
    printf("... Verifying ticks (from last test) are reset to 0 ...\n");

    printf("  Ticks BEFORE reset: L_Total=%ld, R_Total=%ld\n", 
        encoder_get_total_ticks_left(), encoder_get_total_ticks_right());

    encoder_reset_total_ticks();

    total_l = encoder_get_total_ticks_left();
    total_r = encoder_get_total_ticks_right();
    
    printf("  Ticks AFTER reset: L_Total=%ld, R_Total=%ld\n", total_l, total_r);
    printf("  Expected: L_Total = 0, R_Total = 0\n");

    pass = (total_l == 0 && total_r == 0);
    printf("Result: %s\n", pass ? "Pass" : "Fail");
    next_test_pause();

    // =======================================================
    // --- Test 5: Independent Control ---
    // =======================================================
    printf("=== [Test E-UT-05: Verify Independent Control] ===\n");
    printf("... Spinning BOTH motors for %d ms ...\n", TEST_DURATION_MS);

    encoder_reset_total_ticks();
    run_and_poll(TEST_MOTOR_SPEED, TEST_MOTOR_SPEED); // Left ON, Right ON

    total_l = encoder_get_total_ticks_left();
    total_r = encoder_get_total_ticks_right();

    printf("--- Actual State ---\n");
    printf("  Left Ticks: %ld\n", total_l);
    printf("  Right Ticks: %ld\n", total_r);
    printf("  Expected: Left Ticks > 0, Right Ticks > 0\n");

    pass = (total_l > 0 && total_r > 0);
    printf("Result: %s\n", pass ? "Pass" : "Fail");
    
    printf("\n--- ALL TESTS COMPLETE ---\n");

    while (true) {
        sleep_ms(1000);
    }

    return 0; // Unreachable
}