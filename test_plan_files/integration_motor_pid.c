#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <math.h> // For fabsf()
#include <stdlib.h> // For labs()

// --- Driver Headers ---
#include "drivers/motor.h" 
#include "drivers/encoder.h" 
#include "drivers/pid_controller.h"

/*
 * NOTE: Your motor.h must define these
 */
#ifndef MOTOR_LEFT
#define MOTOR_LEFT 0
#endif
#ifndef MOTOR_RIGHT
#define MOTOR_RIGHT 1
#endif

// --- Test Configuration ---
#define LOOP_DELAY_MS 10        // Run control loop at 100Hz
#define LOOP_DELAY_S 0.01f      // Loop delay in seconds (for PID dt)
#define TEST_DURATION_MS 2000   // Run each test for 2 seconds
#define TARGET_SPEED 20.0f      // Target speed: 20 ticks per loop

// --- PID Tuning Gains (MUST BE TUNED FOR YOUR ROBOT) ---
// These are just example values to make it move.
#define KP 5.0f   // Proportional gain (e.g., 5.0 * 20-tick error = 100% power)
#define KI 0.5f   // Integral gain
#define KD 0.0f   // Derivative gain (often 0 for simple speed control)
#define PID_MIN_OUT -100.0f
#define PID_MAX_OUT 100.0f

// --- Global PID State Variables ---
static pid_state_t g_pid_left;
static pid_state_t g_pid_right;


/**
 * @brief Pauses between tests and prints a separator
 */
static void next_test_pause(void) {
    printf("\n... (pausing for 2 seconds) ...\n\n");
    sleep_ms(2000);
}

/**
 * @brief This is the core of the test.
 * It runs the full PID control loop for a set duration.
 */
static void run_pid_loop(float target_speed_l, float target_speed_r) {
    
    // Reset system state for a new test
    encoder_reset_total_ticks();
    
    // Re-initialize PID controllers to reset integral and error history
    pid_init(&g_pid_left, KP, KI, KD, PID_MIN_OUT, PID_MAX_OUT, 0.02f);
    pid_init(&g_pid_right, KP, KI, KD, PID_MIN_OUT, PID_MAX_OUT, 0.02f);
    
    // Get the end time for the test
    absolute_time_t end_time = make_timeout_time_ms(TEST_DURATION_MS);

    // Loop for the duration of the test
    while (absolute_time_diff_us(get_absolute_time(), end_time) > 0) {
        
        // 1. Get Feedback (Current Speed)
        int32_t delta_l, delta_r;
        encoder_update_and_get_ticks(&delta_l, &delta_r);

        // 2. Calculate PID Output
        float output_l = pid_calculate(&g_pid_left, target_speed_l, (float)delta_l, LOOP_DELAY_S);
        float output_r = pid_calculate(&g_pid_right, target_speed_r, (float)delta_r, LOOP_DELAY_S);
        
        // 3. Control Motors
        motor_set_speed(MOTOR_LEFT, output_l);
        motor_set_speed(MOTOR_RIGHT, output_r);
        
        sleep_ms(LOOP_DELAY_MS); // Maintain 100Hz loop rate
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
/**
 * @brief Main test harness
 */
int main() {
    stdio_init_all();
    sleep_ms(3000); // Wait for serial monitor to connect

    printf("--- Automated PID Motor Control Integration Test ---\n");
    printf("--- (Robot MUST be on blocks!) ---\n");
    
    // 1. Initialize all drivers
    motor_init();
    encoder_init();
    // PID controllers are initialized in the run_pid_loop() function
    
    printf("\nPress ENTER in the serial monitor to start all tests...\n");
    getchar(); // Wait for a single key press

    // --- Variables for results ---
    int32_t total_l, total_r;
    bool pass;

    // =======================================================
    // --- Test 1: Steady State (Zero Target) ---
    // =======================================================
    printf("=== [Test PID-IT-01: Steady State (Zero Target)] ===\n");
    printf("... Setting Target Speed: Left=0.0, Right=0.0 for %d ms ...\n", TEST_DURATION_MS);
    
    run_pid_loop(0.0f, 0.0f);
    
    total_l = encoder_get_total_ticks_left();
    total_r = encoder_get_total_ticks_right();

    printf("--- Actual State ---\n");
    printf("  Total Left Ticks: %ld\n", total_l);
    printf("  Total Right Ticks: %ld\n", total_r);
    printf("Expected: Left Ticks < 5, Right Ticks < 5\n");

    // Use labs() (long absolute value) for tolerance
    pass = (labs(total_l) < 5 && labs(total_r) < 5);
    printf("Result: %s\n", pass ? "Pass" : "Fail");
    next_test_pause();


    // =======================================================
    // --- Test 2: Left Motor Speed Control ---
    // =======================================================
    printf("=== [Test PID-IT-02: Left Motor Speed Control] ===\n");
    printf("... Setting Target Speed: Left=%.1f, Right=0.0 for %d ms ...\n", TARGET_SPEED, TEST_DURATION_MS);

    run_pid_loop(TARGET_SPEED, 0.0f); 

    total_l = encoder_get_total_ticks_left();
    total_r = encoder_get_total_ticks_right();

    printf("--- Actual State ---\n");
    printf("  Total Left Ticks: %ld\n", total_l);
    printf("  Total Right Ticks: %ld\n", total_r);
    
    // --- UPDATED EXPECTATION ---
    printf("Expected: Left Ticks > 40, Right Ticks < 5\n");
    pass = (total_l > 40 && labs(total_r) < 5);
    // --- END UPDATE ---

    printf("Result: %s\n", pass ? "Pass" : "Fail");
    next_test_pause();

    // =======================================================
    // --- Test 3: Right Motor Speed Control ---
    // =======================================================
    printf("=== [Test PID-IT-03: Right Motor Speed Control] ===\n");
    printf("... Setting Target Speed: Left=0.0, Right=%.1f for %d ms ...\n", TARGET_SPEED, TEST_DURATION_MS);

    run_pid_loop(0.0f, TARGET_SPEED); // Left OFF, Right ON

    total_l = encoder_get_total_ticks_left();
    total_r = encoder_get_total_ticks_right();

    printf("--- Actual State ---\n");
    printf("  Total Left Ticks: %ld\n", total_l);
    printf("  Total Right Ticks: %ld\n", total_r);
    
    // --- UPDATED EXPECTATION ---
    printf("Expected: Left Ticks < 5, Right Ticks > 40\n");
    pass = (labs(total_l) < 5 && total_r > 40);
    // --- END UPDATE ---

    printf("Result: %s\n", pass ? "Pass" : "Fail");
    next_test_pause();

    // =======================================================
    // --- Test 4: Dual Motor Control & Synchronization ---
    // =======================================================
    printf("=== [Test PID-IT-04: Dual Motor Control & Synchronization] ===\n");
    printf("... Setting Target Speed: Left=%.1f, Right=%.1f for %d ms ...\n", TARGET_SPEED, TARGET_SPEED, TEST_DURATION_MS);

    run_pid_loop(TARGET_SPEED, TARGET_SPEED); // Left ON, Right ON

    total_l = encoder_get_total_ticks_left();
    total_r = encoder_get_total_ticks_right();

    printf("--- Actual State ---\n");
    printf("  Total Left Ticks: %ld\n", total_l);
    printf("  Total Right Ticks: %ld\n", total_r);
    
    // Check synchronization (e.g., within 20%)
    float avg_ticks = (float)(total_l + total_r) / 2.0f;
    float percent_diff = 0.0f; // Default to 0 if no ticks
    
    // Add divide-by-zero check (which you fixed before)
    if (avg_ticks > 1.0f) {
        percent_diff = 100.0f * fabsf((float)(total_l - total_r)) / avg_ticks;
    }

    printf("  Tick Difference: %.1f%%\n", percent_diff);
    
    // --- UPDATED EXPECTATION ---
    printf("Expected: Left Ticks > 40, Right Ticks > 40, Difference < 20.0%%\n");
    pass = (total_l > 40 && total_r > 40 && percent_diff < 20.0f);
    // --- END UPDATE ---
    
    printf("Result: %s\n", pass ? "Pass" : "Fail");
    
    printf("\n--- ALL TESTS COMPLETE ---\n");

    while (true) {
        sleep_ms(1000);
    }

    return 0; // Unreachable
}