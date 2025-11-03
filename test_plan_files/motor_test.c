#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h" // We still need this for the *pin function* defines
#include <stdio.h>

// --- IMPORTANT ---
// We must include your motor driver's header
#include "drivers/motor.h" 

/*
 * NOTE: Your motor.h must define these constants
 */
#ifndef MOTOR_LEFT
#define MOTOR_LEFT 0
#endif
#ifndef MOTOR_RIGHT
#define MOTOR_RIGHT 1
#endif

// --- Pin Definitions (from your motor.c) ---
#define MOTOR_LEFT_FWD  9
#define MOTOR_LEFT_REV  8
#define MOTOR_RIGHT_FWD 10
#define MOTOR_RIGHT_REV 11

/**
 * @brief Helper function to print the detailed hardware state of a motor.
 * This version is 100% verifiable and will compile.
 */
static void print_motor_state(int motor_id) {
    uint pin_fwd, pin_rev;
    const char* motor_name;
    
    if (motor_id == MOTOR_LEFT) {
        pin_fwd = MOTOR_LEFT_FWD; // 9
        pin_rev = MOTOR_LEFT_REV; // 8
        motor_name = "LEFT (GP9/8)";
    } else {
        pin_fwd = MOTOR_RIGHT_FWD; // 10
        pin_rev = MOTOR_RIGHT_REV; // 11
        motor_name = "RIGHT (GP10/11)";
    }

    printf("--- Actual State for %s ---\n", motor_name);
    
    // Get pin functions
    uint fn_fwd = gpio_get_function(pin_fwd);
    uint fn_rev = gpio_get_function(pin_rev);

    // Print FWD pin state
    printf("  FWD (GP%u) Function: ", pin_fwd);
    if (fn_fwd == GPIO_FUNC_PWM) {
        printf("GPIO_FUNC_PWM\n");
    } else if (fn_fwd == GPIO_FUNC_SIO) {
        printf("GPIO_FUNC_SIO (Level=%d)\n", gpio_get(pin_fwd));
    } else {
        printf("Other (%u)\n", fn_fwd);
    }

    // Print REV pin state
    printf("  REV (GP%u) Function: ", pin_rev);
    if (fn_rev == GPIO_FUNC_PWM) {
        printf("GPIO_FUNC_PWM\n");
    } else if (fn_rev == GPIO_FUNC_SIO) {
        printf("GPIO_FUNC_SIO (Level=%d)\n", gpio_get(pin_rev));
    } else {
        printf("Other (%u)\n", fn_rev);
    }
    printf("----------------------------------\n");
}

/**
 * @brief Pauses between tests
 */
static void wait_for_next_test(int ms) {
    printf("\n... (pausing for %d ms) ...\n\n", ms);
    sleep_ms(ms);
}

// --- Test Case Functions ---

void run_test_01() {
    printf("=== [Test M-UT-01: Verify motor_init()] ===\n");
    printf("Expected: All pins GPIO_FUNC_SIO, Level=0.\n");
    // motor_init() was already called in main()
    print_motor_state(MOTOR_LEFT);
    print_motor_state(MOTOR_RIGHT);
}

void run_test_02() {
    printf("=== [Test M-UT-02: Left motor 50%% forward] ===\n");
    printf("Expected: LEFT FWD(GP9)=GPIO_FUNC_PWM, REV(GP8)=GPIO_FUNC_SIO (Level=0).\n");
    motor_set_speed(MOTOR_LEFT, 50.0f);
    print_motor_state(MOTOR_LEFT);
}

void run_test_03() {
    printf("=== [Test M-UT-03: Right motor 100%% reverse] ===\n");
    printf("Expected: RIGHT FWD(GP10)=GPIO_FUNC_SIO (Level=0), REV(GP11)=GPIO_FUNC_PWM.\n");
    motor_set_speed(MOTOR_RIGHT, -100.0f);
    print_motor_state(MOTOR_RIGHT);
}

void run_test_04() {
    printf("=== [Test M-UT-04: Stop/coast from running] ===\n");
    printf("...Setting Left to 60%% for 1 sec...\n");
    motor_set_speed(MOTOR_LEFT, 60.0f);
    sleep_ms(1000);
    printf("...Setting Left to 0%%...\n");
    printf("Expected: LEFT FWD(GP9)=GPIO_FUNC_SIO (Level=0), REV(GP8)=GPIO_FUNC_SIO (Level=0).\n");
    motor_set_speed(MOTOR_LEFT, 0.0f);
    print_motor_state(MOTOR_LEFT);
}

void run_test_05() {
    printf("=== [Test M-UT-05: Deadband threshold] ===\n");
    printf("...Setting Left to +5.0 (should be 0)...\n");
    motor_set_speed(MOTOR_LEFT, 5.0f);
    print_motor_state(MOTOR_LEFT);
    printf("...Setting Left to -5.0 (should be 0)...\n");
    motor_set_speed(MOTOR_LEFT, -5.0f);
    print_motor_state(MOTOR_LEFT);
    printf("Expected (Above): Both pins GPIO_FUNC_SIO, Level=0.\n\n");
    sleep_ms(1000);
    printf("...Setting Left to +6.0 (should be FWD)...\n");
    motor_set_speed(MOTOR_LEFT, 6.0f);
    print_motor_state(MOTOR_LEFT);
    printf("Expected: FWD(GP9)=GPIO_FUNC_PWM, REV(GP8)=GPIO_FUNC_SIO (Level=0).\n\n");
    sleep_ms(1000);
    printf("...Setting Left to -6.0 (should be REV)...\n");
    motor_set_speed(MOTOR_LEFT, -6.0f);
    print_motor_state(MOTOR_LEFT);
    printf("Expected: FWD(GP9)=GPIO_FUNC_SIO (Level=0), REV(GP8)=GPIO_FUNC_PWM.\n");
}

void run_test_06() {
    printf("=== [Test M-UT-06: Clamping > 100%%] ===\n");
    printf("...Setting Right to +150 (should clamp to 100)...\n");
    printf("Expected: RIGHT FWD(GP10)=GPIO_FUNC_PWM, REV(GP11)=GPIO_FUNC_SIO (Level=0).\n");
    motor_set_speed(MOTOR_RIGHT, 150.0f);
    print_motor_state(MOTOR_RIGHT);
    sleep_ms(1000);
    printf("\n...Setting Right to -200 (should clamp to -100)...\n");
    printf("Expected: RIGHT FWD(GP10)=GPIO_FUNC_SIO (Level=0), REV(GP11)=GPIO_FUNC_PWM.\n");
    motor_set_speed(MOTOR_RIGHT, -200.0f);
    print_motor_state(MOTOR_RIGHT);
}

void run_test_07() {
    printf("=== [Test M-UT-07: Direction flip safety] ===\n");
    printf("...Setting Left to +100...\n");
    printf("Expected: FWD(GP9)=GPIO_FUNC_PWM, REV(GP8)=GPIO_FUNC_SIO (Level=0).\n");
    motor_set_speed(MOTOR_LEFT, 100.0f);
    print_motor_state(MOTOR_LEFT);
    sleep_ms(500);
    printf("\n...Setting Left to -100 (immediate flip)...\n");
    printf("Expected: FWD(GP9)=GPIO_FUNC_SIO (Level=0), REV(GP8)=GPIO_FUNC_PWM.\n");
    motor_set_speed(MOTOR_LEFT, -100.0f);
    print_motor_state(MOTOR_LEFT);
}

void run_test_08() {
    printf("=== [Test M-UT-08: Independent control] ===\n");
    printf("...Setting Left to +30 and Right to -60...\n");
    printf("Expected: LEFT FWD(GP9)=GPIO_FUNC_PWM, RIGHT REV(GP11)=GPIO_FUNC_PWM.\n");
    motor_set_speed(MOTOR_LEFT, 30.0f);
    motor_set_speed(MOTOR_RIGHT, -60.0f);
    print_motor_state(MOTOR_LEFT);
    print_motor_state(MOTOR_RIGHT);
}

void run_test_09() {
    printf("=== [Test M-UT-09: Invalid motor ID] ===\n");
    printf("...Setting motor 99 to 50.0...\n");
    printf("Expected: No change. Both motors should be in previous state (from last test).\n");
    motor_set_speed(99, 50.0f);
    print_motor_state(MOTOR_LEFT);
    print_motor_state(MOTOR_RIGHT);
}

void run_test_10_stop_all() {
    printf("=== [Test Stop: Stopping all motors] ===\n");
    printf("Expected: All pins GPIO_FUNC_SIO, Level=0.\n");
    motor_set_speed(MOTOR_LEFT, 0.0f);
    motor_set_speed(MOTOR_RIGHT, 0.0f);
    print_motor_state(MOTOR_LEFT);
    print_motor_state(MOTOR_RIGHT);
}

/**
 * @brief Main test loop
 */
int main() {
    stdio_init_all();
    sleep_ms(3000); // Wait for serial monitor to connect

    printf("Motor Driver Test Harness Initializing...\n");
    
    // This is the first function we test
    motor_init();
    
    printf("\nPress ENTER in the serial monitor to start all tests...\n");
    getchar(); // Wait for a single key press

    // --- Run all tests sequentially ---
    run_test_01();
    wait_for_next_test(2000);

    run_test_02();
    wait_for_next_test(2000);
    
    run_test_03();
    wait_for_next_test(2000);
    
    run_test_04();
    wait_for_next_test(2000);
    
    run_test_05();
    wait_for_next_test(2000);
    
    run_test_06();
    wait_for_next_test(2000);
    
    run_test_07();
    wait_for_next_test(2000);
    
    run_test_08();
    // --- FIX: Corrected typo '2G000' to '2000' ---
    wait_for_next_test(2000);
    
    run_test_09();
    wait_for_next_test(2000);

    run_test_10_stop_all();

    printf("\n--- ALL TESTS COMPLETE ---\n");

    while (true) {
        sleep_ms(1000);
    }

    return 0; // Unreachable
}