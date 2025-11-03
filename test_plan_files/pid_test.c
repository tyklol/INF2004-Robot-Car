#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h> // For fabsf()

// --- IMPORTANT ---
// Include the new advanced driver
#include "drivers/pid_controller.h" 

// --- Test State ---
static pid_state_t pid;
static const float dt = 1.0f; // Use 1.0s for simple math
static int test_count = 0;
static int pass_count = 0;

/**
 * @brief Helper to check if two floats are "close enough"
 */
static bool check_float(float a, float b, float tolerance) {
    return fabsf(a - b) < tolerance;
}

/**
 * @brief Helper to print test results
 */
static void print_result(const char* test_name, bool pass) {
    test_count++;
    if (pass) {
        pass_count++;
        printf("  Result: PASS\n");
    } else {
        printf("  Result: FAIL\n");
    }
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
    printf("=== [Test PID-UT-01: Verify Proportional (P) Term] ===\n");
    printf("Expected: Output = 10.0\n");
    
    // Test: Kp=1.0, Ki=0, Kd=0. d_tau_s=0 (no filter)
    pid_init(&pid, 1.0f, 0.0f, 0.0f, -100.0f, 100.0f, 0.0f);
    float output = pid_calculate(&pid, 10.0f, 0.0f, dt);

    printf("--- Actual State ---\n");
    printf("  Output: %.3f\n", output);
    
    print_result("P-Term", check_float(output, 10.0f, 0.001f));
}

void run_test_02() {
    printf("=== [Test PID-UT-02: Verify Integral (I) Term] ===\n");
    printf("Expected: Output steps: 5.0 -> 10.0 -> 15.0\n");
    
    // Test: Kp=0, Ki=0.5, Kd=0. d_tau_s=0
    pid_init(&pid, 0.0f, 0.5f, 0.0f, -100.0f, 100.0f, 0.0f);
    
    float out1 = pid_calculate(&pid, 10.0f, 0.0f, dt); // integral = 10. out = 5.0
    float out2 = pid_calculate(&pid, 10.0f, 0.0f, dt); // integral = 20. out = 10.0
    float out3 = pid_calculate(&pid, 10.0f, 0.0f, dt); // integral = 30. out = 15.0
    
    printf("--- Actual State ---\n");
    printf("  Output 1: %.3f\n", out1);
    printf("  Output 2: %.3f\n", out2);
    printf("  Output 3: %.3f\n", out3);
    
    bool pass = check_float(out1, 5.0f, 0.001f) &&
                  check_float(out2, 10.0f, 0.001f) &&
                  check_float(out3, 15.0f, 0.001f);
    print_result("I-Term", pass);
}

void run_test_03() {
    printf("=== [Test PID-UT-03: Verify Derivative (D) Term (on Measurement)] ===\n");
    printf("Expected: Output steps: 0.0 -> -20.0 -> 0.0\n");
    
    // Test: Kp=0, Ki=0, Kd=2.0. d_tau_s=0
    // D-Term is -kd * (current - last_meas) / dt
    pid_init(&pid, 0.0f, 0.0f, 2.0f, -100.0f, 100.0f, 0.0f);
    
    float out1 = pid_calculate(&pid, 0.0f, 0.0f, dt);  // last_meas=0. d_raw=0. out=0.
    float out2 = pid_calculate(&pid, 0.0f, 10.0f, dt); // last_meas=10. d_raw=(10-0)/1=10. out = -2.0 * 10 = -20.
    float out3 = pid_calculate(&pid, 0.0f, 10.0f, dt); // last_meas=10. d_raw=(10-10)/1=0. out = 0.
    
    printf("--- Actual State ---\n");
    printf("  Output 1: %.3f\n", out1);
    printf("  Output 2: %.3f\n", out2);
    printf("  Output 3: %.3f\n", out3);
    
    bool pass = check_float(out1, 0.0f, 0.001f) &&
                  check_float(out2, -20.0f, 0.001f) &&
                  check_float(out3, 0.0f, 0.001f);
    print_result("D-Term (on Meas)", pass);
}

void run_test_04() {
    printf("=== [Test PID-UT-04: Verify Headroom Anti-Windup] ===\n");
    printf("Expected: Output=80.0, Integral=-9200.0\n");
    
    // Test: Kp=1.0, Ki=0.1, Kd=0. Clamp output at 80. d_tau_s=0
    pid_init(&pid, 1.0f, 0.1f, 0.0f, -80.0f, 80.0f, 0.0f);
    
    // P-term = 1.0 * 1000 = 1000.
    // u_pd = 1000.
    // i_max = max_out - u_pd = 80 - 1000 = -920.
    // i_min = min_out - u_pd = -80 - 1000 = -1080.
    // integral = error * dt = 1000. i_term = 0.1 * 1000 = 100.
    // i_term_clamped = clampf(100, -1080, -920) = -920.
    // output = u_pd + i_term_clamped = 1000 + (-920) = 80.
    // pid.integral = i_term_clamped / ki = -920 / 0.1 = -9200.
    float out1 = pid_calculate(&pid, 1000.0f, 0.0f, dt);
    
    printf("--- Actual State ---\n");
    printf("  Output 1: %.3f\n", out1);
    printf("  Internal Integral: %.3f\n", pid.integral);
    
    bool pass = check_float(out1, 80.0f, 0.001f) &&
                  check_float(pid.integral, -9200.0f, 0.001f);
    print_result("Clamping", pass);
}

void run_test_05() {
    printf("=== [Test PID-UT-05: Verify Steady State (Zero Error)] ===\n");
    printf("Expected: Output=0.0, Integral=0.0\n");

    pid_init(&pid, 1.5f, 0.5f, 0.1f, -100.0f, 100.0f, 0.0f);
    pid_reset(&pid); // Ensure integral is 0
    
    float output = pid_calculate(&pid, 20.0f, 20.0f, dt); // Zero error
    
    printf("--- Actual State ---\n");
    printf("  Output: %.3f\n", output);
    printf("  Internal Integral: %.3f\n", pid.integral);
    
    bool pass = check_float(output, 0.0f, 0.001f) &&
                  check_float(pid.integral, 0.0f, 0.001f);
    print_result("Steady State", pass);
}

void run_test_06() {
    printf("=== [Test PID-UT-06: Verify D-Term Low-pass Filter] ===\n");
    printf("Expected: Output steps: 0.0 -> -5.0 -> -2.5\n");
    
    // Test: Kp=0, Ki=0, Kd=1.0. Tau=1.0s
    // dt=1.0s. alpha = dt / (tau + dt) = 1.0 / (1.0 + 1.0) = 0.5
    pid_init(&pid, 0.0f, 0.0f, 1.0f, -100.0f, 100.0f, 1.0f);
    
    // Step 1: Init. d_raw=0, d_filtered=0. out=0
    float out1 = pid_calculate(&pid, 0.0f, 0.0f, dt);
    
    // Step 2: d_raw=10. d_filtered = 0 + 0.5 * (10 - 0) = 5.0. out = -kd * 5.0 = -5.0
    float out2 = pid_calculate(&pid, 0.0f, 10.0f, dt);
    
    // Step 3: d_raw=0. d_filtered = 5.0 + 0.5 * (0 - 5.0) = 2.5. out = -kd * 2.5 = -2.5
    float out3 = pid_calculate(&pid, 0.0f, 10.0f, dt);
    
    printf("--- Actual State ---\n");
    printf("  Output 1: %.3f\n", out1);
    printf("  Output 2: %.3f\n", out2);
    printf("  Output 3: %.3f\n", out3);
    
    bool pass = check_float(out1, 0.0f, 0.001f) &&
                  check_float(out2, -5.0f, 0.001f) &&
                  check_float(out3, -2.5f, 0.001f);
    print_result("D-Term Filter", pass);
}


/**
 * @brief Main test loop
 */
int main() {
    stdio_init_all();
    sleep_ms(3000); // Wait for serial monitor to connect

    printf("Advanced PID Controller Test Harness Initializing...\n");
    
    printf("\nPress ENTER in the serial monitor to start all tests...\n");
    getchar(); // Wait for a single key press

    // --- Run all tests sequentially ---
    run_test_01();
    wait_for_next_test(1000);

    run_test_02();
    wait_for_next_test(1000);
    
    run_test_03();
    wait_for_next_test(1000);
    
    run_test_04();
    wait_for_next_test(1000);
    
    run_test_05();
    wait_for_next_test(1000);

    run_test_06();
    
    printf("\n--- ALL TESTS COMPLETE ---\n");
    printf("Summary: %d / %d tests passed.\n", pass_count, test_count);

    while (true) {
        sleep_ms(1000);
    }

    return 0; // Unreachable
}