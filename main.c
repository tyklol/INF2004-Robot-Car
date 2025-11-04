/*
 * =====================================================================================
 *
 * Filename:  main.c
 *
 * Description:  Main application logic for Demo 1: Straight-line motion
 * using IMU heading PID and encoder telemetry.
 *
 * OPTIMIZED VERSION - Simplified for 2D ground motion
 *
 * =====================================================================================
 */

#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>
#include <math.h>

// Include your project headers
#include "motor.h"
#include "encoder.h"
#include "imu.h"
#include "pid_controller.h"

// --- Demo Configuration ---

// Main control loop frequency (50 Hz)
#define MAIN_LOOP_PERIOD_MS 20
// Base speed to send to motors (e.g., 40% power)
#define BASE_MOTOR_SPEED 40.0f
// Wheel calibration constant
#define CM_PER_TICK 0.51f

// --- Heading PID Configuration ---
// Tuned for smooth straight-line motion
#define HEADING_KP 2.5f
#define HEADING_KI 0.80f
#define HEADING_KD 1.2f

// Steering correction limits (±20%)
#define HEADING_PID_MIN_OUT -20.0f
#define HEADING_PID_MAX_OUT 20.0f
// D-term smoothing for gentle corrections
#define HEADING_D_TAU_S 0.05f

// --- Global State ---
static pid_state_t g_pid_heading;
static float g_target_heading_deg = 0.0f;
static bool g_demo_running = false;

// Heading filter (simple moving average)
#define HEADING_FILTER_SIZE 5
static float heading_buffer[HEADING_FILTER_SIZE] = {0};
static int heading_buffer_idx = 0;
static bool heading_filter_init = false;

/**
 * @brief Filters heading using circular moving average
 * Handles 0°/360° wraparound correctly
 */
static float filter_heading(float raw_heading) {
    if (!heading_filter_init) {
        for (int i = 0; i < HEADING_FILTER_SIZE; i++) {
            heading_buffer[i] = raw_heading;
        }
        heading_filter_init = true;
        return raw_heading;
    }
    
    heading_buffer[heading_buffer_idx] = raw_heading;
    heading_buffer_idx = (heading_buffer_idx + 1) % HEADING_FILTER_SIZE;
    
    // Average using sin/cos to handle wraparound
    float sum_sin = 0.0f, sum_cos = 0.0f;
    for (int i = 0; i < HEADING_FILTER_SIZE; i++) {
        float rad = heading_buffer[i] * (M_PI / 180.0f);
        sum_sin += sinf(rad);
        sum_cos += cosf(rad);
    }
    
    float avg_rad = atan2f(sum_sin, sum_cos);
    float avg_deg = avg_rad * (180.0f / M_PI);
    if (avg_deg < 0.0f) avg_deg += 360.0f;
    
    return avg_deg;
}

/**
 * @brief Calculates shortest angular difference [-180, 180]
 */
static inline float heading_error(float target, float current) {
    float error = target - current;
    while (error > 180.0f) error -= 360.0f;
    while (error <= -180.0f) error += 360.0f;
    return error;
}

/**
 * @brief Publishes telemetry data (stub)
 */
void publish_telemetry(float speed_l_cm_s, float speed_r_cm_s,
                       float dist_l_cm, float dist_r_cm,
                       imu_pose_t *pose, float target_heading,
                       float heading_error, float steering_output) {
    printf(
        "[Telem] Tgt:%.1f Yaw:%.1f Err:%.1f Steer:%.1f | SpdL:%.1f SpdR:%.1f DistL:%.1f DistR:%.1f\n",
        target_heading, pose->yaw_deg, heading_error, steering_output,
        speed_l_cm_s, speed_r_cm_s, dist_l_cm, dist_r_cm);
}

/**
 * @brief Main control loop (50 Hz)
 */
bool main_loop_callback(struct repeating_timer *t) {
    static imu_pose_t current_pose;
    int32_t delta_ticks_left, delta_ticks_right;
    const float dt_s = (float)MAIN_LOOP_PERIOD_MS / 1000.0f;

    // --- 1. Read Sensors ---
    imu_update(&current_pose);
    encoder_update_and_get_ticks(&delta_ticks_left, &delta_ticks_right);
    float filtered_heading = filter_heading(current_pose.yaw_deg);

    // --- 2. Calculate Telemetry ---
    float speed_l_cm_s = ((float)delta_ticks_left * CM_PER_TICK) / dt_s;
    float speed_r_cm_s = ((float)delta_ticks_right * CM_PER_TICK) / dt_s;
    float dist_l_cm = (float)encoder_get_total_ticks_left() * CM_PER_TICK;
    float dist_r_cm = (float)encoder_get_total_ticks_right() * CM_PER_TICK;

    // --- 3. Heading Control ---
    float steering_bias = 0.0f;
    float h_error = 0.0f;
    
    if (g_demo_running) {
        h_error = heading_error(g_target_heading_deg, filtered_heading);
        steering_bias = pid_calculate(&g_pid_heading, 0.0f, h_error, dt_s);

        // Apply differential steering
        float motor_l_speed = BASE_MOTOR_SPEED - steering_bias;
        float motor_r_speed = BASE_MOTOR_SPEED + steering_bias;

        // Clamp to safe range [0, 100]
        motor_l_speed = (motor_l_speed < 0.0f) ? 0.0f : (motor_l_speed > 100.0f) ? 100.0f : motor_l_speed;
        motor_r_speed = (motor_r_speed < 0.0f) ? 0.0f : (motor_r_speed > 100.0f) ? 100.0f : motor_r_speed;

        motor_set_speed(MOTOR_LEFT, motor_l_speed);
        motor_set_speed(MOTOR_RIGHT, motor_r_speed);
    } else {
        motor_set_speed(MOTOR_LEFT, 0.0f);
        motor_set_speed(MOTOR_RIGHT, 0.0f);
    }

    // --- 4. Telemetry ---
    publish_telemetry(speed_l_cm_s, speed_r_cm_s, dist_l_cm, dist_r_cm,
                      &current_pose, g_target_heading_deg, h_error, steering_bias);

    return true;
}

// --- Main Entry Point ---

int main() {
    stdio_init_all();
    sleep_ms(3000);
    printf("=== Demo 1: Straight Line Motion (Optimized) ===\n\n");

    // --- 1. Initialize Hardware ---
    printf("Initializing Motors...\n");
    motor_init();

    printf("Initializing Encoders...\n");
    encoder_init();

    printf("Initializing I2C and IMU (LSM303DLHC)...\n");
    if (!imu_init_i2c0_gp0_gp1()) {
        printf("!! FATAL: I2C init failed.\n");
        while (true);
    }
    if (!lsm303dlhc_init()) {
        printf("!! FATAL: LSM303DLHC sensor not found.\n");
        while (true);
    }

    // --- 2. Calibration ---
    printf("\n=== CALIBRATION ===\n");
    imu_calibrate_bias();
    
    // --- 3. Verify Calibration Quality ---
    printf("Verifying heading stability (robot must be still)...\n");
    float heading_samples[20];
    for (int i = 0; i < 20; i++) {
        imu_pose_t test_pose;
        imu_update(&test_pose);
        heading_samples[i] = test_pose.yaw_deg;
        sleep_ms(50);
    }
    
    // Calculate standard deviation
    float mean = 0.0f;
    for (int i = 0; i < 20; i++) mean += heading_samples[i];
    mean /= 20.0f;
    
    float variance = 0.0f;
    for (int i = 0; i < 20; i++) {
        float diff = heading_samples[i] - mean;
        variance += diff * diff;
    }
    float std_dev = sqrtf(variance / 20.0f);
    
    printf("  Mean heading: %.1f°, Noise (std dev): %.2f°\n", mean, std_dev);
    if (std_dev > 3.0f) {
        printf("  ⚠ WARNING: High noise! Re-calibrate for better results.\n");
        printf("  (Good calibration: std_dev < 2.0°)\n");
    } else {
        printf("  ✓ Calibration quality: GOOD\n");
    }
    printf("\n");
    sleep_ms(1000);

    // --- 4. Initialize PID ---
    pid_init(&g_pid_heading,
             HEADING_KP, HEADING_KI, HEADING_KD,
             HEADING_PID_MIN_OUT, HEADING_PID_MAX_OUT,
             HEADING_D_TAU_S);

    // --- 5. Set Target Heading ---
    printf("Capturing initial heading as target...\n");
    imu_pose_t initial_pose;
    for (int i = 0; i < 10; i++) {
        imu_update(&initial_pose);
        sleep_ms(20);
    }
    g_target_heading_deg = initial_pose.yaw_deg;
    printf("  Target heading: %.2f°\n\n", g_target_heading_deg);

    // --- 6. Start Control Loop ---
    printf("Starting control loop (50 Hz)...\n");
    struct repeating_timer main_timer;
    if (!add_repeating_timer_ms(MAIN_LOOP_PERIOD_MS, main_loop_callback, NULL, &main_timer)) {
        printf("!! FATAL: Failed to start timer.\n");
        while (true);
    }

    // --- 7. Go! ---
    printf("\n*** ROBOT ACTIVE - STARTING MOTION ***\n\n");
    encoder_reset_total_ticks();
    pid_reset(&g_pid_heading);
    g_demo_running = true;

    while (true) {
        sleep_ms(100);
    }

    return 0;
}