/*
 * =====================================================================================
 *
 * Filename:  main.c
 *
 * Description:  Main application logic for Demo 1: Straight-line motion
 * using IMU heading PID and encoder telemetry.
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
// Base speed to send to motors (e.g., 30% power)
#define BASE_MOTOR_SPEED 40.0f
// This constant is CRITICAL. You must measure your wheel circumference
// and divide by the (number of encoder ticks per revolution).
// (e.g., 20.4cm circ. / 40 ticks/rev = 0.51 cm/tick)
#define CM_PER_TICK 0.51f

// --- Heading PID Configuration ---
// These values MUST be tuned for your robot's weight and motors.
// Start with Kp and tune it first.
#define HEADING_KP 1.2f  // Proportional gain (Adjust this first!)
#define HEADING_KI 0.05f // Integral gain (Helps fix long-term drift)
#define HEADING_KD 0.1f  // Derivative gain (Helps prevent overshooting)

// Limit the steering correction (e.g., +/- 25% speed)
#define HEADING_PID_MIN_OUT -25.0f
#define HEADING_PID_MAX_OUT 25.0f
// Optional D-term smoothing (0.01s = 10ms)
#define HEADING_D_TAU_S 0.01f

// --- Global State ---
static pid_state_t g_pid_heading;
static float g_target_heading_deg = 0.0f;
static bool g_demo_running = false;

// --- Telemetry Function (MQTT Stub) ---

/**
 * @brief Publishes telemetry data.
 * (REPLACE THIS with your actual MQTT publishing code)
 */
void publish_telemetry(float speed_l_cm_s, float speed_r_cm_s,
                       float dist_l_cm, float dist_r_cm,
                       imu_pose_t *pose, float target_heading)
{
    // This just prints to the serial console.
    // In a real implementation, you would format this as JSON or
    // another format and publish to your MQTT topics.
    printf(
        "[Telemetry] Target:%.1f, Yaw:%.1f, Roll:%.1f, Pitch:%.1f, Spd_L:%.1f, Spd_R:%.1f, Dist_L:%.1f\n",
        target_heading,
        pose->yaw_deg,
        pose->roll_deg,
        pose->pitch_deg,
        speed_l_cm_s,
        speed_r_cm_s,
        dist_l_cm);

    // Example: Publishing raw vs. filtered data (as required by demo)
    // Your imu_update() already provides the filtered accel (ax_f, ay_f, az_f)
    // You would need to modify imu_update() to also pass out the *raw* accel
    // values if you want to compare them.
    // printf("[IMU] RawAX: %d, FltAX: %.2f\n", raw_ax, pose->ax);
}

/**
 * @brief Calculates the shortest angle difference between two headings.
 * Result is in range [-180, 180].
 */
static inline float heading_error(float target, float current)
{
    float error = target - current;
    while (error > 180.0f)
        error -= 360.0f;
    while (error <= -180.0f)
        error += 360.0f;
    return error;
}

/**
 * @brief Main control loop, called by a repeating timer.
 */
bool main_loop_callback(struct repeating_timer *t)
{
    static imu_pose_t current_pose;
    int32_t delta_ticks_left, delta_ticks_right;
    const float dt_s = (float)MAIN_LOOP_PERIOD_MS / 1000.0f;

    // --- 1. Read Sensors ---
    imu_update(&current_pose);
    encoder_update_and_get_ticks(&delta_ticks_left, &delta_ticks_right);

    // --- 2. Calculate State & Telemetry ---
    // Speed (cm/s)
    float speed_l_cm_s = ((float)delta_ticks_left * CM_PER_TICK) / dt_s;
    float speed_r_cm_s = ((float)delta_ticks_right * CM_PER_TICK) / dt_s;
    // Total Distance (cm)
    float dist_l_cm = (float)encoder_get_total_ticks_left() * CM_PER_TICK;
    float dist_r_cm = (float)encoder_get_total_ticks_right() * CM_PER_TICK;

    // --- 3. Run Control Logic ---
    if (g_demo_running)
    {
        // Calculate the heading error (handles 359->1 degree wrap-around)
        float error = heading_error(g_target_heading_deg, current_pose.yaw_deg);

        // Calculate steering correction.
        // We give the PID the error, and its target is 0.
        // We use -error so a positive error (turn right) causes a positive bias.
        float steering_bias = pid_calculate(&g_pid_heading, 0.0f, -error, dt_s);

        // --- 4. Set Motors ---
        // Apply differential steering bias
        // --- 4. Set Motors ---
        // Apply differential steering bias
        // --- 4. Set Motors ---
        // Apply differential steering bias
        // SWAPPED signs to correct the steering direction
        float motor_l_speed = BASE_MOTOR_SPEED - steering_bias;
        float motor_r_speed = BASE_MOTOR_SPEED + steering_bias;

        motor_set_speed(MOTOR_LEFT, motor_l_speed);
        motor_set_speed(MOTOR_RIGHT, motor_r_speed);
    }
    else
    {
        // Stop motors if demo isn't running
        motor_set_speed(MOTOR_LEFT, 0.0f);
        motor_set_speed(MOTOR_RIGHT, 0.0f);
    }

    // --- 5. Publish Telemetry ---
    // (This runs whether the demo is active or not, for live data)
    publish_telemetry(speed_l_cm_s, speed_r_cm_s, dist_l_cm, dist_r_cm,
                      &current_pose, g_target_heading_deg);

    return true; // Keep timer running
}

// --- Main Application Entry ---

int main()
{
    stdio_init_all();
    sleep_ms(2000); // Wait for serial console to connect
    printf("--- Demo 1: Straight Line IMU/Encoder Test ---\n");

    // --- 1. Initialize Hardware ---
    printf("Initializing Motors...\n");
    motor_init();

    printf("Initializing Encoders...\n");
    encoder_init();

    printf("Initializing I2C and IMU (LSM303DLHC)...\n");
    // Note: imu_init_i2c0_gp0_gp1() actually uses i2c1 on pins 2,3
    if (!imu_init_i2c0_gp0_gp1())
    {
        printf("!! FATAL: I2C init failed.\n");
        while (true)
            ;
    }
    if (!lsm303dlhc_init())
    {
        printf("!! FATAL: LSM303DLHC sensor not found.\n");
        while (true)
            ;
    }

    // --- 2. Address Warnings ---
    printf("\n--- !!! WARNINGS !!! ---\n");
    printf("1. 'imu_calibrate_bias()' in imu.c does NOT calibrate the magnetometer.\n");
    printf("   Heading will be INACCURATE until you implement full calibration!\n");
    printf("2. 'encoder.c' only counts forward ticks via polling.\n");
    printf("------------------------\n\n");

    // --- 3. Run (Flawed) Calibration ---
    printf("Place robot level and still. Running accel bias calibration...\n");
    imu_calibrate_bias(); // This only calibrates Accel X/Y
    printf("Calibration complete.\n");

    // --- 4. Initialize PID Controller ---
    pid_init(&g_pid_heading,
             HEADING_KP, HEADING_KI, HEADING_KD,
             HEADING_PID_MIN_OUT, HEADING_PID_MAX_OUT,
             HEADING_D_TAU_S);

    // --- 5. Get Target Heading ---
    printf("Getting initial heading as target...\n");
    imu_pose_t initial_pose;
    // Read a few times to let filters settle
    for (int i = 0; i < 10; i++)
    {
        imu_update(&initial_pose);
        sleep_ms(20);
    }

    g_target_heading_deg = initial_pose.yaw_deg;
    printf("... Target Heading SET to: %.2f degrees\n", g_target_heading_deg);

    // --- 6. Initialize MQTT (Stub) ---
    printf("Initializing MQTT (Stubbed with printf)...\n");
    // your_mqtt_init_function();

    // --- 7. Start Main Loop ---
    printf("Starting main control loop (50 Hz).\n");
    struct repeating_timer main_timer;
    if (!add_repeating_timer_ms(MAIN_LOOP_PERIOD_MS, main_loop_callback, NULL, &main_timer))
    {
        printf("!! FATAL: Failed to start repeating timer.\n");
        while (true)
            ;
    }

    // --- 8. Start Demo ---
    printf("\n*** ROBOT IS LIVE! Starting motion. ***\n");
    encoder_reset_total_ticks();
    pid_reset(&g_pid_heading);
    g_demo_running = true;

    // Everything else is handled by the timer ISR.
    // The main thread can sleep or handle other tasks (e.g., MQTT keep-alives).
    while (true)
    {
        // your_mqtt_loop_function();
        sleep_ms(100);
    }

    return 0;
}