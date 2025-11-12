#include <stdio.h>
#include <math.h> // --- NEW --- For angle math
#include "pico/stdlib.h"
#include "pico/time.h" // --- NEW --- For time_us_64()
#include "hardware/gpio.h"
#include "hardware/i2c.h" // --- NEW --- For IMU
#include "drivers/pid_controller.h"
#include "drivers/motor.h"   // (Assuming you have this)
#include "drivers/encoder.h" // (Assuming you have this)
#include "drivers/imu.c" // --- NEW --- Include IMU driver

#define HEADING_KP 1.0f
#define HEADING_KI 0.0f
#define HEADING_KD 0.3f
#define HEADING_PID_MIN_OUT -20.0f
#define HEADING_PID_MAX_OUT 20.0f
#define HEADING_D_TAU_S 0.0f
#define BUTTON_FORWARD_CAPTURE 20

// --- NEW: Control Loop Definitions ---
#define BASE_SPEED 40.0f         // Base motor speed (0-100)
#define CALIBRATION_TIME_MS 5000 // 5 seconds
#define LOOP_RATE_MS 20          // Run control loop every 20ms (50Hz)
#define LOOP_RATE_S 0.02f

static pid_state_t g_pid_heading;

// --- Your existing main function, now completed ---
int main(void) {
    stdio_init_all();
    sleep_ms(6000); // Wait for USB serial

    printf("Robotic Car Program Started!\n");

    printf("[Init] Motors...\n");
    motor_init();
    printf("[Init] Encoders...\n");
    encoder_init();

    // --- NEW: Initialize IMU ---
    printf("[Init] IMU...\n");
    imu_init(); // Replaced your imu_main() with this explicit init

    gpio_init(BUTTON_FORWARD_CAPTURE);
    gpio_set_dir(BUTTON_FORWARD_CAPTURE, GPIO_IN);
    gpio_pull_up(BUTTON_FORWARD_CAPTURE);

    printf("\nPress button to calibrate forward heading...\n");
    // Wait for button press (active-low)
    while (gpio_get(BUTTON_FORWARD_CAPTURE) != 0)
        sleep_ms(10);
    sleep_ms(50);
    while (gpio_get(BUTTON_FORWARD_CAPTURE) == 0)
        sleep_ms(10);
    sleep_ms(50);

    // --- NEW: Calibrate Target Yaw ---
    printf("[Calibrating] Determining forward heading for %d seconds...\n", CALIBRATION_TIME_MS / 1000);
    float yaw_sum = 0.0f;
    int yaw_count = 0;
    uint32_t cal_start_time = time_us_32();

    while ((time_us_32() - cal_start_time) < (CALIBRATION_TIME_MS * 1000))
    {
        yaw_sum += get_filtered_heading(filtered_accel, filtered_mag);
        yaw_count++;
        sleep_ms(LOOP_RATE_MS); // Read at the same rate as the control loop
    }

    float target_yaw = yaw_sum / (float)yaw_count;
    printf("[Calibrating] Complete. Target Yaw: %.2f degrees\n", target_yaw);
    // --- End Calibration ---

    printf("[Init] PID controller...\n");
    pid_init(&g_pid_heading,
             HEADING_KP, HEADING_KI, HEADING_KD,
             HEADING_PID_MIN_OUT, HEADING_PID_MAX_OUT,
             HEADING_D_TAU_S);

    motor_set_speed(MOTOR_LEFT, 0.0f);
    motor_set_speed(MOTOR_RIGHT, 0.0f);

    sleep_ms(1000); // Pause before starting
    printf("Starting main control loop!\n\n");
    printf("Target Yaw | Current Yaw | Error | PID Out | Left Spd | Right Spd\n");
    printf("----------------------------------------------------------\n");

    static uint32_t last_print_time = 0;
    while (1) {
        // --- NEW: Fixed-rate loop timing ---
        uint64_t loop_start_us = time_us_64();

        // --- 1. Get Sensor Feedback ---
        int32_t delta_ticks_left, delta_ticks_right;
        encoder_update_and_get_ticks(&delta_ticks_left, &delta_ticks_right);
        // (You can use delta_ticks_left/right for speed PID later)

        float current_yaw = get_filtered_heading(filtered_accel, filtered_mag);

        // --- 2. Calculate PID Correction ---
        // We use LOOP_RATE_S as a constant dt
        float correction_power = pid_calculate(&g_pid_heading, target_yaw, current_yaw, LOOP_RATE_S);

        // --- 3. Actuate Motors ---
        float left_speed = BASE_SPEED + correction_power;
        float right_speed = BASE_SPEED - correction_power;

        // Clamp final motor speeds to valid range (e.g., 0-100)
        left_speed = clampf(left_speed, 0.0f, 100.0f);
        right_speed = clampf(right_speed, 0.0f, 100.0f);

        motor_set_speed(MOTOR_LEFT, left_speed);
        motor_set_speed(MOTOR_RIGHT, right_speed);

        // --- 4. Print Telemetry (for tuning) ---
        float error = target_yaw - current_yaw;
        // Fix error wrap-around for printing
        if (error > 180.0f)
            error -= 360.0f;
        else if (error < -180.0f)
            error += 360.0f;

        // Add code to print every second instead of every loop
        uint32_t now_ms = time_us_64() / 1000;
        if (now_ms - last_print_time >= 1000) {
            last_print_time = now_ms;
            printf("%.1f | %.1f | %5.1f | %7.1f | %8.1f | %8.1f\r\n",
                   target_yaw, current_yaw, error,
                   correction_power, left_speed, right_speed);
        }

        // --- 5. Enforce Loop Rate ---
        uint64_t loop_end_us = time_us_64();
        int64_t sleep_us = (LOOP_RATE_MS * 1000) - (loop_end_us - loop_start_us);
        if (sleep_us > 0)
            sleep_ms(sleep_us / 1000);
    }

    return 0;
}