/*
 * =====================================================================================
 *
 * Filename:  main.c
 *
 * Description:  FreeRTOS robot control - main_control_task + network_task
 *               Uses pico_cyw43_arch_lwip_sys_freertos for WiFi integration
 *
 * =====================================================================================
 */

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include <stdio.h>
#include <math.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

// Project headers
#include "motor.h"
#include "encoder.h"
#include "imu.h"
#include "pid_controller.h"
#include "mqtt.h"
#include "imu.c"

// --- Configuration ---
#define CONTROL_LOOP_MS 20  // 50 Hz control loop
#define MQTT_PUBLISH_MS 200 // 5 Hz telemetry publishing

// Robot parameters
#define BASE_MOTOR_SPEED 40.0f
#define CM_PER_TICK 0.51f

// PID tuning
#define HEADING_KP 10.5f
#define HEADING_KI 0.90f
#define HEADING_KD 2.0f
#define HEADING_PID_MIN_OUT -20.0f
#define HEADING_PID_MAX_OUT 20.0f
#define HEADING_D_TAU_S 0.10f

// Task priorities
#define MAIN_CONTROL_TASK_PRIORITY (tskIDLE_PRIORITY + 3UL)
#define NETWORK_TASK_PRIORITY      (tskIDLE_PRIORITY + 1UL)

// Button pins
#define BUTTON_FORWARD_CAPTURE 20
#define BUTTON_CALIBRATE       21

// --- Global State (volatile for cross-task access) ---
volatile bool g_demo_running = false;
volatile float g_target_heading_deg = 0.0f;

// Telemetry data (written by control task, read by network task)
volatile float g_current_heading = 0.0f;
volatile float g_heading_error = 0.0f;
volatile float g_steering_output = 0.0f;
volatile float g_speed_left_cm_s = 0.0f;
volatile float g_speed_right_cm_s = 0.0f;
volatile float g_dist_left_cm = 0.0f;
volatile float g_dist_right_cm = 0.0f;
volatile float g_imu_ax = 0.0f;
volatile float g_imu_ay = 0.0f;
volatile float g_imu_az = 0.0f;
volatile float g_imu_roll = 0.0f;
volatile float g_imu_pitch = 0.0f;
volatile float g_imu_yaw = 0.0f;

// MQTT client
static MQTT_CLIENT_DATA_T mqtt_state;

// PID controller
static pid_state_t g_pid_heading;

// Heading filter
#define HEADING_FILTER_SIZE 5
static float g_heading_buffer[HEADING_FILTER_SIZE] = {0};
static int g_heading_buffer_idx = 0;
static bool g_heading_filter_init = false;

// --- Utility Functions ---

static float filter_heading(float raw_heading) {
    if (!g_heading_filter_init) {
        for (int i = 0; i < HEADING_FILTER_SIZE; i++) {
            g_heading_buffer[i] = raw_heading;
        }
        g_heading_filter_init = true;
        return raw_heading;
    }
    
    g_heading_buffer[g_heading_buffer_idx] = raw_heading;
    g_heading_buffer_idx = (g_heading_buffer_idx + 1) % HEADING_FILTER_SIZE;
    
    float sum_sin = 0.0f, sum_cos = 0.0f;
    for (int i = 0; i < HEADING_FILTER_SIZE; i++) {
        float rad = g_heading_buffer[i] * (M_PI / 180.0f);
        sum_sin += sinf(rad);
        sum_cos += cosf(rad);
    }
    
    float avg_rad = atan2f(sum_sin, sum_cos);
    float avg_deg = avg_rad * (180.0f / M_PI);
    if (avg_deg < 0.0f) avg_deg += 360.0f;
    
    return avg_deg;
}

static inline float heading_error_calc(float target, float current) {
    float error = target - current;
    while (error > 180.0f) error -= 360.0f;
    while (error <= -180.0f) error += 360.0f;
    return error;
}

// --- LED Blink Helper ---
void blink_led(int count, int delay_ms) {
    for (int i = 0; i < count; i++) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

// =============================================================================
// TASK 1: Main Control Task (High Priority, 50 Hz)
// =============================================================================
void main_control_task(__unused void *params) {
    // Wait for CYW43 to initialize (done in network_task)
    vTaskDelay(pdMS_TO_TICKS(500));
    printf("[Control] Task started\n");
    
    const float dt_s = (float)CONTROL_LOOP_MS / 1000.0f;
    
    while (1) {
        int32_t delta_ticks_left, delta_ticks_right;
        imu_pose_t current_pose;
        
        // --- 1. Read Sensors ---
        imu_update(&current_pose);
        encoder_update_and_get_ticks(&delta_ticks_left, &delta_ticks_right);
        float filtered_heading = filter_heading(current_pose.yaw_deg);
        
        // --- 2. Calculate Telemetry ---
        float speed_l = ((float)delta_ticks_left * CM_PER_TICK) / dt_s;
        float speed_r = ((float)delta_ticks_right * CM_PER_TICK) / dt_s;
        float dist_l = (float)encoder_get_total_ticks_left() * CM_PER_TICK;
        float dist_r = (float)encoder_get_total_ticks_right() * CM_PER_TICK;
        
        // Update global telemetry (no mutex needed with volatile)
        g_speed_left_cm_s = speed_l;
        g_speed_right_cm_s = speed_r;
        g_dist_left_cm = dist_l;
        g_dist_right_cm = dist_r;
        g_current_heading = filtered_heading;
        g_imu_ax = current_pose.ax;
        g_imu_ay = current_pose.ay;
        g_imu_az = current_pose.az;
        g_imu_roll = current_pose.roll_deg;
        g_imu_pitch = current_pose.pitch_deg;
        g_imu_yaw = current_pose.yaw_deg;
        
        // --- 3. PID Control ---
        float steering_bias = 0.0f;
        float h_error = 0.0f;
        
        if (g_demo_running) {
            h_error = heading_error_calc(g_target_heading_deg, filtered_heading);
            steering_bias = pid_calculate(&g_pid_heading, 0.0f, h_error, dt_s);
            
            float motor_l_speed = BASE_MOTOR_SPEED - steering_bias;
            float motor_r_speed = BASE_MOTOR_SPEED + steering_bias;
            
            // Clamp
            if (motor_l_speed < 0.0f) motor_l_speed = 0.0f;
            if (motor_l_speed > 100.0f) motor_l_speed = 100.0f;
            if (motor_r_speed < 0.0f) motor_r_speed = 0.0f;
            if (motor_r_speed > 100.0f) motor_r_speed = 100.0f;
            
            motor_set_speed(MOTOR_LEFT, motor_l_speed);
            motor_set_speed(MOTOR_RIGHT, motor_r_speed);
        } else {
            motor_set_speed(MOTOR_LEFT, 0.0f);
            motor_set_speed(MOTOR_RIGHT, 0.0f);
        }
        
        // Update error telemetry
        g_heading_error = h_error;
        g_steering_output = steering_bias;

        
        float target = g_target_heading_deg;
        float heading = g_current_heading;
        float error = g_heading_error;
        float steer = g_steering_output;
        float spdL = g_speed_left_cm_s;
        float spdR = g_speed_right_cm_s;
        float distL = g_dist_left_cm;
        float distR = g_dist_right_cm;
        float ax = g_imu_ax;
        float ay = g_imu_ay;
        float az = g_imu_az;
        float roll = g_imu_roll;
        float pitch = g_imu_pitch;
        float yaw = g_imu_yaw;
        printf("[PRINT] ax: %.1f ay: %.1f az: %.1f roll: %.1f pitch: %.1f yaw: %.1f | Tgt: %.1f Yaw: %.1f Err: %.1f Steer: %.1f | L: %.1f R: %.1f\n",
            ax, ay, az, roll, pitch, yaw, target, heading, error, steer, spdL, spdR);
        
        // --- 4. Wait for next cycle ---
        vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_MS));
    }
}

// =============================================================================
// TASK 2: Network Task (Lower Priority)
// =============================================================================
void network_task(__unused void *params) {
    // Initialize CYW43 (WiFi chip) - must be done before other tasks use it
    if (cyw43_arch_init()) {
        printf("[Network] FATAL: CYW43 init failed\n");
        for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    printf("[Network] CYW43 initialized\n");
    
    printf("[Network] Task started. Initializing MQTT...\n");
    mqtt_client_init(&mqtt_state);
    
    // Connect to WiFi (blocking, with retry)
    while (!mqtt_client_try_wifi_connect(&mqtt_state, 10000)) {
        printf("[Network] WiFi connection failed, retrying in 2s...\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    printf("[Network] WiFi connected\n");
    
    // Start MQTT
    mqtt_client_start_mqtt(&mqtt_state);
    
    uint32_t last_mqtt_publish_ms = 0;
    uint32_t last_status_print_ms = 0;
    
    for (;;) {
        // Poll MQTT stack (non-blocking)
        mqtt_client_poll(&mqtt_state);
        
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        
        // Print connection status every 10 seconds
        if (now_ms - last_status_print_ms >= 10000) {
            last_status_print_ms = now_ms;
            bool online = mqtt_client_is_online(&mqtt_state);
            printf("[Network] MQTT: %s\n", online ? "ONLINE" : "OFFLINE");
        }
        
        // Publish telemetry at 5 Hz
        if (mqtt_client_is_online(&mqtt_state) && 
            (now_ms - last_mqtt_publish_ms >= MQTT_PUBLISH_MS)) {
            last_mqtt_publish_ms = now_ms;
            
            // Copy telemetry (volatile reads are atomic on ARM Cortex-M)
            float target = g_target_heading_deg;
            float heading = g_current_heading;
            float error = g_heading_error;
            float steer = g_steering_output;
            float spdL = g_speed_left_cm_s;
            float spdR = g_speed_right_cm_s;
            float distL = g_dist_left_cm;
            float distR = g_dist_right_cm;
            float ax = g_imu_ax;
            float ay = g_imu_ay;
            float az = g_imu_az;
            float roll = g_imu_roll;
            float pitch = g_imu_pitch;
            float yaw = g_imu_yaw;
            
            // Publish main telemetry
            char buf[256];
            snprintf(buf, sizeof(buf),
                "{\"target\":%.1f,\"heading\":%.1f,\"err\":%.1f,\"steer\":%.1f,"
                "\"spdL\":%.1f,\"spdR\":%.1f,\"distL\":%.1f,\"distR\":%.1f}",
                target, heading, error, steer, spdL, spdR, distL, distR);
            mqtt_client_publish(&mqtt_state, "robot/telemetry", buf);
            
            // Publish IMU data
            mqtt_pub_imu_raw(&mqtt_state, ax, ay, az, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            mqtt_pub_imu_filtered(&mqtt_state, ax, ay, az, roll, pitch, yaw);
            
            // Print to console
            const char* status = (fabsf(error) < 2.0f) ? "STRAIGHT" :
                                (error > 0.0f) ? "LEFT" : "RIGHT";
            printf("[%s] Tgt:%.1f Yaw:%.1f Err:%.1f Steer:%.1f | L:%.1f R:%.1f\n",
                   status, target, heading, error, steer, spdL, spdR);
        }
        
        // Yield CPU every 100ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelay(pdMS_TO_TICKS(100));
}

// =============================================================================
// Initialization & Launch
// =============================================================================
void vLaunch(void) {
    printf("\n=== Robot Control System (FreeRTOS + MQTT) ===\n\n");
    
    // --- 1. Initialize Hardware (before creating tasks) ---
    printf("[Init] Motors...\n");
    motor_init();
    
    printf("[Init] Encoders...\n");
    encoder_init();
    
    printf("[Init] I2C and IMU...\n");
    if (!imu_init_i2c0_gp0_gp1()) {
        printf("[Init] FATAL: I2C init failed\n");
        while (1) tight_loop_contents();
    }
    if (!lsm303dlhc_init()) {
        printf("[Init] FATAL: IMU not found\n");
        while (1) tight_loop_contents();
    }
    
    // --- 2. Capture Forward Direction ---
    printf("\n=== FORWARD DIRECTION SETUP ===\n");
    printf("Point robot forward, place on level surface.\n");
    printf("Press GP20 button...\n");
    
    gpio_init(BUTTON_FORWARD_CAPTURE);
    gpio_set_dir(BUTTON_FORWARD_CAPTURE, GPIO_IN);
    gpio_pull_up(BUTTON_FORWARD_CAPTURE);
    
    // Wait for button press (active-low)
    while (gpio_get(BUTTON_FORWARD_CAPTURE) != 0) sleep_ms(10);
    sleep_ms(50);
    while (gpio_get(BUTTON_FORWARD_CAPTURE) == 0) sleep_ms(10);
    sleep_ms(50);
    
    printf("Capturing forward heading...\n");
    float sum_sin = 0.0f, sum_cos = 0.0f;
    for (int i = 0; i < 20; i++) {
        imu_pose_t temp;
        imu_update(&temp);
        float rad = temp.yaw_deg * (M_PI / 180.0f);
        sum_sin += sinf(rad);
        sum_cos += cosf(rad);
        sleep_ms(50);
    }
    
    float forward_rad = atan2f(sum_sin, sum_cos);
    g_target_heading_deg = forward_rad * (180.0f / M_PI);
    if (g_target_heading_deg < 0.0f) g_target_heading_deg += 360.0f;
    
    printf("Forward heading: %.1f deg\n\n", g_target_heading_deg);
    
    // --- 3. Calibration ---
    printf("=== CALIBRATION ===\n");
    imu_calibrate_bias();
    
    // Verify calibration quality
    printf("Verifying calibration...\n");
    float samples[20];
    for (int i = 0; i < 20; i++) {
        imu_pose_t test;
        imu_update(&test);
        samples[i] = test.yaw_deg;
        sleep_ms(50);
    }
    
    float mean = 0.0f;
    for (int i = 0; i < 20; i++) mean += samples[i];
    mean /= 20.0f;
    
    float variance = 0.0f;
    for (int i = 0; i < 20; i++) {
        float diff = samples[i] - mean;
        variance += diff * diff;
    }
    float std_dev = sqrtf(variance / 20.0f);
    
    printf("  Mean: %.1f deg, Noise: %.2f deg\n", mean, std_dev);
    if (std_dev > 3.0f) {
        printf("  WARNING: High noise (%.2f deg)\n", std_dev);
    } else {
        printf("  Quality: GOOD\n");
    }
    printf("\n");
    
    // --- 4. Initialize PID ---
    printf("[Init] PID controller...\n");
    pid_init(&g_pid_heading,
             HEADING_KP, HEADING_KI, HEADING_KD,
             HEADING_PID_MIN_OUT, HEADING_PID_MAX_OUT,
             HEADING_D_TAU_S);
    
    // --- 5. Stop motors initially ---
    motor_set_speed(MOTOR_LEFT, 0.0f);
    motor_set_speed(MOTOR_RIGHT, 0.0f);
    
    // --- 6. Create FreeRTOS Tasks ---
    printf("[Init] Creating tasks...\n");
    
    // Network task MUST be created first (initializes CYW43)
    // xTaskCreate(network_task, "NetworkTask", 2048, NULL, 
    //             NETWORK_TASK_PRIORITY, NULL);
    
    // Control task created second (waits for CYW43 init)
    xTaskCreate(main_control_task, "ControlTask", 1024, NULL, 
                MAIN_CONTROL_TASK_PRIORITY, NULL);
    
    printf("[Init] Complete! Starting scheduler...\n\n");
    
    // --- 7. Start Demo ---
    sleep_ms(1000);
    encoder_reset_total_ticks();
    pid_reset(&g_pid_heading);
    
    printf("*** ROBOT READY ***\n");
    printf("Target heading: %.1f deg\n", g_target_heading_deg);
    printf("Starting motion in 2 seconds...\n\n");
    
    sleep_ms(2000);
    g_demo_running = true;
    printf("*** MOTION STARTED ***\n\n");
    
    // --- 8. Start FreeRTOS Scheduler ---
    vTaskStartScheduler();
    
    // Should never reach here
    printf("[Init] FATAL: Scheduler failed\n");
    while (1) tight_loop_contents();
}

// =============================================================================
// Main Entry Point
// =============================================================================
int main(void) {
    stdio_init_all();
    sleep_ms(6000);  // Wait for USB serial
    
    printf("Robotic Car Program Started!\n");
    printf("WIFI_SSID: %s\n", WIFI_SSID);
    printf("MQTT_SERVER: %s\n", MQTT_SERVER);
    printf("\n");
    
    // vLaunch();
    imu_main();
    
    // Unreachable
    for (;;);
    return 0;
}