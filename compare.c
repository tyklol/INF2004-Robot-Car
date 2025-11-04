#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/cyw43_arch.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h" // --- DEMO 1 MOD --- Added for mutex

// Your driver includes
#include "drivers/motor.h"
#include "drivers/ir.h"      // Kept for init, but not used in control loop
#include "drivers/mqtt.h"    // Kept for network task
#include "drivers/encoder.h" // For polling total ticks
#include "drivers/imu.h"
#include "drivers/pid_controller.h" // We will use this extensively


// --- Pin Definitions for Buttons ---
#define BUTTON_SET_YAW 21         // GP21 (Set current heading as target)
#define BUTTON_MODE_START_STOP 20 // GP20 (Scenario Toggle)

// --- ADC Pin for Calibration ---
// --- DEMO 1 MOD --- Not used for Demo 1, but kept for hardware init
#define PIN_IR_RIGHT_ADC_GPIO 26 // GP26 = ADC0
#define PIN_IR_RIGHT_ADC_CH 0
#define ADC_SMOOTH_SAMPLES 12

// --- Settings ---
#define DEBOUNCE_TIME_US 150000         // 150 milliseconds
#define PRINT_DELAY_MS 100              // Print values every 100ms
#define CONTROL_LOOP_MS 10              // Run the *entire* control loop every 10ms (100Hz)
#define TEMP_PUBLISH_INTERVAL_MS 1000   // Publish temp every 1 second
#define MQTT_PUBLISH_INTERVAL_MS 200    // --- DEMO 1 MOD --- Publish telemetry every 200ms (5Hz)

// --- ROBOT TUNING PARAMETERS (!!! TUNE THESE FOR DEMO 1 !!!) ---
#define DEMO1_BASE_SPEED_TPS 40.0f      // Target speed in ticks-per-second
#define DEMO1_TRIM_LIMIT_TPS 20.0f      // Max yaw correction (in ticks-per-second)
#define DEMO1_MOTOR_MIN_POWER -80.0f    // Min motor power (leave headroom)
#define DEMO1_MOTOR_MAX_POWER 80.0f     // Max motor power (leave headroom)

// --- PID GAINS (!!! TUNE THESE !!!) ---
// Heading PID: Corrects yaw error, outputs a 'trim' in TPS
#define DEMO1_HEADING_KP 1.2f
#define DEMO1_HEADING_KI 0.05f
#define DEMO1_HEADING_KD 0.15f
#define DEMO1_HEADING_TAU 0.02f         // D-term filter time constant (s)

// Wheel Speed PID: Corrects speed error, outputs motor power
#define DEMO1_WHEEL_KP 2.0f             // Proportional (P)
#define DEMO1_WHEEL_KI 1.0f             // Integral (I) - helps overcome friction
#define DEMO1_WHEEL_KD 0.01f            // Derivative (D) - smooths response
#define DEMO1_WHEEL_TAU 0.0f            // No D-term filter for wheels

// --- Task Priorities ---
#define MAIN_CONTROL_TASK_PRIORITY (tskIDLE_PRIORITY + 3UL)
#define IMU_TASK_PRIORITY (tskIDLE_PRIORITY + 2UL)
#define NETWORK_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)

// --- State Machine (--- DEMO 1 MOD --- Simplified for Demo 1) ---
typedef enum {
    STOPPED_NO_YAW_TARGET, // Waiting for user to set yaw
    STOPPED_YAW_SET,       // Yaw is set, ready to drive
    DRIVING_STRAIGHT       // Driving
} RobotState;


// --- IMU Task Definitions (from File 1) ---
#define IMU_LOOP_HZ 20 // --- DEMO 1 MOD --- Renamed from LOOP_HZ
#define DT_SEC (1.0f / IMU_LOOP_HZ)


// ---------- Helpers (from File 1) ----------
static float heading_err_deg(float target, float current)
{
    float e = target - current;
    while (e > 180.f)
        e -= 360.f;
    while (e < -180.f)
        e += 360.f;
    return e;
}

// --- Shared Global State ---
RobotState current_state = STOPPED_NO_YAW_TARGET;
bool needs_update = true;
uint32_t last_button_press_time = 0;

// --- DEMO 1 MOD --- Global state for PID controllers and telemetry
pid_state_t g_pid_heading;
pid_state_t g_pid_wheel_left;
pid_state_t g_pid_wheel_right;

float g_yaw_target_deg = 0.0f;
float g_target_power_left = 0.0f;  // Current power sent to left motor
float g_target_power_right = 0.0f; // Current power sent to right motor
float g_current_speed_left_tps = 0.0f;  // Measured speed
float g_current_speed_right_tps = 0.0f; // Measured speed

int32_t g_total_ticks_left = 0;
int32_t g_total_ticks_right = 0;
uint32_t g_last_print_time = 0;

// --- DEMO 1 MOD --- IMU data shared with a mutex
static imu_pose_t g_latest_pose;
static SemaphoreHandle_t g_pose_mutex;

// --- Global State (from File 1, needed for other tasks) ---
static MQTT_CLIENT_DATA_T mqtt_state;


// --- LED Blink (from File 2) ---
void blink_led(int count, int delay_ms) {
    for (int i = 0; i < count; i++) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

/**
 * @brief MASTER ISR for BUTTONS ONLY
 * --- DEMO 1 MOD --- Simplified state flow for Demo 1 ---
 */
void gpio_interrupt_handler(uint gpio, uint32_t events) {
    if (gpio == BUTTON_SET_YAW || gpio == BUTTON_MODE_START_STOP) {
        uint32_t now = time_us_32();
        if (now - last_button_press_time < DEBOUNCE_TIME_US) {
            return;
        }
        last_button_press_time = now;

        if (gpio == BUTTON_SET_YAW) {
            // --- Set current yaw as the target ---
            if (xSemaphoreTakeFromISR(g_pose_mutex, NULL) == pdTRUE) {
                g_yaw_target_deg = g_latest_pose.yaw_deg;
                xSemaphoreGiveFromISR(g_pose_mutex, NULL);
            }
            
            if (current_state == STOPPED_NO_YAW_TARGET) {
                current_state = STOPPED_YAW_SET;
            }
            // If already driving or stopped, just reset the target
            needs_update = true;

        } else if (gpio == BUTTON_MODE_START_STOP) {
            if (current_state == STOPPED_YAW_SET) {
                current_state = DRIVING_STRAIGHT;
            } else if (current_state == DRIVING_STRAIGHT) {
                current_state = STOPPED_YAW_SET;
            }
            // If yaw is not set, this button does nothing
            needs_update = true;
        }
    }
}

/**
 * @brief TASK 1: Handles all high-frequency robot control logic.
 * --- DEMO 1 MOD --- This task now runs the full stabilization PID loop ---
 */
void main_control_task(__unused void *params) {
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for IMU to be ready
    printf("Main control task started (Demo 1: Straight Line).\n");

    absolute_time_t last_loop_time = get_absolute_time();
    float dt_s = (float)CONTROL_LOOP_MS / 1000.0f; // Initial dt

    for (;;) {
        // --- 1. Calculate precise dt ---
        absolute_time_t now = get_absolute_time();
        dt_s = (float)absolute_time_diff_us(last_loop_time, now) / 1000000.0f;
        last_loop_time = now;
        if (dt_s <= 0.0f) { dt_s = (float)CONTROL_LOOP_MS / 1000.0f; } // Safety fallback

        // --- 2. Get All Sensor Data ---
        
        // Get IMU pose
        imu_pose_t local_pose;
        if (xSemaphoreTake(g_pose_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            local_pose = g_latest_pose;
            xSemaphoreGive(g_pose_mutex);
        } else {
            // Failed to get mutex, skip this control loop
            printf("WARN: main_control_task failed to get pose mutex.\n");
            vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_MS));
            continue;
        }

        // Get Encoder deltas
        int32_t delta_ticks_left, delta_ticks_right;
        encoder_update_and_get_ticks(&delta_ticks_left, &delta_ticks_right);

        // Update total ticks (for telemetry)
        g_total_ticks_left = encoder_get_total_ticks_left();
        g_total_ticks_right = encoder_get_total_ticks_right();

        // Calculate current speeds in Ticks-Per-Second
        g_current_speed_left_tps = (float)delta_ticks_left / dt_s;
        g_current_speed_right_tps = (float)delta_ticks_right / dt_s;


        // --- 3. Handle State Change Requests ---
        if (needs_update) {
            needs_update = false;
            
            // Reset motor power
            g_target_power_left = 0.0f;
            g_target_power_right = 0.0f;
            motor_set_speed(MOTOR_LEFT, 0.0f);
            motor_set_speed(MOTOR_RIGHT, 0.0f);

            // --- DEMO 1 MOD --- State logic for Demo 1
            switch (current_state) {
                case STOPPED_NO_YAW_TARGET:
                    printf("Mode: STOPPED. Press GP21 to set target yaw.\n");
                    blink_led(1, 200);
                    break;
                case STOPPED_YAW_SET:
                    printf("Mode: STOPPED. Yaw target: %.1f. Press GP20 to start.\n", g_yaw_target_deg);
                    // Init all 3 PIDs
                    pid_init(&g_pid_heading, DEMO1_HEADING_KP, DEMO1_HEADING_KI, DEMO1_HEADING_KD,
                             -DEMO1_TRIM_LIMIT_TPS, DEMO1_TRIM_LIMIT_TPS, DEMO1_HEADING_TAU);
                    pid_init(&g_pid_wheel_left, DEMO1_WHEEL_KP, DEMO1_WHEEL_KI, DEMO1_WHEEL_KD,
                             DEMO1_MOTOR_MIN_POWER, DEMO1_MOTOR_MAX_POWER, DEMO1_WHEEL_TAU);
                    pid_init(&g_pid_wheel_right, DEMO1_WHEEL_KP, DEMO1_WHEEL_KI, DEMO1_WHEEL_KD,
                             DEMO1_MOTOR_MIN_POWER, DEMO1_MOTOR_MAX_POWER, DEMO1_WHEEL_TAU);
                    blink_led(2, 200);
                    break;
                case DRIVING_STRAIGHT:
                    printf("Mode: DRIVING STRAIGHT! Base Speed: %.1f tps\n", DEMO1_BASE_SPEED_TPS);
                    // Reset encoders and PIDs for a fresh start
                    encoder_reset_total_ticks();
                    g_total_ticks_left = 0;
                    g_total_ticks_right = 0;
                    pid_reset(&g_pid_heading);
                    pid_reset(&g_pid_wheel_left);
                    pid_reset(&g_pid_wheel_right);
                    blink_led(3, 100);
                    break;
            }
        }

        // --- 4. Run Control Logic ---
        if (current_state == DRIVING_STRAIGHT) {
            // --- Heading PID ---
            // Calculate error (target = 0, current = -error)
            float yaw_error = heading_err_deg(g_yaw_target_deg, local_pose.yaw_deg);
            float trim_tps = pid_calculate(&g_pid_heading, 0.0f, -yaw_error, dt_s);

            // --- Wheel Speed Targets ---
            float target_speed_L = DEMO1_BASE_SPEED_TPS + trim_tps;
            float target_speed_R = DEMO1_BASE_SPEED_TPS - trim_tps;

            // --- Wheel Speed PIDs ---
            g_target_power_left = pid_calculate(&g_pid_wheel_left, target_speed_L, g_current_speed_left_tps, dt_s);
            g_target_power_right = pid_calculate(&g_pid_wheel_right, target_speed_R, g_current_speed_right_tps, dt_s);

        } else {
            // State is STOPPED
            g_target_power_left = 0.0f;
            g_target_power_right = 0.0f;
            g_current_speed_left_tps = 0.0f;
            g_current_speed_right_tps = 0.0f;
        }

        // --- 5. Actuate Motors ---
        motor_set_speed(MOTOR_LEFT, g_target_power_left);
        motor_set_speed(MOTOR_RIGHT, g_target_power_right);


        // --- 6. Print Debugging Information ---
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - g_last_print_time > PRINT_DELAY_MS) {
            g_last_print_time = now_ms;
            if (current_state == DRIVING_STRAIGHT) {
                printf("Yaw(Tgt/Cur):%5.1f/%5.1f | Spd(L/R):%5.1f/%5.1f | Pwr(L/R):%5.1f/%5.1f | Ticks(L/R):%ld/%ld\n",
                       g_yaw_target_deg, local_pose.yaw_deg,
                       g_current_speed_left_tps, g_current_speed_right_tps,
                       g_target_power_left, g_target_power_right,
                       g_total_ticks_left, g_total_ticks_right);
            } else {
                 printf("STOPPED | Yaw(Tgt/Cur):%5.1f/%5.1f | Press GP20 to start.\n",
                       g_yaw_target_deg, local_pose.yaw_deg);
            }
        }

        // Yield to other tasks
        vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_MS));
    }
}


/**
 * @brief TASK 2: Handles all IMU data processing.
 * --- DEMO 1 MOD --- This task ONLY reads IMU data and updates the global pose.
 */
void imu_task(__unused void *params)
{
    // Wait for CYW43 and IMU to be init'd by other tasks
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("IMU task started.\n");

    imu_pose_t local_pose; // Local struct to hold new data

    while (true)
    {
        // --- 1. Update IMU readings ---
        imu_update(&local_pose);

        // --- 2. Update shared global w/ mutex ---
        if (xSemaphoreTake(g_pose_mutex, portMAX_DELAY) == pdTRUE)
        {
            g_latest_pose = local_pose;
            xSemaphoreGive(g_pose_mutex);
        }

        // --- 3. Debug output (optional) ---
        // (The main_control_task printf is more useful)
        // printf("[IMU] yaw=%.1f | roll=%.1f pitch=%.1f\r\n",
        //        local_pose.yaw_deg, local_pose.roll_deg, local_pose.pitch_deg);

        // --- 4. Delay ---
        vTaskDelay(pdMS_TO_TICKS(1000 / IMU_LOOP_HZ));
    }
}

/**
 * @brief TASK 3: Handles all Wi-Fi and MQTT communication.
 * --- DEMO 1 MOD --- Moved IMU publish logic here from old imu_task.
 */
void network_task(__unused void *params)
{
    if (cyw43_arch_init())
    {
        printf("Wi-Fi init failed");
        for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    else
    {
        printf("CYW43 Arch Initialized (for LED & Wi-Fi)\n");
    }

    printf("Network task started. Initializing MQTT...\n");
    mqtt_client_init(&mqtt_state);

    while (!mqtt_client_try_wifi_connect(&mqtt_state, 10000))
    {
        printf("WiFi connection failed, retrying in 2s...\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    mqtt_client_start_mqtt(&mqtt_state);
    uint32_t last_temp_publish_ms = to_ms_since_boot(get_absolute_time());
    uint32_t last_telemetry_publish_ms = to_ms_since_boot(get_absolute_time()); // --- DEMO 1 MOD ---

    for (;;)
    {
        mqtt_client_poll(&mqtt_state);
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());

        if (mqtt_client_is_online(&mqtt_state))
        {
            // --- Publish core telemetry (IMU, speed, etc.) ---
            if (now_ms - last_telemetry_publish_ms >= MQTT_PUBLISH_INTERVAL_MS)
            {
                last_telemetry_publish_ms = now_ms;
                
                // Get a safe copy of the IMU data
                imu_pose_t local_pose;
                if (xSemaphoreTake(g_pose_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    local_pose = g_latest_pose;
                    xSemaphoreGive(g_pose_mutex);

                    // Publish IMU data (as in original imu_task)
                    // Publish IMU data (as in original imu_task)
                    mqtt_pub_imu_raw(
                        &mqtt_state, local_pose.ax, local_pose.ay, local_pose.az,
                        0.0f, 0.0f, 0.0f, // Pass 0 for gyro (gx, gy, gz) - LSM303DLHC has no gyro
                        local_pose.mx, local_pose.my, local_pose.mz);
                    mqtt_pub_imu_filtered(
                        &mqtt_state, local_pose.ax, local_pose.ay, local_pose.az,
                        local_pose.roll_deg, local_pose.pitch_deg, local_pose.yaw_deg);
                } else {
                    printf("WARN: network_task failed to get pose mutex.\n");
                }
                
                // --- DEMO 1 MOD --- Publish speed, distance, and power
                // (Assuming your mqtt.c has functions for these topics)
                
                // mqtt_pub_speeds(&mqtt_state, g_current_speed_left_tps, g_current_speed_right_tps);
                // mqtt_pub_distance(&mqtt_state, (float)g_total_ticks_left, (float)g_total_ticks_right);
                // mqtt_pub_power(&mqtt_state, g_target_power_left, g_target_power_right);
                // mqtt_pub_pid_outputs(&mqtt_state, g_pid_heading.integral, g_pid_wheel_left.integral, g_pid_wheel_right.integral);
            }

            // --- Publish Pico temperature (less frequently) ---
            if (now_ms - last_temp_publish_ms >= TEMP_PUBLISH_INTERVAL_MS)
            {
                last_temp_publish_ms = now_ms;
                mqtt_client_publish_temperature(&mqtt_state);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Poll MQTT client 10x/sec
    }
}

/**
 * @brief The "launcher" function.
 */
void vLaunch(void)
{
    // --- 1. Initialize Hardware ---
    adc_init();
    adc_gpio_init(PIN_IR_RIGHT_ADC_GPIO); // GP26

    motor_init();
    ir_init();      // Init IR sensors (even if not used for this demo)
    encoder_init();

    // --- I2C + IMU init ---
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("\r\n=== DEMO 1: Straight Line Stabilization ===\r\n");

    imu_init_i2c0_gp0_gp1();
    if (!lsm303dlhc_init()) // --- DEMO 1 MOD --- Assuming lsm303, use your IMU init
    {
        printf("IMU init failed (WHO_AM_I check failed)\r\n");
        while (1) { blink_led(5, 100); sleep_ms(1000); }
    }
    printf("[IMU] Bias calibration... keep still.\r\n");
    imu_calibrate_bias();
    imu_set_accel_ema_alpha(0.12f);
    // --- End of IMU init ---

    // --- 2. Create Mutex ---
    g_pose_mutex = xSemaphoreCreateMutex(); // --- DEMO 1 MOD ---
    if (g_pose_mutex == NULL) {
        printf("FATAL: Failed to create mutex.\n");
        while(1) { blink_led(10, 50); sleep_ms(1000); }
    }

    // --- 3. Setup Buttons ---
    gpio_init(BUTTON_SET_YAW);
    gpio_set_dir(BUTTON_SET_YAW, GPIO_IN);
    gpio_pull_up(BUTTON_SET_YAW);
    gpio_init(BUTTON_MODE_START_STOP);
    gpio_set_dir(BUTTON_MODE_START_STOP, GPIO_IN);
    gpio_pull_up(BUTTON_MODE_START_STOP);

    // --- 4. Setup Interrupts (BUTTONS ONLY) ---
    gpio_set_irq_enabled_with_callback(BUTTON_SET_YAW, GPIO_IRQ_EDGE_FALL, true, &gpio_interrupt_handler);
    gpio_set_irq_enabled(BUTTON_MODE_START_STOP, GPIO_IRQ_EDGE_FALL, true);

    // --- 5. Stop motors initially ---
    motor_set_speed(MOTOR_LEFT, 0.0f);
    motor_set_speed(MOTOR_RIGHT, 0.0f);
    printf("Mode: STOPPED. Press GP21 to set yaw target.\n");

    // --- 6. Create Tasks ---
    xTaskCreate(network_task, "NetworkTask", 2048, NULL, NETWORK_TASK_PRIORITY, NULL);
    xTaskCreate(main_control_task, "MainControlTask", 2048, NULL, MAIN_CONTROL_TASK_PRIORITY, NULL); // --- DEMO 1 MOD --- Increased stack
    xTaskCreate(imu_task, "IMUTask", 1024, NULL, IMU_TASK_PRIORITY, NULL);

    // --- 7. Start the Scheduler ---
    printf("Starting FreeRTOS scheduler\n");
    vTaskStartScheduler();
}

/**
 * @brief The main() entry point.
 */
int main()
{
    stdio_init_all();
    sleep_ms(3000); // Wait for serial monitor to connect
    printf("Robotic Car Program Started!\n");
    printf("WIFI_SSID: %s\n", WIFI_SSID);
    printf("MQTT_SERVER: %s\n", MQTT_SERVER);

    vLaunch();

    for (;;)
        ; // Unreachable
    return 0;
}