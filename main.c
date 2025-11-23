#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/cyw43_arch.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

// Your driver includes
#include "drivers/motor.h"
#include "drivers/ir.h"      
#include "drivers/mqtt.h"    
#include "drivers/encoder.h" 
#include "drivers/imu.c"

// --- Pin Definitions for Buttons ---
#define BUTTON_MODE_CALIBRATE   21
#define BUTTON_MODE_START_STOP 20 

// --- ADC Pin for Calibration ---
#define PIN_IR_RIGHT_ADC_GPIO    26
#define PIN_IR_RIGHT_ADC_CH      0
#define ADC_SMOOTH_SAMPLES       12

// --- Settings ---
#define DEBOUNCE_TIME_US   150000 
#define PRINT_DELAY_MS     100    
#define CONTROL_LOOP_MS    10     
#define TEMP_PUBLISH_INTERVAL_MS 1000 
#define TELEMETRY_PUBLISH_INTERVAL_MS 200

// --- Task Priorities ---
#define MAIN_CONTROL_TASK_PRIORITY (tskIDLE_PRIORITY + 2UL)
#define NETWORK_TASK_PRIORITY      (tskIDLE_PRIORITY + 1UL)

// --- State Machine ---
typedef enum {
    STATIONARY_NOT_CALIBRATED,
    CALIBRATE_WHITE_START, 
    CALIBRATE_WHITE_END,
    CALIBRATE_BLACK_START, 
    CALIBRATED_STOPPED,    
} RobotState;

// --- Shared Global State ---
RobotState current_state = STATIONARY_NOT_CALIBRATED;
bool needs_update = true;
uint32_t last_button_press_time = 0;
uint16_t g_calib_white = 0;
uint16_t g_calib_black = 0;
bool g_is_calibrated = false;

// --- MODIFIED: 4 Scenarios ---
int g_current_scenario = 0; // 0=STOP, 1=RUN, 2=TUNE R->L, 3=TUNE L->R
float g_line_follow_power = 0.0f;

float g_target_power_left = 0.0f;
float g_target_power_right = 0.0f;
uint16_t g_sensor_R = 0;
const char *g_current_ir_state = "IDLE";
uint32_t g_last_print_time = 0;

// --- NEW: Variable to store lost time from driver ---
uint32_t g_current_lost_time_ms = 0;

int32_t g_total_ticks_left = 0;
int32_t g_total_ticks_right = 0;

static MQTT_CLIENT_DATA_T mqtt_state;

/**
 * @brief Blinks the onboard LED to indicate the current scenario.
 * State 0 = 1 blink, State 1 = 2 blinks, ..., State 3 = 4 blinks.
 */
void indicate_state_led(int scenario) {
    int blinks = scenario + 1; 
    vTaskDelay(pdMS_TO_TICKS(250));
    for (int i = 0; i < blinks; i++) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(150));
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

/**
 * @brief MODIFIED: Helper function to load tuning params for all 4 scenarios.
 */
/**
 * @brief MODIFIED: Helper function to load tuning params for all 4 scenarios.
 */
// void apply_scenario_tune(int scenario) {
//     printf("\n--- CONFIGURING SCENARIO %d ---\n", scenario);
    
//     // Default values (from your "Balanced & Fast" tune)
//     float kp = 32.0f;
//     float ki = 0.0f;
//     float kd = 0.6f;
//     float steer_clamp = 30.0f;
//     float base_scale = 1.0f;
//     float curve_slowdown = 0.7f;
//     float curve_exp = 1.3f;
//     float min_speed_frac = 0.25f;

//     // --- 
//     // --- !!! YOUR FINAL TUNED VALUES GO HERE !!! ---
//     // --- 
//     // (Values from your logs)
//     uint32_t final_timeout_R_L = 1200; // (Based on 228ms max wobble)
//     uint32_t final_timeout_L_R = 1200; // (Based on 453ms max wobble)
//     // ---
//     // ---
//     uint32_t running_timeout = 800;  // Safe value for normal running
//     uint32_t tuning_timeout = 5000;   // Safe value for tuning modes

//     switch (scenario) {
//         case 0: // STOPPED
//             g_line_follow_power = 0.0f;
//             printf("MODE: STOPPED (Scenario 0)\n");
//             kp = 15.0f; kd = 0.0f; steer_clamp = 20.0f; // Use safe/default params
//             ir_reset_robot_state(); // Reset to default (RIGHT_EDGE) and reset PID
//             break;
        
//         case 1: // "RUN" Mode - CENTERLINE TRACKING
//             g_line_follow_power = 45.0f;
//             printf("MODE: RUN (Adaptive Edge Tracking)\n");
            
//             // Use EXACT same parameters as Case 2/3 (they work!)
//             base_scale = 1.05f;
//             curve_exp = 1.1f;
            
//             // CONSERVATIVE timeouts - only switch when truly lost
//             final_timeout_R_L = running_timeout;  // 800ms before switching R->L
//             final_timeout_L_R = running_timeout;  // 800ms before switching L->R
//             // Start on RIGHT edge (works for anticlockwise startup)
//             // Will auto-switch to LEFT edge if track is clockwise
//             ir_force_state(IR_STATE_TRACK_RIGHT);
//             break;

//         case 2: // "TUNE" Mode (Right-to-Left)
//             // g_line_follow_power = 40.0f;
//             // kp = 40.0f;       // from KP_SINGLE
//             // ki = 0.0f;       // from KI_SINGLE
//             // kd = 2.0f;       // from KD_SINGLE
//             // steer_clamp = 30.0f;   // from STEER_CLAMP_TPS
//             // base_scale = 0.85f;   // from BASE_SPEED_SCALE
//             // curve_slowdown = 0.70f;  // from CURVE_SLOWDOWN
//             // curve_exp = 1.6f;    // from CURVE_EXP
//             // min_speed_frac = 0.30f;  // from MIN_SPEED_FRACTION
//             // printf("MODE: TUNE R->L | Forcing RIGHT_EDGE\n");
//             // ir_set_tuning_params(kp, ki, kd, steer_clamp, base_scale, 
//             //                      curve_slowdown, curve_exp, min_speed_frac, 
//             //                      tuning_timeout, tuning_timeout);

//             g_line_follow_power = 45.0f;
//             kd = 1.0f;
//             base_scale = 1.00f;
//             curve_exp = 1.1f;
//             final_timeout_R_L = tuning_timeout;
//             final_timeout_L_R = tuning_timeout;
//             printf("MODE: TUNE R->L | Forcing RIGHT_EDGE\n");
//             ir_force_state(IR_STATE_TRACK_RIGHT); // Force RIGHT_EDGE and reset PID
//             break;

//         case 3: // "TUNE" Mode (Left-to-Right)
//             g_line_follow_power = 45.0f;
//             kd = 1.0f;
//             base_scale = 1.00f;
//             curve_exp = 1.1f;
//             final_timeout_R_L = tuning_timeout;
//             final_timeout_L_R = tuning_timeout;
//             printf("MODE: TUNE L->R | Forcing LEFT_EDGE\n");
//             ir_force_state(IR_STATE_TRACK_LEFT); // Force LEFT_EDGE and reset PID
//             break;
//     }
//     ir_set_tuning_params(kp, ki, kd, steer_clamp, base_scale, 
//                                  curve_slowdown, curve_exp, min_speed_frac, 
//                                  final_timeout_R_L, final_timeout_L_R);
    
//     // ---
//     // THE BUG WAS LIKELY A STRAY ir_reset_robot_state(); CALL HERE.
//     // MAKE SURE IT IS GONE.
//     // ---
// }

bool g_is_running = false;
uint16_t val_white = 0;
uint16_t val_black = 0;
/**
 * @brief MASTER ISR for BUTTONS
 * MODIFIED: Cycles scenarios 0-3
 */
void gpio_interrupt_handler(uint gpio, uint32_t events) {
    static int cal_step = 0;
    static uint32_t last_time = 0;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_time < 300) return; // Debounce
    last_time = now;

    if (cal_step == 0) {
        val_white = ir_read_single_sensor_raw();
        printf("WHITE saved: %d. Place on BLACK and press button.\n", val_white);
        g_current_ir_state = "CALIBRATING WHITE";
        cal_step++;
    } else if (cal_step == 1) {
        val_black = ir_read_single_sensor_raw();
        ir_set_calibration_values(val_white, val_black);
        g_is_calibrated = true;
        printf("BLACK saved: %d. Ready! Press to RUN.\n", val_black);
        g_current_ir_state = "CALIBRATING BLACK";
        cal_step++;
    } else {
        g_is_running = !g_is_running;
        printf(g_is_running ? "RUNNING!\n" : "STOPPED.\n");
    }
}

/**
 * @brief TASK 1: Handles all high-frequency robot control logic.
 */
void main_control_task(__unused void *params) {
    vTaskDelay(pdMS_TO_TICKS(500));
    printf("Control Task Started.\n");
    
    // Default Speed (Tickets Per Second? Or %? Adjust to your motor driver)
    float base_speed = 35.0f; 
    
    float target_L = 0;
    float target_R = 0;
    uint16_t current_adc = 0;

    for (;;) {
        if (g_is_running && g_is_calibrated) {
            
            // CALL THE SIMPLIFIED PID FUNCTION
            ir_follow_line_single(base_speed, &target_L, &target_R, &current_adc);
            
            // Actuate
            motor_set_speed(MOTOR_LEFT, target_L);
            motor_set_speed(MOTOR_RIGHT, target_R);

        } else {
            // Stop motors if not running
            motor_set_speed(MOTOR_LEFT, 0);
            motor_set_speed(MOTOR_RIGHT, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_MS));
    }
}

// --- network_task (unchanged) ---
void network_task(__unused void *params) {   
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed");
        for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    } else {
        printf("CYW43 Arch Initialized (for LED & Wi-Fi)\n");
    }
    printf("Network task started. Initializing MQTT...\n");
    mqtt_client_init(&mqtt_state);
    while (!mqtt_client_try_wifi_connect(&mqtt_state, 10000)) {
        printf("WiFi connection failed, retrying in 2s...\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    mqtt_client_start_mqtt(&mqtt_state);
    uint32_t last_temp_publish_ms = 0;
    uint32_t last_telemetry_publish_ms = 0;
    for (;;) {
        mqtt_client_poll(&mqtt_state); 
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        // if (mqtt_client_is_online(&mqtt_state) && (now_ms - last_temp_publish_ms >= TEMP_PUBLISH_INTERVAL_MS)) {
        //     last_temp_publish_ms = now_ms;
        //     mqtt_client_publish_temperature(&mqtt_state);
        // }

        if (mqtt_client_is_online(&mqtt_state) && (now_ms - last_telemetry_publish_ms >= TELEMETRY_PUBLISH_INTERVAL_MS)) {
            last_telemetry_publish_ms = now_ms;
            mqtt_pub_telemetry(&mqtt_state,
                g_current_scenario,
                g_current_ir_state,
                g_target_power_left,
                g_target_power_right,
                g_total_ticks_left,
                g_total_ticks_right,
                g_current_scenario > 0 ? g_sensor_R : ir_read_single_sensor_raw(),
                g_current_lost_time_ms, 1);
            mqtt_pub_imu_raw(&mqtt_state,
                raw_accel_data[0], raw_accel_data[1], raw_accel_data[2],
                raw_mag_data[0], raw_mag_data[1], raw_mag_data[2]
            );
            mqtt_pub_imu_filtered(&mqtt_state,
                filtered_accel[0], filtered_accel[1], filtered_accel[2],
                roll_deg, pitch_deg, yaw_deg
            );
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// --- vLaunch (unchanged) ---
void vLaunch(void) {
    adc_init();
    adc_gpio_init(PIN_IR_RIGHT_ADC_GPIO);
    imu_init();
    motor_init();
    ir_init();      
    encoder_init(); 
    gpio_init(BUTTON_MODE_CALIBRATE);
    gpio_set_dir(BUTTON_MODE_CALIBRATE, GPIO_IN);
    gpio_pull_up(BUTTON_MODE_CALIBRATE);
    gpio_init(BUTTON_MODE_START_STOP);
    gpio_set_dir(BUTTON_MODE_START_STOP, GPIO_IN);
    gpio_pull_up(BUTTON_MODE_START_STOP);
    gpio_set_irq_enabled_with_callback(BUTTON_MODE_CALIBRATE, GPIO_IRQ_EDGE_FALL, true, &gpio_interrupt_handler);
    gpio_set_irq_enabled(BUTTON_MODE_START_STOP, GPIO_IRQ_EDGE_FALL, true);
    motor_set_speed(MOTOR_LEFT, 0.0f);
    motor_set_speed(MOTOR_RIGHT, 0.0f);
    printf("Mode: STATIONARY. Press GP21 to start calibration.\n");
    xTaskCreate(network_task, "NetworkTask", 2048, NULL, NETWORK_TASK_PRIORITY, NULL);
    xTaskCreate(main_control_task, "MainControlTask", 1024, NULL, MAIN_CONTROL_TASK_PRIORITY, NULL);
    printf("Starting FreeRTOS scheduler\n");
    vTaskStartScheduler();
}

// --- main (unchanged) ---
int main() {
    stdio_init_all();
    sleep_ms(3000);
    printf("Robotic Car Program Started!\n");
    printf("WIFI_SSID: %s\n", WIFI_SSID);
    printf("MQTT_SERVER: %s\n", MQTT_SERVER);
    vLaunch();
    for (;;);
    return 0;
}