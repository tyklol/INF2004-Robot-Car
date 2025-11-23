#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/cyw43_arch.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

// Driver includes
#include "drivers/motor.h"
#include "drivers/ir.h"      
#include "drivers/mqtt.h"    
#include "drivers/encoder.h" 
#include "drivers/imu.h"
#include "led.h" // LED Driver

// --- Pin Definitions ---
#define BUTTON_MODE_CALIBRATE   21
#define BUTTON_MODE_START_STOP 20 
#define PIN_IR_RIGHT_ADC_GPIO    26
#define PIN_IR_RIGHT_ADC_CH      0

// --- Settings ---
#define DEBOUNCE_TIME_US   200000 
#define CONTROL_LOOP_MS    10     
#define TELEMETRY_PUBLISH_INTERVAL_MS 200
// --- Task Priorities ---
#define MAIN_CONTROL_TASK_PRIORITY (tskIDLE_PRIORITY + 2UL)
#define NETWORK_TASK_PRIORITY      (tskIDLE_PRIORITY + 1UL) 

// --- State Machine for Calibration ---
typedef enum {
    STATIONARY_NOT_CALIBRATED,
    CALIBRATE_WHITE_WAIT,      // Waiting for user to press button on white
    CALIBRATE_WHITE_SAMPLING,  // The 5-second read loop
    CALIBRATE_BLACK_WAIT,      // Waiting for user to press button on black
    CALIBRATE_BLACK_SAMPLING,  // The 5-second read loop
    CALIBRATED_STOPPED,        // Done, ready to toggle GP20
} CalibrationState;

// --- Globals ---
volatile CalibrationState current_state = STATIONARY_NOT_CALIBRATED;
volatile bool g_btn_calibrate_flag = false; 
volatile bool g_btn_start_stop_flag = false;
volatile uint32_t last_button_press_time = 0;

// Driving & Calibration Globals
bool g_is_running = false;
uint16_t g_calib_white = 0;
uint16_t g_calib_black = 0;
bool g_is_calibrated = false;

// Drive variables
float g_line_follow_power = 35.0f; // Base speed
float g_target_power_left = 0.0f;
float g_target_power_right = 0.0f;
uint16_t g_sensor_R = 0;
const char *g_current_ir_state = "IDLE";
uint32_t g_current_lost_time_ms = 0;

int32_t g_total_ticks_left = 0;
int32_t g_total_ticks_right = 0;
static MQTT_CLIENT_DATA_T mqtt_state;

int g_obs_step = 0;              // 0 = Idle, 1-7 = Steps of the maneuver
int g_obs_dir = 0;               // 1 = Right Detour, -1 = Left Detour
float g_obs_base_heading = 0.0f; // The heading of the track before we started
uint32_t g_obs_timer_start = 0;  // To track the 1-second moves

// --- ADC Helper ---
uint16_t ir_read_single_sensor_raw_local(void) {
    adc_select_input(PIN_IR_RIGHT_ADC_CH);
    return adc_read();
}
// --- IMU Helper ---
float get_angle_diff(float target, float current) {
    float diff = target - current;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

// --- Helper: Normalizes angle to 0-360 ---
float norm_angle(float angle) {
    while (angle < 0) angle += 360.0f;
    while (angle >= 360) angle -= 360.0f;
    return angle;
}

// --- Helper: Executes a Turn to a specific absolute heading ---
// Returns TRUE when the turn is complete.
bool perform_turn_step(float target_heading) {
    float current_yaw = imu_get_heading();
    float error = get_angle_diff(target_heading, current_yaw);

    // Use your Power-Boosted Tuning
    float turn_speed = error * 2.5f; 

    // Clamp Max
    if (turn_speed > 45.0f) turn_speed = 45.0f;
    if (turn_speed < -45.0f) turn_speed = -45.0f;

    // Stiction Boost (Min Power)
    if (turn_speed > 0 && turn_speed < 20.0f) turn_speed = 20.0f;
    if (turn_speed < 0 && turn_speed > -20.0f) turn_speed = -20.0f;

    motor_set_speed(MOTOR_LEFT, turn_speed);
    motor_set_speed(MOTOR_RIGHT, -turn_speed);
    update_leds(50, 0, 50, 50, 0, 50); // Purple while turning

    // Completion Check (10 degree window)
    if (fabs(error) < 10.0f) {
        motor_set_speed(MOTOR_LEFT, 0);
        motor_set_speed(MOTOR_RIGHT, 0);
        return true;
    }
    return false;
}

// --- Helper: Drives Straight on a specific heading ---
// Returns TRUE when time is up.
bool perform_drive_step(float target_heading, uint32_t duration_ms) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    // Check Timer
    if (now - g_obs_timer_start > duration_ms) {
        motor_set_speed(MOTOR_LEFT, 0);
        motor_set_speed(MOTOR_RIGHT, 0);
        return true; // Time is up
    }

    // Heading Hold (Keep straight)
    float current_yaw = imu_get_heading();
    float error = get_angle_diff(target_heading, current_yaw);
    
    // Gentle correction
    float correction = error * 1.5f; 
    
    // Base speed for the move
    float forward_speed = 35.0f; 

    motor_set_speed(MOTOR_LEFT, forward_speed + correction);
    motor_set_speed(MOTOR_RIGHT, forward_speed - correction);
    
    update_leds(50, 50, 0, 50, 50, 0); // Yellow while driving blind
    return false;
}

// --- MASTER ISR for BUTTONS ---
void gpio_interrupt_handler(uint gpio, uint32_t events) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_button_press_time < 300) return; // Debounce
    last_button_press_time = now;

    if (gpio == BUTTON_MODE_CALIBRATE) {
        g_btn_calibrate_flag = true;
    } 
    else if (gpio == BUTTON_MODE_START_STOP) {
        g_btn_start_stop_flag = true;
    }
}

// --- Main Task: Handles Calibration (5s loops) and Driving (GP20 logic) ---
void main_control_task(__unused void *params) {
    vTaskDelay(pdMS_TO_TICKS(500));
    printf("Control Task Started.\n");

    for (;;) {
        imu_get_heading();
        // --- 1. HANDLE CALIBRATION BUTTON (GP21) ---
        if (g_btn_calibrate_flag) {
            g_btn_calibrate_flag = false;
            // Advance the state machine if valid
            if (current_state == STATIONARY_NOT_CALIBRATED) {
                current_state = CALIBRATE_WHITE_SAMPLING;
            } else if (current_state == CALIBRATE_BLACK_WAIT) {
                current_state = CALIBRATE_BLACK_SAMPLING;
            } else if (current_state == CALIBRATED_STOPPED) {
                // Reset
                current_state = STATIONARY_NOT_CALIBRATED;
                g_is_calibrated = false;
                g_is_running = false;
                printf(">> Resetting Calibration.\n");
            }
        }

        // --- 2. HANDLE START/STOP BUTTON (GP20) ---
        if (g_btn_start_stop_flag) {
            g_btn_start_stop_flag = false;
            if (g_is_calibrated) {
                g_is_running = !g_is_running;
                // Reset turn flags on stop
                if (!g_is_running) {
                    g_obs_step = 0;
                }
                printf(g_is_running ? ">> STARTED (GP20)\n" : ">> STOPPED (GP20)\n");
            } else {
                printf(">> IGNORED GP20: Not Calibrated\n");
                update_leds(50, 0, 0, 50, 0, 0); // Red Flash warning
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        }

        // --- 3. STATE MACHINE / LOGIC ---
        switch (current_state) {
            case STATIONARY_NOT_CALIBRATED:
                // Red Breath or Static
                update_leds(20, 0, 0, 20, 0, 0);
                motor_set_speed(MOTOR_LEFT, 0); 
                motor_set_speed(MOTOR_RIGHT, 0);
                break;
            case CALIBRATE_WHITE_SAMPLING: {
                printf(">> SAMPLING WHITE (5 Seconds)...\n");
                uint64_t sum = 0;
                // 50 iterations * 100ms = 5000ms = 5 Seconds
                for(int i=0; i<50; i++) {
                    sum += ir_read_single_sensor_raw_local();
                    
                    // Flash White
                    update_leds(50, 50, 50, 50, 50, 50); 
                    vTaskDelay(pdMS_TO_TICKS(50)); 
                    update_leds(0, 0, 0, 0, 0, 0);       
                    vTaskDelay(pdMS_TO_TICKS(50)); 
                }
                g_calib_white = (uint16_t)(sum / 50);
                printf(">> WHITE SAVED: %u. Place on BLACK and press GP21.\n", g_calib_white);
                
                current_state = CALIBRATE_BLACK_WAIT;
                // Solid Green indicates "Wait/Next Step"
                update_leds(0, 50, 0, 0, 50, 0);
                break;
            }
            case CALIBRATE_BLACK_WAIT:
                // Just keep LEDs green while waiting for button
                update_leds(0, 50, 0, 0, 50, 0);
                break;
            case CALIBRATE_BLACK_SAMPLING: {
                printf(">> SAMPLING BLACK (5 Seconds)...\n");
                uint64_t sum = 0;
                // 50 iterations * 100ms = 5000ms = 5 Seconds
                for(int i=0; i<50; i++) {
                    sum += ir_read_single_sensor_raw_local();
                    
                    // Flash Blue
                    update_leds(0, 0, 50, 0, 0, 50); 
                    vTaskDelay(pdMS_TO_TICKS(50)); 
                    update_leds(0, 0, 0, 0, 0, 0);   
                    vTaskDelay(pdMS_TO_TICKS(50)); 
                }
                g_calib_black = (uint16_t)(sum / 50);
                printf(">> BLACK SAVED: %u.\n", g_calib_black);
                
                // Finalize
                ir_set_calibration_values(g_calib_white, g_calib_black);
                g_is_calibrated = true;
                current_state = CALIBRATED_STOPPED;
                
                printf(">> CALIBRATION DONE. Range: %d. Press GP20 to RUN.\n", g_calib_black - g_calib_white);
                update_leds(0, 50, 0, 0, 50, 0); // Green Success
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            }
            case CALIBRATED_STOPPED:
                if (g_is_running) {
                    
                    // === 1. MQTT COMMAND CHECK ===
                    if (g_barcode_received && g_obs_step == 0) {
                        
                        // Save the heading we had ON THE TRACK before turning
                        g_obs_base_heading = imu_get_heading();
                        
                        if (strcmp(g_last_barcode_cmd, "LEFT") == 0) {
                            printf("OBSTACLE: Detour Left Sequence Start\n");
                            g_obs_dir = -1; // -1 = Left
                            g_obs_step = 1; // Start the machine
                        } 
                        else if (strcmp(g_last_barcode_cmd, "RIGHT") == 0) {
                             printf("OBSTACLE: Detour Right Sequence Start\n");
                             g_obs_dir = 1; // 1 = Right
                             g_obs_step = 1;
                        }
                        g_barcode_received = false; 
                    }

                    // === 2. OBSTACLE SEQUENCE MACHINE ===
                    if (g_obs_step > 0) {
                        float target = 0;
                        
                        switch (g_obs_step) {
                            // --- STEP 1: TURN OUT (90 deg) ---
                            case 1:
                                target = norm_angle(g_obs_base_heading + (g_obs_dir * 90.0f));
                                if (perform_turn_step(target)) {
                                    vTaskDelay(pdMS_TO_TICKS(200)); // Pause
                                    g_obs_timer_start = to_ms_since_boot(get_absolute_time());
                                    g_obs_step++; // Next Step
                                }
                                break;

                            // --- STEP 2: DRIVE OUT (1 sec) ---
                            case 2:
                                target = norm_angle(g_obs_base_heading + (g_obs_dir * 90.0f));
                                if (perform_drive_step(target, 1200)) { // 1000 ms
                                    g_obs_step++;
                                }
                                break;

                            // --- STEP 3: TURN PARALLEL (Back to Base Heading) ---
                            case 3:
                                target = norm_angle(g_obs_base_heading); // Parallel to track
                                if (perform_turn_step(target)) {
                                    vTaskDelay(pdMS_TO_TICKS(200));
                                    g_obs_timer_start = to_ms_since_boot(get_absolute_time());
                                    g_obs_step++;
                                }
                                break;

                            // --- STEP 4: DRIVE PARALLEL (1 sec) ---
                            case 4:
                                target = norm_angle(g_obs_base_heading);
                                if (perform_drive_step(target, 1200)) {
                                    g_obs_step++;
                                }
                                break;

                            // --- STEP 5: TURN IN (Towards Track) ---
                            // If we went Left (-90), we need to turn Right (+90) relative to base to hit the track.
                            case 5:
                                target = norm_angle(g_obs_base_heading - (g_obs_dir * 90.0f));
                                if (perform_turn_step(target)) {
                                    vTaskDelay(pdMS_TO_TICKS(200));
                                    g_obs_timer_start = to_ms_since_boot(get_absolute_time());
                                    g_obs_step++;
                                }
                                break;

                            // --- STEP 6: DRIVE IN (1 sec) ---
                            case 6:
                                target = norm_angle(g_obs_base_heading - (g_obs_dir * 90.0f));
                                if (perform_drive_step(target, 1200)) {
                                    g_obs_step++;
                                }
                                break;

                            // --- STEP 7: ALIGN (Back to Base Heading) ---
                            case 7:
                                target = norm_angle(g_obs_base_heading); // Face forward again
                                if (perform_turn_step(target)) {
                                    printf("OBSTACLE: Sequence Complete. Resuming Line.\n");
                                    ir_reset_pid_state(); // Clear PID memory
                                    g_obs_step = 0; // Done!
                                }
                                break;
                        }

                    } else {
                        // === 3. NORMAL LINE FOLLOWING ===
                        // (This only runs if g_obs_step == 0)
                        ir_set_calibration_values(100,900);
                        ir_follow_line_single(g_line_follow_power, &g_target_power_left, &g_target_power_right, &g_sensor_R);
                        motor_set_speed(MOTOR_LEFT, g_target_power_left);
                        motor_set_speed(MOTOR_RIGHT, g_target_power_right);
                        
                        float steer = g_target_power_left - g_target_power_right;
                        handle_smart_indicators(steer);
                    }

                } else {
                    ir_set_calibration_values(150,900 );
                    // --- STOPPED LOGIC ---
                    motor_set_speed(MOTOR_LEFT, 0);
                    motor_set_speed(MOTOR_RIGHT, 0);
                    // Dim Blue = Idle/Ready
                    update_leds(0, 0, 20, 0, 0, 20);
                }
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_MS));
    }
}

// --- Network Task (Standard) ---
void network_task(__unused void *params) {   
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return;
    }
    mqtt_client_init(&mqtt_state);
    while (!mqtt_client_try_wifi_connect(&mqtt_state, 10000)) {
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    mqtt_client_start_mqtt(&mqtt_state);
    
    uint32_t last_pub = 0;
    for (;;) {
        mqtt_client_poll(&mqtt_state); 
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (mqtt_client_is_online(&mqtt_state) && (now - last_pub >= TELEMETRY_PUBLISH_INTERVAL_MS)) {
            last_pub = now;
            mqtt_pub_telemetry(&mqtt_state,
                (g_is_running ? 1 : 0), // Scenario 1=Run, 0=Stop
                g_current_ir_state,
                g_target_power_left, g_target_power_right,
                g_total_ticks_left, g_total_ticks_right,
                g_sensor_R, g_current_lost_time_ms, g_obs_dir);
            mqtt_pub_imu_raw(
                &mqtt_state, 
                (float)raw_accel_data[0], (float)raw_accel_data[1], (float)raw_accel_data[2],
                (float)raw_mag_data[0], (float)raw_mag_data[1], (float)raw_mag_data[2]);
            mqtt_pub_imu_filtered(
                &mqtt_state, filtered_accel[0], filtered_accel[1], filtered_accel[2],
                roll_deg, pitch_deg, yaw_deg
            );
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// --- Launch ---
void vLaunch(void) {
    // 1. Init Hardware
    adc_init();
    adc_gpio_init(PIN_IR_RIGHT_ADC_GPIO);
    adc_select_input(PIN_IR_RIGHT_ADC_CH);
    
    // 2. Init Drivers
    led_init(); // New LED Driver
    motor_init();
    ir_init();       
    encoder_init(); 
    imu_init();
    
    // 3. Init Buttons
    gpio_init(BUTTON_MODE_CALIBRATE); gpio_set_dir(BUTTON_MODE_CALIBRATE, GPIO_IN); gpio_pull_up(BUTTON_MODE_CALIBRATE);
    gpio_init(BUTTON_MODE_START_STOP); gpio_set_dir(BUTTON_MODE_START_STOP, GPIO_IN); gpio_pull_up(BUTTON_MODE_START_STOP);
    
    gpio_set_irq_enabled_with_callback(BUTTON_MODE_CALIBRATE, GPIO_IRQ_EDGE_FALL, true, &gpio_interrupt_handler);
    gpio_set_irq_enabled(BUTTON_MODE_START_STOP, GPIO_IRQ_EDGE_FALL, true);
    
    printf("System Ready. Press GP21 to Calibrate, GP20 to Run.\n");
    
    xTaskCreate(network_task, "NetworkTask", 2048, NULL, NETWORK_TASK_PRIORITY, NULL);
    xTaskCreate(main_control_task, "MainTask", 1024, NULL, MAIN_CONTROL_TASK_PRIORITY, NULL);
    
    vTaskStartScheduler();
}

int main() {
    stdio_init_all();
    sleep_ms(3000);
    vLaunch();
    while(1);
    return 0;
}