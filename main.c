#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/cyw43_arch.h"
#include "FreeRTOS.h"
#include "task.h"

// --- DRIVERS ---
#include "drivers/motor.h"
#include "drivers/ir.h"
#include "drivers/mqtt.h"
#include "drivers/imu.h"
#include "led.h"
#include "barcode.h"
#include "encoder.h"
#include "ultrasonic.h" 

// --- PIN DEFINITIONS ---
#define BUTTON_MODE_GP20        20
#define BUTTON_MODE_CALIB_START 21
#define PIN_IR_RIGHT_ADC_GPIO   26
#define PIN_IR_RIGHT_ADC_CH     0

// Ultrasonic & Servo Pins
#define TRIG_PIN 17
#define ECHO_PIN 16
#define SERVO_PIN 15

#define DEBOUNCE_TIME_US   200000
#define CONTROL_LOOP_MS    10
#define TELEMETRY_PUBLISH_INTERVAL_MS 200

#define BARCODE_TASK_PRIORITY       (tskIDLE_PRIORITY + 3UL)
#define MAIN_CONTROL_TASK_PRIORITY  (tskIDLE_PRIORITY + 2UL)
#define NETWORK_TASK_PRIORITY       (tskIDLE_PRIORITY + 1UL)

typedef enum {
    STATIONARY_NOT_CALIBRATED,
    CALIBRATE_WHITE_WAIT,
    CALIBRATE_WHITE_SAMPLING,
    CALIBRATE_BLACK_WAIT,
    CALIBRATE_BLACK_SAMPLING,
    CALIBRATED_STOPPED,
} CalibrationState;

// --- Globals ---
volatile CalibrationState current_state = STATIONARY_NOT_CALIBRATED;
volatile bool g_btn_action_flag = false;
volatile uint32_t last_button_press_time = 0;

bool g_is_running = false;
uint16_t g_calib_white = 0;
uint16_t g_calib_black = 0;
bool g_is_calibrated = false;

// Drive variables
float g_target_power_left = 0.0f;
float g_target_power_right = 0.0f;
uint16_t g_sensor_R = 0;
const char *g_current_ir_state = "IDLE";
uint32_t g_current_lost_time_ms = 0;

// Obstacle Avoidance Globals
volatile int g_maneuver_state = 0; // 0 = Line Follow, 1 = Obstacle Mode
volatile int g_obs_step = 0;
volatile float g_measured_width = 0.0f;
volatile float g_obs_base_heading = 0.0f;
volatile float g_last_known_dist = 100.0f; // Made global so Network Task can see it

static MQTT_CLIENT_DATA_T mqtt_state;

// --- HELPER FUNCTIONS ---

uint16_t ir_read_single_sensor_raw_local(void) {
    taskENTER_CRITICAL();
    adc_select_input(PIN_IR_RIGHT_ADC_CH);
    uint16_t val = adc_read();
    taskEXIT_CRITICAL();
    return val;
}

void gpio_interrupt_handler(uint gpio, uint32_t events) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_button_press_time < 300) return;
    last_button_press_time = now;
    if (gpio == BUTTON_MODE_CALIB_START) g_btn_action_flag = true;
}

// --- Main Control Task ---
void main_control_task(__unused void *params) {
    vTaskDelay(pdMS_TO_TICKS(500));
    printf("Starting ADAPTIVE STABILITY Mode with ULTRASONIC SCAN.\n");

    ir_set_tuning_params(45.0f, 0.0f, 7.0f, 65.0f, 1.0f, 0.5f, 1.0f, 0.4f);
    float base_speed = 38.0f;

    int32_t delta_left = 0;
    int32_t delta_right = 0;

    // Static variables for Ultrasonic timing
    static uint32_t last_us_check_time = 0;

    for (;;) {
        // Handle Button
        if (g_btn_action_flag) {
            g_btn_action_flag = false;
            if (!g_is_calibrated) {
                if (current_state == STATIONARY_NOT_CALIBRATED) current_state = CALIBRATE_WHITE_SAMPLING;
                else if (current_state == CALIBRATE_BLACK_WAIT) current_state = CALIBRATE_BLACK_SAMPLING;
            } else {
                g_is_running = !g_is_running;
                if (!g_is_running) {
                    update_leds(50, 0, 0, 50, 0, 0);
                } else {
                    ir_reset_pid_state();
                    encoder_reset_total_ticks();
                    update_leds(0, 50, 0, 0, 50, 0);
                    // Reset Maneuver state on restart
                    g_maneuver_state = 0;
                    g_obs_step = 0;
                    g_last_known_dist = 100.0f;
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
            }
        }

        switch (current_state) {
            case STATIONARY_NOT_CALIBRATED:
                update_leds(20, 0, 0, 20, 0, 0); motor_set_speed(MOTOR_LEFT, 0); motor_set_speed(MOTOR_RIGHT, 0); break;

            case CALIBRATE_WHITE_SAMPLING: {
                uint64_t sum = 0;
                for(int i=0; i<10; i++) { sum += ir_read_single_sensor_raw_local(); update_leds(50, 50, 50, 50, 50, 50); vTaskDelay(50); update_leds(0,0,0,0,0,0); vTaskDelay(50); }
                g_calib_white = (uint16_t)(sum / 10); current_state = CALIBRATE_BLACK_WAIT; update_leds(0, 50, 0, 0, 50, 0); break; }

            case CALIBRATE_BLACK_WAIT: update_leds(0, 50, 0, 0, 50, 0); break;

            case CALIBRATE_BLACK_SAMPLING: {
                uint64_t sum = 0;
                for(int i=0; i<10; i++) { sum += ir_read_single_sensor_raw_local(); update_leds(0, 0, 50, 0, 0, 50); vTaskDelay(50); update_leds(0,0,0,0,0,0); vTaskDelay(50); }
                g_calib_black = (uint16_t)(sum / 10); ir_set_calibration_values(g_calib_white, g_calib_black);
                g_is_calibrated = true; current_state = CALIBRATED_STOPPED; update_leds(0, 50, 0, 0, 50, 0); vTaskDelay(200); break; }

            case CALIBRATED_STOPPED:
                if (g_is_running) {
                    
                    if (g_maneuver_state == 0) {
                        // --- 1. ULTRASONIC DETECTION ---
                        // Only run detection if we are in Line Follow mode (state 0)
                        
                        uint32_t now_us = to_ms_since_boot(get_absolute_time());
                        if (now_us - last_us_check_time > 100) {
                            last_us_check_time = now_us;
                            float d = get_median_distance(1);
                            if (d > 0.1f) g_last_known_dist = d;
                        }

                        // Obstacle detected in close range
                        if (g_last_known_dist < 23.0f && g_last_known_dist > 2.0f) {
                            printf("Obstacle detected at %.1f cm! Stopping and Scanning...\n", g_last_known_dist);
                            
                            // stop motors
                            motor_set_speed(MOTOR_LEFT, 0);
                            motor_set_speed(MOTOR_RIGHT, 0);
                            update_leds(50, 0, 0, 50, 0, 0); // Red LEDs

                            // perform scan (Blocking)
                            vTaskDelay(pdMS_TO_TICKS(500));
                            g_measured_width = perform_full_scan_routine(g_last_known_dist);
                            printf("Scan complete. Width: %.2f cm\n", g_measured_width);

                            // init avoidance maneuver state
                            g_obs_base_heading = imu_get_heading();
                            g_maneuver_state = 1; // 1 = obstacle mode
                            g_obs_step = 1;       // start sequence
                            g_last_known_dist = 100.0f; // reset so we don't loop immediately
                        }

                        // --- 2. LINE FOLLOWER ---
                        // Only drive if we didn't just switch to obstacle mode
                        if (g_maneuver_state == 0) {
                            update_leds(0, 50, 50, 0, 50, 50); // Cyan

                            ir_follow_line_single(base_speed,
                                                  &g_target_power_left, &g_target_power_right, &g_sensor_R,
                                                  &g_current_ir_state, &g_current_lost_time_ms);

                            encoder_update_and_get_ticks(&delta_left, &delta_right);

                            if (strncmp(g_current_ir_state, "STRT", 4) == 0) {
                                int32_t diff = delta_left - delta_right;
                                float correction = diff * 0.3f;
                                g_target_power_left -= correction;
                                g_target_power_right += correction;
                            }

                            motor_set_speed(MOTOR_LEFT, g_target_power_left);
                            motor_set_speed(MOTOR_RIGHT, g_target_power_right);
                        }
                    } 
                    else if (g_maneuver_state == 1) {
                        // --- 3. OBSTACLE STOPPED STATE ---
                        // The car has scanned and is now waiting.
                        // Since we just want to stop, we force motors to 0 here.
                        motor_set_speed(MOTOR_LEFT, 0);
                        motor_set_speed(MOTOR_RIGHT, 0);
                        update_leds(50, 0, 0, 50, 0, 0); // Remain Red
                    }

                } else {
                    // Not Running
                    ir_set_calibration_values(150, 900);
                    motor_set_speed(MOTOR_LEFT, 0); motor_set_speed(MOTOR_RIGHT, 0);
                    if ((to_ms_since_boot(get_absolute_time()) % 1000) < 500) update_leds(0,50,50,0,50,50);
                    else update_leds(0,0,0,0,0,0);
                }
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_MS));
    }
}

// --- Barcode Task ---
void barcode_task(__unused void *params) {
    // background scanner that fills globals for main_control_task
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("\n=== Barcode Task Started (%d Hz) ===\n", BARCODE_SAMPLE_HZ);

    static char prev_payload[60] = "";

    for (;;) {
        barcode_step_300hz();
        run_t r;
        while (barcode_get_next_run(&r)) {}

        const char *cmd = barcode_last_command();
        if (cmd && cmd[0] != '\0' && strcmp(cmd, "NONE") != 0) {
            if (strcmp(cmd, "LEFT") == 0 || strcmp(cmd, "RIGHT") == 0) {
                if (g_is_running) {
                    printf("!!! BARCODE DETECTED: %s -> STOPPING CAR !!!\n", cmd);
                    g_is_running = false;
                    motor_set_speed(MOTOR_LEFT, 0);
                    motor_set_speed(MOTOR_RIGHT, 0);
                    update_leds(50, 0, 0, 50, 0, 0);
                    ir_reset_pid_state();
                    g_target_power_left = 0;
                    g_target_power_right = 0;
                }
            }
            barcode_clear_result();
        }
    }
}

void network_task(__unused void *params) {
    if (cyw43_arch_init()) return;
    mqtt_client_init(&mqtt_state);
    while (!mqtt_client_try_wifi_connect(&mqtt_state, 10000)) { vTaskDelay(2000); }
    mqtt_client_start_mqtt(&mqtt_state);
    uint32_t last_pub = 0;
    for (;;) {
        mqtt_client_poll(&mqtt_state);
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (mqtt_client_is_online(&mqtt_state) && (now - last_pub >= TELEMETRY_PUBLISH_INTERVAL_MS)) {
            last_pub = now;
            int32_t ticks_l = encoder_get_total_ticks_left();
            int32_t ticks_r = encoder_get_total_ticks_right();

            // Correctly uses the global variable g_last_known_dist
            mqtt_pub_telemetry(&mqtt_state, (g_is_running ? 1 : 0), g_current_ir_state, g_target_power_left, g_target_power_right,
                               ticks_l, ticks_r, g_sensor_R, g_current_lost_time_ms, g_last_known_dist);
            mqtt_pub_imu_filtered(&mqtt_state, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }
        vTaskDelay(100);
    }
}

void vLaunch(void) {
    adc_init(); adc_gpio_init(PIN_IR_RIGHT_ADC_GPIO); adc_select_input(PIN_IR_RIGHT_ADC_CH);
    led_init(); 
    motor_init(); 
    ir_init(); 
    encoder_init();
    barcode_init();
    imu_init();

    // Initialize Ultrasonic & Servo
    ultrasonic_init(TRIG_PIN, ECHO_PIN, SERVO_PIN);

    gpio_init(BUTTON_MODE_CALIB_START); gpio_set_dir(BUTTON_MODE_CALIB_START, GPIO_IN); gpio_pull_up(BUTTON_MODE_CALIB_START);
    gpio_set_irq_enabled_with_callback(BUTTON_MODE_CALIB_START, GPIO_IRQ_EDGE_FALL, true, &gpio_interrupt_handler);

    xTaskCreate(network_task, "NetworkTask", 2048, NULL, NETWORK_TASK_PRIORITY, NULL);
    xTaskCreate(main_control_task, "MainTask", 1024, NULL, MAIN_CONTROL_TASK_PRIORITY, NULL);
    xTaskCreate(barcode_task, "BarcodeTask", 2048, NULL, BARCODE_TASK_PRIORITY, NULL);

    vTaskStartScheduler();
}

int main() { stdio_init_all(); sleep_ms(3000); vLaunch(); while(1); return 0; }