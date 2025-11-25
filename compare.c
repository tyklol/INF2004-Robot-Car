#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/cyw43_arch.h"

// freertos includes
#include "FreeRTOS.h"
#include "task.h"

// driver includes
#include "drivers/motor.h"
#include "drivers/ir.h"      
#include "drivers/mqtt.h"    
#include "drivers/encoder.h" 
#include "drivers/ultrasonic.h" 
#include "led.h" 
#include "barcode.h" 
#include "drivers/imu.h"

// --- pin definitions ---
#define BUTTON_MODE_CALIBRATE   21
#define BUTTON_MODE_START_STOP  20 
#define PIN_IR_RIGHT_ADC_GPIO   26
#define PIN_IR_RIGHT_ADC_CH     0

// --- ultrasonic pins ---
#define TRIG_PIN 17
#define ECHO_PIN 16 
#define SERVO_PIN 15

// --- settings ---
#define DEBOUNCE_TIME_US   200000 
#define CONTROL_LOOP_MS    10     
#define TELEMETRY_PUBLISH_INTERVAL_MS 200

// --- task priorities ---
#define BARCODE_TASK_PRIORITY       (tskIDLE_PRIORITY + 3UL) // highest priority for timing
#define MAIN_CONTROL_TASK_PRIORITY  (tskIDLE_PRIORITY + 2UL)
#define NETWORK_TASK_PRIORITY       (tskIDLE_PRIORITY + 1UL) 

// --- state machine for calibration ---
typedef enum {
    STATIONARY_NOT_CALIBRATED,
    CALIBRATE_WHITE_WAIT,      
    CALIBRATE_WHITE_SAMPLING,  
    CALIBRATE_BLACK_WAIT,      
    CALIBRATE_BLACK_SAMPLING,  
    CALIBRATED_STOPPED,        
} CalibrationState;

// --- globals ---
volatile CalibrationState current_state = STATIONARY_NOT_CALIBRATED;
volatile bool g_btn_calibrate_flag = false; 
volatile bool g_btn_start_stop_flag = false;
volatile uint32_t last_button_press_time = 0;

// driving & calibration globals
bool g_is_running = false;
uint16_t g_calib_white = 0;
uint16_t g_calib_black = 0;
bool g_is_calibrated = false;

// drive variables
float g_line_follow_power = 35.0f; 
float g_target_power_left = 0.0f;
float g_target_power_right = 0.0f;
uint16_t g_sensor_R = 0;
const char *g_current_ir_state = "IDLE";
uint32_t g_current_lost_time_ms = 0;

// ultrasonic telemetry
float g_measured_width = 0.0f;

// encoder globals
int32_t g_total_ticks_left = 0;
int32_t g_total_ticks_right = 0;
static MQTT_CLIENT_DATA_T mqtt_state;

// imu / obstacle avoidance globals
int g_maneuver_state = 0;        // 0 = idle, 1 = obstacle detour, 2 = intersection turn
int g_obs_step = 0;              // 1-7 steps for rectangular avoidance
int g_obs_dir = 0;               // 1 = right, -1 = left
float g_obs_base_heading = 0.0f; // reference heading
uint32_t g_obs_timer_start = 0;  // timer for blind moves

// --- adc helper (thread safe) ---
uint16_t ir_read_single_sensor_raw_local(void) {
    taskENTER_CRITICAL();
    adc_select_input(PIN_IR_RIGHT_ADC_CH);
    uint16_t val = adc_read();
    taskEXIT_CRITICAL();
    return val;
}

// --- imu helper ---
float get_angle_diff(float target, float current) {
    float diff = target - current;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

// --- helper: normalizes angle to 0-360 ---
float norm_angle(float angle) {
    while (angle < 0) angle += 360.0f;
    while (angle >= 360) angle -= 360.0f;
    return angle;
}

// --- helper: execute turn to heading ---
bool perform_turn_step(float target_heading) {
    float current_yaw = imu_get_heading();
    float error = get_angle_diff(target_heading, current_yaw);

    // power boosted p-controller for turning
    float turn_speed = error * 2.8f; 

    // clamp max speed
    if (turn_speed > 45.0f) turn_speed = 45.0f;
    if (turn_speed < -45.0f) turn_speed = -45.0f;

    // stiction boost (minimum power to move)
    if (turn_speed > 0 && turn_speed < 22.0f) turn_speed = 22.0f;
    if (turn_speed < 0 && turn_speed > -22.0f) turn_speed = -22.0f;

    motor_set_speed(MOTOR_LEFT, turn_speed);
    motor_set_speed(MOTOR_RIGHT, -turn_speed);
    update_leds(50, 0, 50, 50, 0, 50); // purple indicator

    // completion window
    if (fabs(error) < 8.0f) {
        motor_set_speed(MOTOR_LEFT, 0);
        motor_set_speed(MOTOR_RIGHT, 0);
        return true;
    }
    return false;
}

// --- helper: drive straight with heading hold ---
bool perform_drive_step(float target_heading, uint32_t duration_ms) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    if (now - g_obs_timer_start > duration_ms) {
        motor_set_speed(MOTOR_LEFT, 0);
        motor_set_speed(MOTOR_RIGHT, 0);
        return true; 
    }

    float current_yaw = imu_get_heading();
    float error = get_angle_diff(target_heading, current_yaw);
    
    // gyro correction
    float correction = error * 1.5f; 
    float forward_speed = 35.0f; 

    motor_set_speed(MOTOR_LEFT, forward_speed + correction);
    motor_set_speed(MOTOR_RIGHT, forward_speed - correction);
    
    update_leds(50, 50, 0, 50, 50, 0); // yellow indicator
    return false;
}

// --- master isr for buttons ---
void gpio_interrupt_handler(uint gpio, uint32_t events) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_button_press_time < 300) return; // debounce
    last_button_press_time = now;

    if (gpio == BUTTON_MODE_CALIBRATE) {
        g_btn_calibrate_flag = true;
    } 
    else if (gpio == BUTTON_MODE_START_STOP) {
        g_btn_start_stop_flag = true;
    }
}

// --- main control task ---
void main_control_task(__unused void *params) {
    vTaskDelay(pdMS_TO_TICKS(500));
    printf("control task started.\n");
    
    set_servo_angle(CENTER_ANGLE);

    for (;;) {
        imu_get_heading();
        
        // --- handle calibration button ---
        if (g_btn_calibrate_flag) {
            g_btn_calibrate_flag = false;
            if (current_state == STATIONARY_NOT_CALIBRATED) {
                current_state = CALIBRATE_WHITE_SAMPLING;
            } else if (current_state == CALIBRATE_BLACK_WAIT) {
                current_state = CALIBRATE_BLACK_SAMPLING;
            } else if (current_state == CALIBRATED_STOPPED) {
                current_state = STATIONARY_NOT_CALIBRATED;
                g_is_calibrated = false;
                g_is_running = false;
            }
        }

        // --- handle start/stop button ---
        if (g_btn_start_stop_flag) {
            g_btn_start_stop_flag = false;
            if (g_is_calibrated) {
                g_is_running = !g_is_running;
                if (!g_is_running) {
                    g_maneuver_state = 0;
                    g_obs_step = 0;
                }
            } else {
                update_leds(50, 0, 0, 50, 0, 0); 
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        }

        // --- state machine ---
        switch (current_state) {
            case STATIONARY_NOT_CALIBRATED:
                update_leds(20, 0, 0, 20, 0, 0);
                motor_set_speed(MOTOR_LEFT, 0); 
                motor_set_speed(MOTOR_RIGHT, 0);
                break;

            case CALIBRATE_WHITE_SAMPLING: {
                uint64_t sum = 0;
                for(int i=0; i<50; i++) {
                    sum += ir_read_single_sensor_raw_local();
                    update_leds(50, 50, 50, 50, 50, 50); vTaskDelay(pdMS_TO_TICKS(50)); 
                    update_leds(0, 0, 0, 0, 0, 0);       vTaskDelay(pdMS_TO_TICKS(50)); 
                }
                g_calib_white = (uint16_t)(sum / 50);
                current_state = CALIBRATE_BLACK_WAIT;
                update_leds(0, 50, 0, 0, 50, 0);
                break;
            }

            case CALIBRATE_BLACK_WAIT:
                update_leds(0, 50, 0, 0, 50, 0);
                break;

            case CALIBRATE_BLACK_SAMPLING: {
                uint64_t sum = 0;
                for(int i=0; i<50; i++) {
                    sum += ir_read_single_sensor_raw_local();
                    update_leds(0, 0, 50, 0, 0, 50); vTaskDelay(pdMS_TO_TICKS(50)); 
                    update_leds(0, 0, 0, 0, 0, 0);   vTaskDelay(pdMS_TO_TICKS(50)); 
                }
                g_calib_black = (uint16_t)(sum / 50);
                ir_set_calibration_values(150, 900);
                g_is_calibrated = true;
                current_state = CALIBRATED_STOPPED;
                update_leds(0, 50, 0, 0, 50, 0); 
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            }

            case CALIBRATED_STOPPED:
                if (g_is_running) {
                    
                    // --- ultrasonic safety check ---
                    // only check if we are not already in a maneuver
                    if (g_maneuver_state == 0) {
                        static uint32_t last_us_check_time = 0;
                        static float last_known_dist = 100.0f; 
                        
                        uint32_t now_us = to_ms_since_boot(get_absolute_time());
                        if (now_us - last_us_check_time > 100) {
                            last_us_check_time = now_us;
                            float d = get_median_distance(1); 
                            if (d > 0.1f) last_known_dist = d;
                        }
                        
                        // obstacle detected in close range
                        if (last_known_dist < 23.0f && last_known_dist > 2.0f) {
                            // stop motors
                            motor_set_speed(MOTOR_LEFT, 0);
                            motor_set_speed(MOTOR_RIGHT, 0);
                            update_leds(50, 0, 0, 50, 0, 0);
                            
                            // perform scan
                            vTaskDelay(pdMS_TO_TICKS(500)); 
                            g_measured_width = perform_full_scan_routine(last_known_dist);
                            
                            // init avoidance maneuver
                            g_obs_base_heading = imu_get_heading();
                            g_maneuver_state = 1; // 1 = obstacle mode
                            g_obs_step = 1;       // start sequence
                            last_known_dist = 100.0f; // reset so we don't loop immediately
                        }
                    }

                    // --- barcode command check ---
                    // only check if no maneuver active
                    if (g_barcode_received && g_maneuver_state == 0) {
                        g_obs_base_heading = imu_get_heading();
                        
                        if (strcmp(g_last_barcode_cmd, "LEFT") == 0) {
                            g_obs_dir = -1;
                            g_maneuver_state = 2; // 2 = intersection mode
                        } 
                        else if (strcmp(g_last_barcode_cmd, "RIGHT") == 0) {
                            g_obs_dir = 1;
                            g_maneuver_state = 2;
                        }
                        g_barcode_received = false; 
                    }

                    // --- execute maneuver logic ---
                    if (g_maneuver_state == 1) {
                        // --- obstacle avoidance sequence (rectangular detour) ---
                        float target = 0;
                        switch (g_obs_step) {
                            case 1: // turn out 90
                                target = norm_angle(g_obs_base_heading + (g_obs_dir * 90.0f));
                                if (perform_turn_step(target)) {
                                    vTaskDelay(pdMS_TO_TICKS(100)); 
                                    g_obs_timer_start = to_ms_since_boot(get_absolute_time());
                                    g_obs_step++;
                                }
                                break;
                            case 2: // drive out
                                target = norm_angle(g_obs_base_heading + (g_obs_dir * 90.0f));
                                if (perform_drive_step(target, 800)) g_obs_step++;
                                break;
                            case 3: // turn parallel
                                target = norm_angle(g_obs_base_heading);
                                if (perform_turn_step(target)) {
                                    vTaskDelay(pdMS_TO_TICKS(100)); 
                                    g_obs_timer_start = to_ms_since_boot(get_absolute_time());
                                    g_obs_step++;
                                }
                                break;
                            case 4: // drive parallel (long)
                                target = norm_angle(g_obs_base_heading);
                                if (perform_drive_step(target, 1500)) g_obs_step++;
                                break;
                            case 5: // turn in
                                target = norm_angle(g_obs_base_heading - (g_obs_dir * 90.0f));
                                if (perform_turn_step(target)) {
                                    vTaskDelay(pdMS_TO_TICKS(100)); 
                                    g_obs_timer_start = to_ms_since_boot(get_absolute_time());
                                    g_obs_step++;
                                }
                                break;
                            case 6: // drive in to line
                                target = norm_angle(g_obs_base_heading - (g_obs_dir * 90.0f));
                                if (perform_drive_step(target, 800)) g_obs_step++;
                                break;
                            case 7: // align to line
                                target = norm_angle(g_obs_base_heading); 
                                if (perform_turn_step(target)) {
                                    ir_reset_pid_state(); 
                                    g_maneuver_state = 0; // complete
                                    g_obs_step = 0;
                                }
                                break;
                        }
                    } 
                    else if (g_maneuver_state == 2) {
                        // --- intersection turn sequence (simple 90 degree) ---
                        float target = norm_angle(g_obs_base_heading + (g_obs_dir * 90.0f));
                        if (perform_turn_step(target)) {
                            // drive forward briefly to clear intersection
                            motor_set_speed(MOTOR_LEFT, 35);
                            motor_set_speed(MOTOR_RIGHT, 35);
                            vTaskDelay(pdMS_TO_TICKS(300));
                            
                            ir_reset_pid_state();
                            g_maneuver_state = 0; // complete
                        }
                    } 
                    else {
                        // --- normal line following ---
                        ir_follow_line_single(g_line_follow_power, &g_target_power_left, &g_target_power_right, &g_sensor_R);
                        motor_set_speed(MOTOR_LEFT, g_target_power_left);
                        motor_set_speed(MOTOR_RIGHT, g_target_power_right);
                        
                        float steer = g_target_power_left - g_target_power_right;
                        handle_smart_indicators(steer);
                    }

                } else {
                    // stopped state
                    motor_set_speed(MOTOR_LEFT, 0); 
                    motor_set_speed(MOTOR_RIGHT, 0);
                    update_leds(0, 0, 20, 0, 0, 20);
                }
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_MS));
    }
}

// --- barcode task (high priority) ---
void barcode_task(__unused void *params) {
    for (;;) {
        // sampling at 300hz ensures accurate code39 detection
        barcode_step_300hz(); 

        run_t r;
        // process any detected run-length codes
        while (barcode_get_next_run(&r)) {
             // internal decoder logic updates g_barcode_received flag
        }
    }
}

// --- network task ---
void network_task(__unused void *params) {   
    if (cyw43_arch_init()) return;
    mqtt_client_init(&mqtt_state);
    while (!mqtt_client_try_wifi_connect(&mqtt_state, 10000)) { vTaskDelay(pdMS_TO_TICKS(2000)); }
    mqtt_client_start_mqtt(&mqtt_state);
    
    uint32_t last_pub = 0;
    for (;;) {
        mqtt_client_poll(&mqtt_state); 
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        if (mqtt_client_is_online(&mqtt_state) && (now - last_pub >= TELEMETRY_PUBLISH_INTERVAL_MS)) {
            last_pub = now;
            
            // publish fusion telemetry
            mqtt_pub_telemetry(&mqtt_state,
                (g_is_running ? 1 : 0), 
                g_current_ir_state,
                g_target_power_left, g_target_power_right,
                g_total_ticks_left, g_total_ticks_right,
                g_sensor_R, g_current_lost_time_ms, 
                g_measured_width); 

            // publish imu specific data
            mqtt_pub_imu_filtered(
                &mqtt_state, filtered_accel[0], filtered_accel[1], filtered_accel[2],
                roll_deg, pitch_deg, yaw_deg
            );
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// --- launch sequence ---
void vLaunch(void) {
    // 1. init hardware
    adc_init();
    adc_gpio_init(PIN_IR_RIGHT_ADC_GPIO);
    adc_select_input(PIN_IR_RIGHT_ADC_CH);
    
    // 2. init drivers
    led_init(); 
    motor_init();
    ir_init();       
    encoder_init(); 
    ultrasonic_init(TRIG_PIN, ECHO_PIN, SERVO_PIN);
    barcode_init();
    imu_init();
    
    // 3. init buttons
    gpio_init(BUTTON_MODE_CALIBRATE); gpio_set_dir(BUTTON_MODE_CALIBRATE, GPIO_IN); gpio_pull_up(BUTTON_MODE_CALIBRATE);
    gpio_init(BUTTON_MODE_START_STOP); gpio_set_dir(BUTTON_MODE_START_STOP, GPIO_IN); gpio_pull_up(BUTTON_MODE_START_STOP);
    
    // blocking calibration for barcode sensors if needed
    barcode_calibrate(); 

    gpio_set_irq_enabled_with_callback(BUTTON_MODE_CALIBRATE, GPIO_IRQ_EDGE_FALL, true, &gpio_interrupt_handler);
    gpio_set_irq_enabled(BUTTON_MODE_START_STOP, GPIO_IRQ_EDGE_FALL, true);
    
    // 4. start tasks
    xTaskCreate(network_task, "NetworkTask", 2048, NULL, NETWORK_TASK_PRIORITY, NULL);
    xTaskCreate(main_control_task, "MainTask", 1024, NULL, MAIN_CONTROL_TASK_PRIORITY, NULL);
    xTaskCreate(barcode_task, "BarcodeTask", 2048, NULL, BARCODE_TASK_PRIORITY, NULL);

    vTaskStartScheduler();
}

int main() {
    stdio_init_all();
    sleep_ms(3000); // safety delay for usb
    vLaunch();
    while(1);
    return 0;
}