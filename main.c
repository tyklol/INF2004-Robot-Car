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
#define TRIG_PIN 5
#define ECHO_PIN 4
#define SERVO_PIN 15

// --- settings ---
#define DEBOUNCE_TIME_US   200000 
#define CONTROL_LOOP_MS    10     
#define TELEMETRY_PUBLISH_INTERVAL_MS 200

// --- task priorities ---
#define BARCODE_TASK_PRIORITY       (tskIDLE_PRIORITY + 4UL)
#define ULTRASONIC_TASK_PRIORITY    (tskIDLE_PRIORITY + 3UL) // High priority to catch obstacles fast
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
float g_line_follow_power = 38.0f; 
float g_target_power_left = 0.0f;
float g_target_power_right = 0.0f;
uint16_t g_sensor_R = 0;
const char *g_current_ir_state = "IDLE";
uint32_t g_current_lost_time_ms = 0;

// ultrasonic telemetry
float g_measured_width = 0.0f;
float distance_to_obstacle_cm = 0.0f;

// encoder globals
volatile float g_current_speed_cms = 0.0f;
volatile float g_total_distance_cm = 0.0f;

// imu / obstacle avoidance globals
int g_maneuver_state = 0;        // 0 = idle, 1 = obstacle detour, 2 = intersection turn
int g_obs_step = 0;              // 1-7 steps for rectangular avoidance
int g_obs_dir = 0;               // 1 = right, -1 = left
float g_obs_base_heading = 0.0f; // reference heading
uint32_t g_obs_timer_start = 0;  // timer for blind moves

// --- barcode shared globals ---
char g_last_barcode_cmd[32] = {0};
volatile bool g_barcode_received = false; 
char g_last_barcode_payload[60] = "";

static MQTT_CLIENT_DATA_T mqtt_state;
// --- adc helper (thread safe) ---
uint16_t ir_read_single_sensor_raw_local(void) {
    taskENTER_CRITICAL();
    adc_select_input(PIN_IR_RIGHT_ADC_CH);
    uint16_t val = adc_read();
    taskEXIT_CRITICAL();
    return val;
}

// --- imu helper (thread safe) ---
// Prevents collision between main task and ultrasonic task using I2C
float imu_get_heading_safe(void) {
    taskENTER_CRITICAL();
    float h = imu_get_heading();
    taskEXIT_CRITICAL();
    return h;
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
    static bool is_braking = false;
    static uint32_t brake_start_time = 0;
    
    float current_yaw = imu_get_heading_safe();
    float error = get_angle_diff(target_heading, current_yaw);

    // RESET LOGIC: If error is huge (start of a new turn), ensure braking is off
    if (fabs(error) > 10.0f) {
        is_braking = false;
    }

    // --- PHASE 1: BRAKING SEQUENCE ---
    if (is_braking) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        // Brake for exactly 150ms
        if (now - brake_start_time > 150) {
            motor_set_speed(MOTOR_LEFT, 0);
            motor_set_speed(MOTOR_RIGHT, 0);
            is_braking = false; // Reset for next time
            return true;        // Turn Complete
        }
        
        // Apply "Counter-Torque" to stop rotation instantly
        // If we were turning RIGHT, error was (+), so we need LEFT torque
        // We just lock motors against the spin
        motor_set_speed(MOTOR_LEFT, 0);
        motor_set_speed(MOTOR_RIGHT, 0);
        // Optional: Active reverse kick if 0 isn't enough:
        // motor_set_speed(MOTOR_LEFT, -20); motor_set_speed(MOTOR_RIGHT, -20);
        
        return false; // Still braking
    }

    // --- PHASE 2: ARRIVAL CHECK ---
    // Stop EARLY (at 3 degrees) and let the brake/momentum finish the job
    if (fabs(error) < 3.0f) {
        is_braking = true;
        brake_start_time = to_ms_since_boot(get_absolute_time());
        
        // Hard Stop motors immediately to begin braking phase
        motor_set_speed(MOTOR_LEFT, 0);
        motor_set_speed(MOTOR_RIGHT, 0);
        return false;
    }

    // --- PHASE 3: PID CONTROL ---
    // Use aggressive P-term since we now have braking to catch us
    float kp = (fabs(error) > 20.0f) ? 3.5f : 2.5f;
    float turn_speed = error * kp; 

    // Stiction Boost
    float min_speed = 28.0f;
    if (fabs(error) < 8.0f) min_speed = 32.0f; // Boost more when close to overcome static friction

    if (turn_speed > 0) {
        if (turn_speed < min_speed) turn_speed = min_speed;
        if (turn_speed > 60.0f) turn_speed = 60.0f; 
    } else {
        if (turn_speed > -min_speed) turn_speed = -min_speed;
        if (turn_speed < -60.0f) turn_speed = -60.0f;
    }

    motor_set_speed(MOTOR_LEFT, turn_speed);
    motor_set_speed(MOTOR_RIGHT, -turn_speed);
    
    update_leds(50, 0, 50, 50, 0, 50); 
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

    float current_yaw = imu_get_heading_safe();
    float error = get_angle_diff(target_heading, current_yaw);
    
    // 1. INCREASED GAIN (Was 1.5f, now 5.0f)
    // A higher gain forces the robot to react strongly to even 
    // small deviations (drift), keeping it much straighter.
    float Kp = 5.0f; 
    float correction = error * Kp; 

    float forward_speed = 35.0f; 
    
    float pwr_left = forward_speed + correction;
    float pwr_right = forward_speed - correction;

    // 2. ANTI-STALL CLAMP
    // If we correct too hard, one motor might drop below the stall speed (e.g., 20).
    // This forces the slower motor to keep moving, ensuring a smooth curve 
    // rather than a sharp pivot.
    if (pwr_left < 20.0f) pwr_left = 20.0f;
    if (pwr_right < 20.0f) pwr_right = 20.0f;

    // 3. MAX SPEED CLAMP (Optional safety)
    if (pwr_left > 70.0f) pwr_left = 70.0f;
    if (pwr_right > 70.0f) pwr_right = 70.0f;

    motor_set_speed(MOTOR_LEFT, pwr_left);
    motor_set_speed(MOTOR_RIGHT, pwr_right);
    
    update_leds(50, 50, 0, 50, 50, 0); // yellow indicator
    return false;
}

// --- helper: perform arc turn ---
bool perform_arc_step(float target_heading, float inner_wheel_ratio, int direction) {
    float current_yaw = imu_get_heading_safe();
    float error = get_angle_diff(target_heading, current_yaw);

    // Base speed for the outer wheel (Main movement speed)
    float outer_speed = 45.0f; 
    float inner_speed = outer_speed * inner_wheel_ratio;

    // Determine which wheel is inner/outer based on turn direction
    float left_speed, right_speed;

    // If we need to turn LEFT (direction -1), Left wheel is Inner
    if (direction == -1) {
        left_speed = inner_speed;
        right_speed = outer_speed;
    } 
    // If we need to turn RIGHT (direction 1), Right wheel is Inner
    else {
        left_speed = outer_speed;
        right_speed = inner_speed;
    }

    // Apply to motors
    motor_set_speed(MOTOR_LEFT, left_speed);
    motor_set_speed(MOTOR_RIGHT, right_speed);
    
    // Indicators (Cyan for smooth arc)
    update_leds(0, 50, 50, 0, 50, 50); 

    // Completion Check (Wider window for smooth motion)
    // As soon as we pass the target or get very close, we stop/transition
    if (fabs(error) < 5.0f) {
        return true;
    }
    return false;
}

// --- master isr for buttons ---
void gpio_interrupt_handler(uint gpio, uint32_t events) {
    // 1. Handle Encoders FIRST (High speed, no debounce needed)
    // We pass the event to the driver to check if the pin matches
    encoder_handle_interrupt(gpio, events);

    // 2. Handle Buttons (Low speed, needs debounce)
    if (gpio == BUTTON_MODE_CALIBRATE || gpio == BUTTON_MODE_START_STOP) {
        
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        // Check debounce only for buttons
        if (now - last_button_press_time > 200) { 
            last_button_press_time = now;

            if (gpio == BUTTON_MODE_CALIBRATE) {
                g_btn_calibrate_flag = true;
            } 
            else if (gpio == BUTTON_MODE_START_STOP) {
                g_btn_start_stop_flag = true;
            }
        }
    }
}

// --- NEW TASK: ULTRASONIC TASK ---
void ultrasonic_task(__unused void *params) {
    set_servo_angle(CENTER_ANGLE);
    
    int detection_counter = 0;
    const int REQUIRED_DETECTIONS = 2; 

    for (;;) {
        if (g_is_running && g_maneuver_state == 0) {
            
            float dist = get_median_distance(3); 
            
            if (dist > 0.1f) distance_to_obstacle_cm = dist;

            // CHECK RANGE (Trigger condition)
            if (distance_to_obstacle_cm < 17.0f && distance_to_obstacle_cm > 4.0f) {
                detection_counter++;
            } else {
                detection_counter = 0; 
            }

            if (detection_counter >= REQUIRED_DETECTIONS) {
                detection_counter = 0;

                // 1. ENTER STOP/SCAN STATE
                // This stops the motors (handled in main_control_task under case 3)
                g_maneuver_state = 3; 
                
                // 2. WAIT FOR FULL STOP & SCAN
                // Give it 500ms to physically stop and servo to settle
                vTaskDelay(pdMS_TO_TICKS(500)); 

                // 3. PERFORM SCAN
                g_measured_width = perform_full_scan_routine(distance_to_obstacle_cm);
                
                // 4. *** CRITICAL FIX: WAIT FOR IMU TO SETTLE ***
                // Now that the scan is done and robot is still, wait 200ms
                // to let the IMU filter catch up to the stationary position.
                vTaskDelay(pdMS_TO_TICKS(200));

                // 5. CAPTURE BASE HEADING
                // This is now accurate because the robot has been still for >2s
                g_obs_base_heading = imu_get_heading_safe();
                
                // 6. SETUP MANEUVER
                // g_obs_dir = scan_for_clear_path();
                g_obs_dir = -1; // Default Left for now
                g_obs_step = 1;
                g_maneuver_state = 1; // Start driving
                
                // Reset distance so we don't trigger again immediately
                distance_to_obstacle_cm = 100.0f;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// --- main control task ---
void main_control_task(__unused void *params) {
    vTaskDelay(pdMS_TO_TICKS(500));
    printf("control task started.\n");
    
    set_servo_angle(CENTER_ANGLE);

    // ensure barcode calibration runs only once per calibration cycle
    static bool barcode_cal_done = false;
    ir_set_tuning_params(45.0f, 0.0f, 7.0f, 65.0f, 1.0f, 0.5f, 1.0f, 0.4f);

    // Timer for speed calculation
    uint32_t last_speed_update = 0;

    for (;;) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        // --- 1. UPDATE ENCODER DATA (Run every 100ms) ---
        if (now - last_speed_update >= 100) {
            last_speed_update = now;
            
            // Calculate and update globals
            g_current_speed_cms = encoder_calculate_speed_cms();
            g_total_distance_cm = encoder_get_distance_cm();
        }
        
        imu_get_heading_safe();
        
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

                // allow barcode recalibration next time
                barcode_cal_done = false;
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
                for(int i=0; i<10; i++) {
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
                for(int i=0; i<10; i++) {
                    sum += ir_read_single_sensor_raw_local();
                    update_leds(0, 0, 50, 0, 0, 50); vTaskDelay(pdMS_TO_TICKS(50)); 
                    update_leds(0, 0, 0, 0, 0, 0);   vTaskDelay(pdMS_TO_TICKS(50)); 
                }
                g_calib_black = (uint16_t)(sum / 50);
                ir_set_calibration_values(150, 1200);
                g_is_calibrated = true;
                current_state = CALIBRATED_STOPPED;

                update_leds(0, 50, 0, 0, 50, 0); 
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            }

            case CALIBRATED_STOPPED:
                if (g_is_running) { 
                    // --- barcode command check ---
                    // only check if no maneuver active
                    if (g_barcode_received && g_maneuver_state == 0) {
                        g_obs_base_heading = imu_get_heading_safe();
                        
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
                    if (g_maneuver_state == 3) {
                        // scanning state - motors off
                        motor_set_speed(MOTOR_LEFT, 0);
                        motor_set_speed(MOTOR_RIGHT, 0);
                        update_leds(50, 0, 0, 50, 0, 0); // Red LEDs
                    }
                    // --- execute maneuver logic (OBSTACLE AVOIDANCE) ---
                    else if (g_maneuver_state == 1) {
                        float target = 0;
                        
                        // Setup clearance time based on measured width
                        // This determines how long we drive PARALLEL to the obstacle
                        // uint32_t clearance_time = (uint32_t)(g_measured_width * 45.0f); // Tweak multiplier as needed
                        // if (clearance_time < 1000) clearance_time = 1000; // Minimum box length
                        uint32_t clearance_time = 3000;

                        // Fixed time to drive AWAY from and BACK TO the track (depth of the box)
                        uint32_t depth_time = 1800; 

                        switch (g_obs_step) {
                            // --- STEP 1: TURN OUT (90 deg) ---
                            case 1: 
                                target = norm_angle(g_obs_base_heading + (g_obs_dir * 90.0f));
                                if (perform_turn_step(target)) {
                                    // Turn complete. Reset timer for the next Drive step.
                                    vTaskDelay(pdMS_TO_TICKS(200)); // Brief pause for stability
                                    g_obs_timer_start = to_ms_since_boot(get_absolute_time());
                                    g_obs_step++;
                                }
                                break;
                            // --- STEP 2: DRIVE OUT (Create depth) ---
                            case 2: 
                                target = norm_angle(g_obs_base_heading + (g_obs_dir * 90.0f));
                                if (perform_drive_step(target, depth_time)) {
                                    g_obs_step++;
                                }
                                break;
                            // --- STEP 3: TURN PARALLEL (Face 0 deg relative to base) ---
                            case 3: 
                                target = norm_angle(g_obs_base_heading);
                                if (perform_turn_step(target)) {
                                    vTaskDelay(pdMS_TO_TICKS(200));
                                    g_obs_timer_start = to_ms_since_boot(get_absolute_time());
                                    g_obs_step++;
                                }
                                break;
                            // --- STEP 4: DRIVE PARALLEL (Clear the width) ---
                            case 4: 
                                target = norm_angle(g_obs_base_heading);
                                if (perform_drive_step(target, clearance_time)) {
                                    g_obs_step++;
                                }
                                break;
                            // --- STEP 5: TURN IN (Face towards track) ---
                            // If we dodged Left (-1), we need to turn Right (+90 relative to base) to face track.
                            case 5: 
                                target = norm_angle(g_obs_base_heading - (g_obs_dir * 90.0f));
                                if (perform_turn_step(target)) {
                                    vTaskDelay(pdMS_TO_TICKS(200));
                                    g_obs_timer_start = to_ms_since_boot(get_absolute_time());
                                    g_obs_step++;
                                }
                                break;
                            // --- STEP 6: DRIVE IN (Return to line) ---
                            case 6: 
                                target = norm_angle(g_obs_base_heading - (g_obs_dir * 90.0f));
                                if (perform_drive_step(target, depth_time + 200)) {
                                    g_obs_step++;
                                }
                                break;
                            // --- STEP 7: ALIGN (Face Forward) ---
                            case 7: 
                                target = norm_angle(g_obs_base_heading);
                                if (perform_turn_step(target)) {
                                    printf("OBSTACLE: Box Maneuver Complete.\n");
                                    ir_reset_pid_state(); 
                                    g_maneuver_state = 0; 
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
                        ir_follow_line_single(g_line_follow_power, &g_target_power_left, &g_target_power_right, &g_sensor_R, &g_current_ir_state, &g_current_lost_time_ms);
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
    // background scanner that fills globals for main_control_task
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("\n=== Barcode Task Started (%d Hz) ===\n", BARCODE_SAMPLE_HZ);

    static char prev_payload[60] = "";

    for (;;) {
        // ensure ADC is on barcode channel before stepping
        taskENTER_CRITICAL();
        adc_select_input(BARCODE_ADC_CH); // ADC1 for GP27
        taskEXIT_CRITICAL();

        // sampling at BARCODE_SAMPLE_HZ ensures accurate code39 detection
        barcode_step_300hz(); 

        // Get latest decoded data from barcode.c
        const char *p = barcode_last_payload();    // e.g. "Z"
        const char *c = barcode_last_command();    // e.g. "LEFT"
        const char *s = barcode_last_symbols();    // e.g. "*Z*"

        // new decode when payload changes (non-empty)
        if (p && p[0] && strcmp(prev_payload, p) != 0) {
            // ---- PRINT FROM MAIN (what you want) ----
            if (s && s[0]) {
                printf("[MAIN] SYMS: %s\n", s);
            }
            printf("[MAIN] Decoded: %s\n", p);
            printf("[MAIN] Command: %s\n", (c && c[0]) ? c : "NONE");

            // keep a copy so we only react once per scan
            strncpy(prev_payload, p, sizeof(prev_payload) - 1);
            prev_payload[sizeof(prev_payload) - 1] = '\0';

            // ---- UPDATE GLOBALS FOR OTHER TASKS + MQTT ----
            strncpy(g_last_barcode_payload, p, sizeof(g_last_barcode_payload) - 1);
            g_last_barcode_payload[sizeof(g_last_barcode_payload) - 1] = '\0';

            if (c && c[0]) {
                strncpy(g_last_barcode_cmd, c, sizeof(g_last_barcode_cmd) - 1);
                g_last_barcode_cmd[sizeof(g_last_barcode_cmd) - 1] = '\0';
            } else {
                strcpy(g_last_barcode_cmd, "NONE");
            }

            g_barcode_received = true;

            // optional summary line
            printf("[BC] Payload=%s | Cmd=%s\n",
                   g_last_barcode_payload, g_last_barcode_cmd);
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // yield to other tasks
    }

}

// --- network task ---
void network_task(__unused void *params) {   
    if (cyw43_arch_init()) return;
    mqtt_client_init(&mqtt_state);
    while (!mqtt_client_try_wifi_connect(&mqtt_state, 10000)) { vTaskDelay(pdMS_TO_TICKS(2000)); }
    mqtt_client_start_mqtt(&mqtt_state);
    
    uint32_t last_pub = 0;

    encoder_reset_total_ticks();
    for (;;) {
        mqtt_client_poll(&mqtt_state); 
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        if (mqtt_client_is_online(&mqtt_state) && (now - last_pub >= TELEMETRY_PUBLISH_INTERVAL_MS)) {
            last_pub = now;

            float speed_cms = encoder_calculate_speed_cms();
            float dist_cm = encoder_get_distance_cm();
            
            // publish fusion telemetry
            mqtt_pub_telemetry(&mqtt_state,
                (g_is_running ? 1 : 0), 
                g_current_ir_state,
                g_target_power_left, g_target_power_right,
                encoder_get_left_ticks(), encoder_get_right_ticks(),
                g_sensor_R, g_current_lost_time_ms, 
                speed_cms, dist_cm); 

            // publish imu specific data
            mqtt_pub_imu_filtered(
                &mqtt_state, filtered_accel[0], filtered_accel[1], filtered_accel[2],
                roll_deg, pitch_deg, yaw_deg
            );

            mqtt_pub_obstacle(&mqtt_state, distance_to_obstacle_cm, g_measured_width);

            mqtt_pub_barcode(&mqtt_state, g_last_barcode_cmd, g_last_barcode_payload);
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

    // barcode_calibrate();
    
    // 3. init buttons
    gpio_init(BUTTON_MODE_CALIBRATE); gpio_set_dir(BUTTON_MODE_CALIBRATE, GPIO_IN); gpio_pull_up(BUTTON_MODE_CALIBRATE);
    gpio_init(BUTTON_MODE_START_STOP); gpio_set_dir(BUTTON_MODE_START_STOP, GPIO_IN); gpio_pull_up(BUTTON_MODE_START_STOP);

    gpio_set_irq_enabled_with_callback(BUTTON_MODE_CALIBRATE, GPIO_IRQ_EDGE_FALL, true, &gpio_interrupt_handler);
    gpio_set_irq_enabled(BUTTON_MODE_START_STOP, GPIO_IRQ_EDGE_FALL, true);
    
    // 4. start tasks
    xTaskCreate(network_task, "NetworkTask", 2048, NULL, NETWORK_TASK_PRIORITY, NULL);
    xTaskCreate(main_control_task, "MainTask", 1024, NULL, MAIN_CONTROL_TASK_PRIORITY, NULL);
    // xTaskCreate(barcode_task, "BarcodeTask", 2048, NULL, BARCODE_TASK_PRIORITY, NULL);
    xTaskCreate(ultrasonic_task, "UltrasonicTask", 1024, NULL, ULTRASONIC_TASK_PRIORITY, NULL);

    vTaskStartScheduler();
}

int main() {
    stdio_init_all();
    sleep_ms(3000); // safety delay for usb
    vLaunch();
    while(1);
    return 0;
}
