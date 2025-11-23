#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "FreeRTOS.h"
#include "task.h"

// --- Master Configuration ---
#include "sys_config.h"

// --- Drivers ---
#include "motor/motor.h"
#include "ir/ir.h"
#include "ultrasonic/ultrasonic.h"
#include "encoder/encoder.h"
#include "imu/imu.h"
#include "network/mqtt.h"
#include "leds/ws2812.h"

// --- Task Priorities ---
#define MAIN_CONTROL_TASK_PRIORITY (tskIDLE_PRIORITY + 2UL)
#define NETWORK_TASK_PRIORITY      (tskIDLE_PRIORITY + 1UL)

// --- Global State ---
volatile RobotState current_state = STATIONARY_NOT_CALIBRATED;
volatile bool g_btn_calibrate_flag = false; 
volatile bool g_btn_start_stop_flag = false; 

// Operational Variables
int g_current_scenario = 0; 
float g_line_follow_power = 0.0f;
float g_target_power_left = 0.0f;
float g_target_power_right = 0.0f;
uint16_t g_calib_white = 0;
uint16_t g_calib_black = 0;
bool g_is_calibrated = false;
uint16_t g_sensor_R = 0;
const char *g_current_ir_state = "IDLE";
const char *g_ultrasonic_status = "CLEAR"; 
uint32_t g_current_lost_time_ms = 0;
bool g_object_scan_done = false; 
static MQTT_CLIENT_DATA_T mqtt_state;
uint32_t last_button_press_time = 0;

// --- Tuning Logic ---
void apply_scenario_tune(int scenario) {
    printf("\n--- CONFIGURING SCENARIO %d ---\n", scenario);
    
    // Defaults
    float kp = 32.0f, ki = 0.0f, kd = 0.6f;
    float steer_clamp = 30.0f;
    float base_scale = 1.0f;
    float curve_slowdown = 0.7f;
    float curve_exp = 1.3f;
    float min_speed_frac = 0.25f;
    
    uint32_t running_timeout = 800;
    uint32_t tuning_timeout = 5000;
    uint32_t to_R_L = running_timeout;
    uint32_t to_L_R = running_timeout;

    switch (scenario) {
        case 0: // Stop
            g_line_follow_power = 0.0f;
            printf("MODE: STOPPED\n");
            kp = 15.0f; kd = 0.0f; steer_clamp = 20.0f;
            ir_reset_robot_state();
            break;
        case 1: // Run (Centerline)
            g_line_follow_power = 45.0f;
            printf("MODE: RUN (Adaptive)\n");
            base_scale = 1.05f; curve_exp = 1.1f;
            ir_force_state(IR_STATE_TRACK_RIGHT);
            break;
        case 2: // Tune R->L
            g_line_follow_power = 45.0f;
            kd = 1.0f; base_scale = 1.00f; curve_exp = 1.1f;
            to_R_L = tuning_timeout; to_L_R = tuning_timeout;
            printf("MODE: TUNE R->L\n");
            ir_force_state(IR_STATE_TRACK_RIGHT);
            break;
        case 3: // Tune L->R
            g_line_follow_power = 45.0f;
            kd = 1.0f; base_scale = 1.00f; curve_exp = 1.1f;
            to_R_L = tuning_timeout; to_L_R = tuning_timeout;
            printf("MODE: TUNE L->R\n");
            ir_force_state(IR_STATE_TRACK_LEFT);
            break;
    }
    ir_set_tuning_params(kp, ki, kd, steer_clamp, base_scale, 
                         curve_slowdown, curve_exp, min_speed_frac, 
                         to_R_L, to_L_R);
    g_object_scan_done = false;
}

void gpio_interrupt_handler(uint gpio, uint32_t events) {
    uint32_t now = time_us_32();
    if ((now - last_button_press_time) < DEBOUNCE_TIME_US) return;
    last_button_press_time = now;
    if (gpio == BUTTON_MODE_CALIBRATE) g_btn_calibrate_flag = true;
    else if (gpio == BUTTON_MODE_START_STOP) g_btn_start_stop_flag = true;
}

// --- Main Task ---
void main_control_task(__unused void *params) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    set_servo_angle(CENTER_ANGLE);
    
    uint32_t last_print_time = 0;

    for (;;) {
        // 1. Update Sensors
        encoder_update_and_get_ticks(NULL, NULL);
        get_filtered_heading(raw_accel_data, raw_mag_data, filtered_accel, filtered_mag, &roll_deg, &pitch_deg, &yaw_deg);
        float yaw_rate = imu_get_yaw_rate();

        // 2. Handle Buttons
        if (g_btn_calibrate_flag) {
            g_btn_calibrate_flag = false;
            if (current_state == STATIONARY_NOT_CALIBRATED) current_state = CALIBRATE_WHITE_SAMPLING;
            else if (current_state == CALIBRATE_BLACK_WAIT) current_state = CALIBRATE_BLACK_SAMPLING;
            else if (current_state == CALIBRATED_STOPPED) {
                current_state = STATIONARY_NOT_CALIBRATED;
                g_is_calibrated = false;
                g_current_scenario = 0;
                update_leds(50,0,0, 50,0,0);
            }
        }
        if (g_btn_start_stop_flag) {
            g_btn_start_stop_flag = false;
            if (g_is_calibrated) {
                g_current_scenario = (g_current_scenario + 1) % TOTAL_SCENARIOS;
                apply_scenario_tune(g_current_scenario);
            }
        }

        // 3. State Machine
        switch (current_state) {
            case STATIONARY_NOT_CALIBRATED:
                g_current_ir_state = "NO_CAL";
                update_leds(50,0,0, 50,0,0);
                break;
            case CALIBRATE_WHITE_WAIT: break;
            case CALIBRATE_WHITE_SAMPLING: {
                printf(">> SAMPLING WHITE...\n");
                uint64_t sum = 0;
                for(int i=0; i<50; i++) {
                    sum += ir_read_single_sensor_raw();
                    update_leds(50,50,50, 50,50,50); vTaskDelay(pdMS_TO_TICKS(25));
                    update_leds(0,0,0, 0,0,0);       vTaskDelay(pdMS_TO_TICKS(25));
                }
                g_calib_white = (uint16_t)(sum / 50);
                printf(">> WHITE: %u\n", g_calib_white);
                current_state = CALIBRATE_BLACK_WAIT;
                update_leds(0,50,0, 0,50,0);
                break;
            }
            case CALIBRATE_BLACK_WAIT: break;
            case CALIBRATE_BLACK_SAMPLING: {
                printf(">> SAMPLING BLACK...\n");
                uint64_t sum = 0;
                for(int i=0; i<50; i++) {
                    sum += ir_read_single_sensor_raw();
                    update_leds(0,0,50, 0,0,50); vTaskDelay(pdMS_TO_TICKS(25));
                    update_leds(0,0,0, 0,0,0);   vTaskDelay(pdMS_TO_TICKS(25));
                }
                g_calib_black = (uint16_t)(sum / 50);
                printf(">> BLACK: %u\n", g_calib_black);
                ir_set_calibration_values(g_calib_white, g_calib_black);
                g_is_calibrated = true;
                current_state = CALIBRATED_STOPPED;
                update_leds(0,50,0, 0,50,0); vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            }
            case CALIBRATED_STOPPED:
                g_current_ir_state = "STOPPED";
                break;
        }

        // 4. Drive Logic
        if (g_current_scenario > 0 && g_is_calibrated && current_state == CALIBRATED_STOPPED) {
            float dist_cm = get_median_distance(3);
            bool override = false;

            if (dist_cm != -1.0f && dist_cm <= 30.0f) {
                if (dist_cm > 23.0f) {
                    g_ultrasonic_status = "APPROACH";
                    g_line_follow_power = 30.0f;
                    g_object_scan_done = false;
                } else if (dist_cm >= 18.0f && dist_cm <= 23.0f) {
                    g_ultrasonic_status = "SCANNING";
                    override = true;
                    motor_set_speed(MOTOR_LEFT, 0.0f); motor_set_speed(MOTOR_RIGHT, 0.0f);
                    update_leds(50,0,50, 50,0,50);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    if (!g_object_scan_done) {
                        printf(">> SCANNING...\n");
                        perform_full_scan_routine(dist_cm);
                        g_object_scan_done = true;
                        while(get_median_distance(5) < 30.0f) vTaskDelay(pdMS_TO_TICKS(100));
                    }
                } else {
                    g_ultrasonic_status = "TOO_CLOSE";
                    override = true;
                    motor_set_speed(MOTOR_LEFT, 0.0f); motor_set_speed(MOTOR_RIGHT, 0.0f);
                    update_leds(50,0,0, 50,0,0);
                }
            } else {
                g_ultrasonic_status = "CLEAR";
            }

            if (!override) {
                ir_follow_line_single(g_line_follow_power, &g_target_power_left, &g_target_power_right,
                                      &g_sensor_R, &g_current_ir_state, &g_current_lost_time_ms, yaw_rate);
                motor_set_speed(MOTOR_LEFT, g_target_power_left);
                motor_set_speed(MOTOR_RIGHT, g_target_power_right);
                handle_smart_indicators(g_target_power_left - g_target_power_right);
            }
        } else {
            motor_set_speed(MOTOR_LEFT, 0.0f); motor_set_speed(MOTOR_RIGHT, 0.0f);
            if(current_state == CALIBRATED_STOPPED) update_leds(0,0,20, 0,0,20);
        }

        // 5. Print
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - last_print_time > PRINT_DELAY_MS) {
            last_print_time = now_ms;
            if (g_current_scenario > 0) {
                printf("RUN[%d] | %s | US:%s | P:%.0f/%.0f | L_ms:%u\n", 
                    g_current_scenario, g_current_ir_state, g_ultrasonic_status, 
                    g_target_power_left, g_target_power_right, g_current_lost_time_ms);
            } else {
                printf("STOP | St:%d | ADC:%4u\n", current_state, ir_read_single_sensor_raw());
            }
        }
        vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_MS));
    }
}

// --- Network Task ---
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
            mqtt_pub_telemetry(&mqtt_state, g_current_scenario, g_current_ir_state,
                g_target_power_left, g_target_power_right, 
                encoder_get_total_ticks_left(), encoder_get_total_ticks_right(),
                (g_current_scenario > 0 ? g_sensor_R : ir_read_single_sensor_raw()),
                g_current_lost_time_ms, (int)current_state);
            
            mqtt_pub_imu_filtered(&mqtt_state, filtered_accel[0], filtered_accel[1], filtered_accel[2],
                roll_deg, pitch_deg, yaw_deg);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void vLaunch(void) {
    // Initialization
    leds_init(); // New unified LED init
    imu_init();
    motor_init();
    ir_init();
    encoder_init();
    ultrasonic_init(TRIG_PIN, ECHO_PIN, SERVO_PIN);

    // Buttons
    gpio_init(BUTTON_MODE_CALIBRATE); gpio_set_dir(BUTTON_MODE_CALIBRATE, GPIO_IN); gpio_pull_up(BUTTON_MODE_CALIBRATE);
    gpio_init(BUTTON_MODE_START_STOP); gpio_set_dir(BUTTON_MODE_START_STOP, GPIO_IN); gpio_pull_up(BUTTON_MODE_START_STOP);
    gpio_set_irq_enabled_with_callback(BUTTON_MODE_CALIBRATE, GPIO_IRQ_EDGE_FALL, true, &gpio_interrupt_handler);
    gpio_set_irq_enabled(BUTTON_MODE_START_STOP, GPIO_IRQ_EDGE_FALL, true);

    printf("Mode: STATIONARY. Press GP21 to start calibration.\n");

    xTaskCreate(network_task, "NetworkTask", 2048, NULL, NETWORK_TASK_PRIORITY, NULL);
    xTaskCreate(main_control_task, "MainControlTask", 1024, NULL, MAIN_CONTROL_TASK_PRIORITY, NULL);
    vTaskStartScheduler();
}


int main() {
    // 1. Set Clock FIRST (Moved from leds_init)
    // We do this here so USB init knows the correct speed.
    set_sys_clock_khz(125000, true); 

    // 2. Initialize Stdlo (USB Serial)
    stdio_init_all();
    sleep_ms(3000); // Wait for you to open the serial monitor

    printf("System Clock Set to 125MHz. Starting...\n");

    // 3. Launch the RTOS
    vLaunch();
    return 0;
}