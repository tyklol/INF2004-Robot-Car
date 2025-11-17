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
void apply_scenario_tune(int scenario) {
    printf("\n--- CONFIGURING SCENARIO %d ---\n", scenario);
    
    // Default values (from your "Balanced & Fast" tune)
    float kp = 32.0f;
    float ki = 0.0f;
    float kd = 0.6f;
    float steer_clamp = 30.0f;
    float base_scale = 1.0f;
    float curve_slowdown = 0.7f;
    float curve_exp = 1.3f;
    float min_speed_frac = 0.25f;

    // --- 
    // --- !!! YOUR FINAL TUNED VALUES GO HERE !!! ---
    // --- 
    // (Values from your logs)
    uint32_t final_timeout_R_L = 1200; // (Based on 228ms max wobble)
    uint32_t final_timeout_L_R = 1200; // (Based on 453ms max wobble)
    // ---
    // ---
    
    uint32_t tuning_timeout = 5000;   // Safe value for tuning modes

    switch (scenario) {
        case 0: // STOPPED
            g_line_follow_power = 0.0f;
            printf("MODE: STOPPED (Scenario 0)\n");
            kp = 15.0f; kd = 0.0f; steer_clamp = 20.0f; // Use safe/default params
            ir_set_tuning_params(kp, ki, kd, steer_clamp, base_scale, 
                                 curve_slowdown, curve_exp, min_speed_frac, 
                                 final_timeout_R_L, final_timeout_L_R);
            ir_reset_robot_state(); // Reset to default (RIGHT_EDGE) and reset PID
            break;
        
        case 1: // "RUN" Mode
            g_line_follow_power = 45.0f; 
            printf("MODE: RUN (Scenario 1)\n");
            printf("R->L Timeout: %lu ms | L->R Timeout: %lu ms\n", final_timeout_R_L, final_timeout_L_R);
            kp = 32.0f;
            ki = 0.0f;
            kd = 0.6f;
            steer_clamp = 30.0f;
            ir_set_tuning_params(kp, ki, kd, steer_clamp, base_scale, 
                                 curve_slowdown, curve_exp, min_speed_frac, 
                                 final_timeout_R_L, final_timeout_L_R);
            ir_reset_robot_state(); // Reset to default (RIGHT_EDGE) and reset PID
            break;

        case 2: // "TUNE" Mode (Right-to-Left)
            g_line_follow_power = 40.0f;
            kp = 40.0f;       // from KP_SINGLE
            ki = 0.0f;       // from KI_SINGLE
            kd = 2.0f;       // from KD_SINGLE
            steer_clamp = 30.0f;   // from STEER_CLAMP_TPS
            base_scale = 0.85f;   // from BASE_SPEED_SCALE
            curve_slowdown = 0.70f;  // from CURVE_SLOWDOWN
            curve_exp = 1.6f;    // from CURVE_EXP
            min_speed_frac = 0.30f;  // from MIN_SPEED_FRACTION
            printf("MODE: TUNE R->L | Forcing RIGHT_EDGE\n");
            ir_set_tuning_params(kp, ki, kd, steer_clamp, base_scale, 
                                 curve_slowdown, curve_exp, min_speed_frac, 
                                 tuning_timeout, tuning_timeout);
            ir_force_state(IR_STATE_TRACK_RIGHT); // Force RIGHT_EDGE and reset PID
            break;

        case 3: // "TUNE" Mode (Left-to-Right)
            g_line_follow_power = 45.0f;
            kp = 32.0f;
            ki = 0.0f;
            kd = 0.6f;
            steer_clamp = 30.0f;
            printf("MODE: TUNE L->R | Forcing LEFT_EDGE\n");
            ir_set_tuning_params(kp, ki, kd, steer_clamp, base_scale, 
                                 curve_slowdown, curve_exp, min_speed_frac, 
                                 tuning_timeout, tuning_timeout);
            ir_force_state(IR_STATE_TRACK_LEFT); // Force LEFT_EDGE and reset PID
            break;
    }
    
    // ---
    // THE BUG WAS LIKELY A STRAY ir_reset_robot_state(); CALL HERE.
    // MAKE SURE IT IS GONE.
    // ---
}

/**
 * @brief MASTER ISR for BUTTONS
 * MODIFIED: Cycles scenarios 0-3
 */
void gpio_interrupt_handler(uint gpio, uint32_t events) {
    if (gpio == BUTTON_MODE_CALIBRATE || gpio == BUTTON_MODE_START_STOP) {
        uint32_t now = time_us_32();
        if (now - last_button_press_time < DEBOUNCE_TIME_US) {
            return;
        }
        last_button_press_time = now;

        if (gpio == BUTTON_MODE_CALIBRATE) {
            // (Calibration logic unchanged)
            if (current_state == STATIONARY_NOT_CALIBRATED) {
                current_state = CALIBRATE_WHITE_START;
            } else if (current_state == CALIBRATE_WHITE_START) {
                current_state = CALIBRATE_WHITE_END;
            } else if (current_state == CALIBRATE_WHITE_END) {
                current_state = CALIBRATE_BLACK_START;
            } else if (current_state == CALIBRATE_BLACK_START) {
                current_state = CALIBRATED_STOPPED;
            } else if (current_state == CALIBRATED_STOPPED) {
                current_state = STATIONARY_NOT_CALIBRATED;
                g_is_calibrated = false;
                g_current_scenario = 0;
            }
        } else if (gpio == BUTTON_MODE_START_STOP) {
            // Scenario Cycle Button Logic
            if (g_is_calibrated) {
                // --- MODIFIED: Cycle 0, 1, 2, 3 ---
                g_current_scenario = (g_current_scenario + 1) % 4; 
            }
        }
        needs_update = true;
    }
}

/**
 * @brief TASK 1: Handles all high-frequency robot control logic.
 */
void main_control_task(__unused void *params) {
    vTaskDelay(pdMS_TO_TICKS(500));
    printf("Main control task started (4-Mode Tuning Ready).\n");

    for (;;) {
        // (Encoder poll unchanged)
        int32_t delta_ticks_left, delta_ticks_right;
        encoder_update_and_get_ticks(&delta_ticks_left, &delta_ticks_right);
        g_total_ticks_left = encoder_get_total_ticks_left();
        g_total_ticks_right = encoder_get_total_ticks_right();

        // (State change logic unchanged)
        if (needs_update) {
            needs_update = false;
            encoder_reset_total_ticks();
            g_total_ticks_left = 0;
            g_total_ticks_right = 0;

            switch (current_state) {
                case STATIONARY_NOT_CALIBRATED:
                    g_current_ir_state = "NO_CAL";
                    printf("Mode: NOT CALIBRATED. Press GP21 to start.\n");
                    break;
                case CALIBRATE_WHITE_START:
                    g_current_ir_state = "CAL_WHITE";
                    printf("Mode: Place sensor on WHITE, press GP21 to save.\n");
                    break;
                case CALIBRATE_WHITE_END:
                    g_current_ir_state = "CAL_WHITE";
                    g_calib_white = ir_read_single_sensor_raw();
                    printf("Mode: Place sensor on WHITE, press GP21 to save.\n");
                    break;
                case CALIBRATE_BLACK_START:
                    g_current_ir_state = "CAL_BLACK";
                    printf("WHITE saved: %u. Place on BLACK, press GP21 to save.\n", g_calib_white);
                    break;
                case CALIBRATED_STOPPED:
                    g_calib_black = ir_read_single_sensor_raw();
                    ir_set_calibration_values(g_calib_white, g_calib_black);
                    g_is_calibrated = true;
                    g_current_ir_state = "CAL_STOP";
                    printf("BLACK saved: %u. CALIBRATED! Press GP20 to cycle scenarios.\n", g_calib_black);
                    printf("Calibration Range: %d (should be > 500)\n", (int)(g_calib_black - g_calib_white));
                    
                    apply_scenario_tune(g_current_scenario);
                    indicate_state_led(g_current_scenario);
                    break;
            }

            if (g_is_calibrated) {
                apply_scenario_tune(g_current_scenario);
                indicate_state_led(g_current_scenario);
            }
        }

        // --- Calculate Target Motor Power ---
        if (g_current_scenario > 0 && g_is_calibrated) { // Scenarios 1, 2, or 3
            
            // --- MODIFIED: Add new argument for lost time ---
            ir_follow_line_single(g_line_follow_power,
                                  &g_target_power_left, 
                                  &g_target_power_right,
                                  &g_sensor_R,
                                  &g_current_ir_state,
                                  &g_current_lost_time_ms); // <-- NEW
        } else { // Scenario 0 (STOP) or not calibrated
            g_target_power_left = 0.0f;
            g_target_power_right = 0.0f;
            g_current_lost_time_ms = 0;
        }

        // --- Actuate Motors ---
        motor_set_speed(MOTOR_LEFT, g_target_power_left);
        motor_set_speed(MOTOR_RIGHT, g_target_power_right);

        // --- Print Debugging Information ---
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - g_last_print_time > PRINT_DELAY_MS) {
            g_last_print_time = now_ms;
            
            if (g_current_scenario > 0) {
                // --- MODIFIED: Added L_ms (Lost Time in ms) ---
                printf("SCEN %d: %-10s| Pwr_L:%6.1f Pwr_R:%6.1f | Tick_L:%6ld Tick_R:%6ld | ADC:%4u | L_ms:%5u\n",
                       g_current_scenario,
                       g_current_ir_state,
                       g_target_power_left,
                       g_target_power_right,
                       g_total_ticks_left,
                       g_total_ticks_right,
                       g_sensor_R,
                       g_current_lost_time_ms); // <-- NEW
            } else {
                 printf("STOPPED | State: %-10s| Tick_L:%6ld Tick_R:%6ld | ADC:%4u | L_ms:%5u\n",
                       g_current_ir_state,
                       g_total_ticks_left,
                       g_total_ticks_right,
                       ir_read_single_sensor_raw(),
                       g_current_lost_time_ms); // <-- NEW
            }
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
    uint32_t last_temp_publish_ms = to_ms_since_boot(get_absolute_time());
    for (;;) {
        mqtt_client_poll(&mqtt_state); 
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (mqtt_client_is_online(&mqtt_state) && (now_ms - last_temp_publish_ms >= TEMP_PUBLISH_INTERVAL_MS)) {
            last_temp_publish_ms = now_ms;
            mqtt_client_publish_temperature(&mqtt_state);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// --- vLaunch (unchanged) ---
void vLaunch(void) {
    adc_init();
    adc_gpio_init(PIN_IR_RIGHT_ADC_GPIO);
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