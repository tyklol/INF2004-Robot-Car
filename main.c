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
#include "drivers/ir.h"       // Include the IR sensor driver
#include "drivers/mqtt.h"     // Kept for network task
#include "drivers/encoder.h"  // For polling total ticks

// --- Pin Definitions for Buttons ---
#define BUTTON_MODE_CALIBRATE  21 // GP21 (Calibration Step)
#define BUTTON_MODE_START_STOP 20 // GP20 (Line Follow Toggle)

// --- ADC Pin for Calibration ---
#define PIN_IR_RIGHT_ADC_GPIO    26      // GP26 = ADC0
#define PIN_IR_RIGHT_ADC_CH      0
#define ADC_SMOOTH_SAMPLES       12

// --- Settings ---
#define DEBOUNCE_TIME_US   150000 // 150 milliseconds
#define PRINT_DELAY_MS     100    // Print values every 100ms
#define CONTROL_LOOP_MS    10     // Run the *entire* control loop every 10ms (100Hz)
#define TEMP_PUBLISH_INTERVAL_MS 1000 // Publish temp every 1 second

// --- ROBOT TUNING PARAMETERS ---
#define LINE_FOLLOW_POWER  40.0f  // Base motor power (0-100).
                                  // Increased slightly to help with stalls.

// --- Task Priorities ---
#define MAIN_CONTROL_TASK_PRIORITY (tskIDLE_PRIORITY + 2UL)
#define NETWORK_TASK_PRIORITY      (tskIDLE_PRIORITY + 1UL)

// --- State Machine (Simplified Calibration) ---
typedef enum {
    STATIONARY_NOT_CALIBRATED,
    CALIBRATE_WHITE_START, // Waiting for user to press on white
    CALIBRATE_BLACK_START, // Waiting for user to press on black
    CALIBRATED_STOPPED,
    LINE_FOLLOW_SINGLE
} RobotState;

// --- Shared Global State ---
RobotState current_state = STATIONARY_NOT_CALIBRATED;
bool needs_update = true;
uint32_t last_button_press_time = 0;
uint16_t g_calib_white = 0;
uint16_t g_calib_black = 0;
bool g_is_calibrated = false;

float g_target_power_left = 0.0f;
float g_target_power_right = 0.0f;
uint16_t g_sensor_R = 0;
const char *g_current_ir_state = "IDLE";
uint32_t g_last_print_time = 0;

// --- NEW: Globals for TOTAL ticks ---
int32_t g_total_ticks_left = 0;
int32_t g_total_ticks_right = 0;

static MQTT_CLIENT_DATA_T mqtt_state;

// --- LED Blink ---
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
 * --- MODIFIED: Simplified calibration state flow ---
 */
void gpio_interrupt_handler(uint gpio, uint32_t events) {
    if (gpio == BUTTON_MODE_CALIBRATE || gpio == BUTTON_MODE_START_STOP) {
        uint32_t now = time_us_32();
        if (now - last_button_press_time < DEBOUNCE_TIME_US) {
            return;
        }
        last_button_press_time = now;

        if (gpio == BUTTON_MODE_CALIBRATE) {
            // --- NEW SIMPLIFIED FLOW ---
            if (current_state == STATIONARY_NOT_CALIBRATED) {
                current_state = CALIBRATE_WHITE_START;
            } else if (current_state == CALIBRATE_WHITE_START) {
                current_state = CALIBRATE_BLACK_START;
            } else if (current_state == CALIBRATE_BLACK_START) {
                current_state = CALIBRATED_STOPPED;
            } else if (current_state == CALIBRATED_STOPPED) {
                current_state = STATIONARY_NOT_CALIBRATED;
                g_is_calibrated = false;
            }
        } else if (gpio == BUTTON_MODE_START_STOP) {
            if (g_is_calibrated) {
                if (current_state == CALIBRATED_STOPPED) {
                    current_state = LINE_FOLLOW_SINGLE;
                } else if (current_state == LINE_FOLLOW_SINGLE) {
                    current_state = CALIBRATED_STOPPED;
                }
            }
        }
        needs_update = true;
    }
}

/**
 * @brief TASK 1: Handles all high-frequency robot control logic.
 * --- MODIFIED: Fixed Calibration Logic, Using TOTAL Ticks for Print ---
 */
void main_control_task(__unused void *params) {
    vTaskDelay(pdMS_TO_TICKS(500)); // Wait for network task to init CYW43
    printf("Main control task started (Fixed Calib + Total Ticks).\n");

    for (;;) {
        // --- Poll encoders to update internal state ---
        int32_t delta_ticks_left, delta_ticks_right;
        encoder_update_and_get_ticks(&delta_ticks_left, &delta_ticks_right);

        // --- Get TOTAL accumulated ticks for printing ---
        g_total_ticks_left = encoder_get_total_ticks_left();
        g_total_ticks_right = encoder_get_total_ticks_right();


        // --- Handle State Change Requests ---
        if (needs_update) {
            needs_update = false;
            ir_reset_pid_state(); // Reset IR PID terms
            
            // --- RESET TOTAL TICKS on any state change ---
            encoder_reset_total_ticks();
            g_total_ticks_left = 0;
            g_total_ticks_right = 0;
            
            // Reset target power
            g_target_power_left = 0.0f;
            g_target_power_right = 0.0f;

            // --- NEW SIMPLIFIED STATE LOGIC ---
            switch (current_state) {
                case STATIONARY_NOT_CALIBRATED:
                    g_current_ir_state = "NO_CAL";
                    printf("Mode: NOT CALIBRATED. Press GP21 to start.\n");
                    break;
                case CALIBRATE_WHITE_START:
                    g_current_ir_state = "CAL_WHITE";
                    blink_led(1, 100);
                    printf("Mode: Place sensor on WHITE, press GP21 to save.\n");
                    break;
                case CALIBRATE_BLACK_START:
                    // --- THIS IS THE FIX ---
                    // Save the WHITE value *after* user presses button
                    g_calib_white = ir_read_single_sensor_raw();
                    g_current_ir_state = "CAL_BLACK";
                    blink_led(2, 100);
                    printf("WHITE saved: %u. Place on BLACK, press GP21 to save.\n", g_calib_white);
                    break;
                case CALIBRATED_STOPPED:
                    // Save the BLACK value *after* user presses button
                    g_calib_black = ir_read_single_sensor_raw();
                    ir_set_calibration_values(g_calib_white, g_calib_black);
                    g_is_calibrated = true;
                    g_current_ir_state = "CAL_STOP";
                    blink_led(2, 500);
                    printf("BLACK saved: %u. CALIBRATED! Press GP20 to start/stop.\n", g_calib_black);
                    // This print is from main, it's just for info
                    printf("Calibration Range: %d (should be > 500)\n", (int)(g_calib_black - g_calib_white));
                    break;
                case LINE_FOLLOW_SINGLE:
                    g_current_ir_state = "LINE_FOLLOWING";
                    printf("Mode: Line Following\n");
                    break;
            }
        }

        // --- Calculate Target Motor Power (for Line Following) ---
        if (current_state == LINE_FOLLOW_SINGLE && g_is_calibrated) {
            ir_follow_line_single(LINE_FOLLOW_POWER,
                                  &g_target_power_left, 
                                  &g_target_power_right,
                                  &g_sensor_R,
                                  &g_current_ir_state);
        } else {
            g_target_power_left = 0.0f;
            g_target_power_right = 0.0f;
        }

        // --- Actuate Motors ---
        motor_set_speed(MOTOR_LEFT, g_target_power_left);
        motor_set_speed(MOTOR_RIGHT, g_target_power_right);

        // --- Print Debugging Information ---
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - g_last_print_time > PRINT_DELAY_MS) {
            g_last_print_time = now_ms;
            if (current_state == LINE_FOLLOW_SINGLE) {
                // --- MODIFIED PRINTF: Using TOTAL Ticks ---
                printf("LINE: %-9s | Pwr_L:%6.1f Pwr_R:%6.1f | Tick_L:%6ld Tick_R:%6ld | ADC:%4u\n",
                       g_current_ir_state,
                       g_target_power_left,
                       g_target_power_right,
                       g_total_ticks_left,  // <-- CHANGED
                       g_total_ticks_right, // <-- CHANGED
                       g_sensor_R);
            } else {
                 // --- MODIFIED PRINTF: Using TOTAL Ticks ---
                 printf("STOPPED | State: %-9s | Tick_L:%6ld Tick_R:%6ld | ADC:%4u\n",
                       g_current_ir_state,
                       g_total_ticks_left,  // <-- CHANGED
                       g_total_ticks_right, // <-- CHANGED
                       ir_read_single_sensor_raw());
            }
        }

        // Yield to other tasks
        vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_MS));
    }
}

/**
 * @brief TASK 2: Handles all Wi-Fi and MQTT communication.
 * (No changes)
 */
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

/**
 * @brief The "launcher" function.
 * (No changes from previous)
 */
void vLaunch(void) {
    // --- 1. Initialize Hardware ---
    adc_init();
    adc_gpio_init(PIN_IR_RIGHT_ADC_GPIO); // GP26

    motor_init();
    ir_init();      
    encoder_init(); 
    
    // --- 2. Setup Buttons ---
    gpio_init(BUTTON_MODE_CALIBRATE);
    gpio_set_dir(BUTTON_MODE_CALIBRATE, GPIO_IN);
    gpio_pull_up(BUTTON_MODE_CALIBRATE);
    gpio_init(BUTTON_MODE_START_STOP);
    gpio_set_dir(BUTTON_MODE_START_STOP, GPIO_IN);
    gpio_pull_up(BUTTON_MODE_START_STOP);

    // --- 3. Setup Interrupts (BUTTONS ONLY) ---
    gpio_set_irq_enabled_with_callback(BUTTON_MODE_CALIBRATE, GPIO_IRQ_EDGE_FALL, true, &gpio_interrupt_handler);
    gpio_set_irq_enabled(BUTTON_MODE_START_STOP, GPIO_IRQ_EDGE_FALL, true);
    
    // --- 4. Stop motors initially ---
    motor_set_speed(MOTOR_LEFT, 0.0f);
    motor_set_speed(MOTOR_RIGHT, 0.0f);
    printf("Mode: STATIONARY. Press GP21 to start calibration.\n");

    // --- 5. Create Tasks ---
    xTaskCreate(network_task, "NetworkTask", 2048, NULL, NETWORK_TASK_PRIORITY, NULL);
    xTaskCreate(main_control_task, "MainControlTask", 1024, NULL, MAIN_CONTROL_TASK_PRIORITY, NULL);

    // --- 6. Start the Scheduler ---
    printf("Starting FreeRTOS scheduler\n");
    vTaskStartScheduler();
}

/**
 * @brief The main() entry point.
 * (No changes)
 */
int main() {
    stdio_init_all();
    sleep_ms(3000); // Wait for serial monitor to connect
    printf("Robotic Car Program Started!\n");
    printf("WIFI_SSID: %s\n", WIFI_SSID);
    printf("MQTT_SERVER: %s\n", MQTT_SERVER);

    vLaunch();

    for (;;); // Unreachable
    return 0;
}