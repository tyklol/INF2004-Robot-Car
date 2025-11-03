#include <stdio.h>
#include <math.h> // <-- Included for math functions
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"     // <-- NEW: Need ADC for calibration
#include "pico/cyw43_arch.h" // Needed for network task & LED

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

// Your driver includes
#include "drivers/motor.h"
#include "drivers/ir.h"
#include "drivers/encoder.h"     // <-- Includes new polling function
#include "drivers/mqtt.h"

// --- Pin Definitions for Buttons ---
#define BUTTON_MODE_CALIBRATE  21 // GP21 (Calibration Step)
#define BUTTON_MODE_START_STOP 20 // GP20 (Line Follow Toggle)

// --- NEW: ADC Pin for Calibration ---
#define PIN_IR_RIGHT_ADC_GPIO   26      // GP26 = ADC0
#define PIN_IR_RIGHT_ADC_CH     0
#define ADC_SMOOTH_SAMPLES      12

// --- Settings ---
#define DEBOUNCE_TIME_US   150000 // 150 milliseconds
#define PRINT_DELAY_MS     100    // Print values every 100ms
#define CONTROL_LOOP_MS    10     // Run the *entire* control loop every 10ms (100Hz)
#define TEMP_PUBLISH_INTERVAL_MS 1000 // Publish temp every 1 second

// --- ROBOT TUNING PARAMETERS (!!! TUNE THESE !!!) ---
#define LINE_FOLLOW_SPEED  30.0f  // INCREASED from 20.0f - Target speed (tps) for line following.

// --- ROBOT PHYSICAL PARAMETERS (!!! MEASURE AND TUNE THESE !!!) ---
#define TICKS_PER_REVOLUTION    20.0f
#define WHEEL_DIAMETER_MM       65.0f
#define WHEEL_CIRCUMFERENCE_MM  (3.14159265f * WHEEL_DIAMETER_MM)

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
    LINE_FOLLOW_SINGLE
} RobotState;

// --- PID Controller ---
typedef struct {
    float kp, ki, kd, integral, last_error, min_output, max_output;
} pid_state_t;

static pid_state_t pid_motor_left;
static pid_state_t pid_motor_right;

// --- Helper Functions ---
static uint16_t adc_read_avg(uint ch, int samples) {
    adc_select_input(ch);
    uint32_t acc=0;
    for (int i=0; i<samples; i++) acc += adc_read();
    return (uint16_t)(acc / samples);
}

void pid_init(pid_state_t *pid, float kp, float ki, float kd, float min_out, float max_out) {
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
    pid->integral = 0.0f; pid->last_error = 0.0f;
    pid->min_output = min_out; pid->max_output = max_out;
}

float pid_calculate(pid_state_t *pid, float target, float current, float dt_s) {
    float error = target - current;
    float p_term = pid->kp * error;
    pid->integral += error * dt_s;
    if (pid->integral > 50.0f) pid->integral = 50.0f;
    if (pid->integral < -50.0f) pid->integral = -50.0f;
    float i_term = pid->ki * pid->integral;
    float derivative = (error - pid->last_error) / dt_s;
    pid->last_error = error;
    float d_term = pid->kd * derivative;
    float output = p_term + i_term + d_term;
    if (output > pid->max_output) output = pid->max_output;
    if (output < pid->min_output) output = pid->min_output;
    return output;
}

// --- Shared Global State ---
volatile RobotState current_state = STATIONARY_NOT_CALIBRATED;
volatile bool needs_update = true;
volatile uint32_t last_button_press_time = 0;
volatile uint16_t g_calib_white = 0;
volatile uint16_t g_calib_black = 0;
volatile bool g_is_calibrated = false;
volatile float g_speed_left_rpm = 0.0f;
volatile float g_speed_right_rpm = 0.0f;
volatile float g_distance_travelled_mm = 0.0f;
volatile float g_target_speed_left_tps = 0.0f;
volatile float g_target_speed_right_tps = 0.0f;
volatile uint16_t g_sensor_R = 0;
volatile const char *g_current_ir_state = "IDLE";
uint32_t g_last_print_time = 0;
static MQTT_CLIENT_DATA_T mqtt_state;

// --- NEW: Add these globals for diagnostics ---
volatile float g_current_speed_left_tps = 0.0f;
volatile float g_current_speed_right_tps = 0.0f;
volatile float g_final_power_left = 0.0f;
volatile float g_final_power_right = 0.0f;

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
 */
void gpio_interrupt_handler(uint gpio, uint32_t events) {
    // --- This handler now ONLY handles buttons ---
    if (gpio == BUTTON_MODE_CALIBRATE || gpio == BUTTON_MODE_START_STOP) {
        // Debounce logic
        uint32_t now = time_us_32();
        if (now - last_button_press_time < DEBOUNCE_TIME_US) {
            return;
        }
        last_button_press_time = now;

        // Handle button logic
        if (gpio == BUTTON_MODE_CALIBRATE) {
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
 */
void main_control_task(__unused void *params) {
    vTaskDelay(pdMS_TO_TICKS(500)); // Wait for network task to init CYW43
    printf("Main control task started.\n");

    const float dt_s = (float)CONTROL_LOOP_MS / 1000.0f;

    float Kp = 1.5f, Ki = 0.5f, Kd = 0.0f;
    pid_init(&pid_motor_left, Kp, Ki, Kd, -100.0f, 100.0f);
    pid_init(&pid_motor_right, Kp, Ki, Kd, -100.0f, 100.0f);

    for (;;) {
        // --- NEW: Poll encoders at the top of the loop ---
        int32_t delta_ticks_left;
        int32_t delta_ticks_right;
        encoder_update_and_get_ticks(&delta_ticks_left, &delta_ticks_right);
        // --- End of new polling ---

        // --- 1. Handle State Change Requests ---
        if (needs_update) {
            needs_update = false;
            ir_reset_pid_state();
            pid_init(&pid_motor_left, Kp, Ki, Kd, -100.0f, 100.0f);
            pid_init(&pid_motor_right, Kp, Ki, Kd, -100.0f, 100.0f);
            
            encoder_reset_total_ticks(); // <-- Use new reset function
            g_distance_travelled_mm = 0.0f;

            switch (current_state) {
                case STATIONARY_NOT_CALIBRATED:
                    g_current_ir_state = "NO_CAL";
                    printf("Mode: NOT CALIBRATED. Press GP21 to start.\n");
                    break;
                case CALIBRATE_WHITE_START:
                    g_current_ir_state = "CAL_WHITE";
                    blink_led(1, 100);
                    printf("Mode: Calibrating WHITE. Place sensor on WHITE, press GP21.\n");
                    break;
                case CALIBRATE_WHITE_END:
                    g_current_ir_state = "CAL_WHITE";
                    blink_led(2, 500);
                    printf("Mode: Calibrating WHITE. Place sensor on BLACK, press GP21.\n");
                    break;
                case CALIBRATE_BLACK_START:
                    g_calib_white = adc_read_avg(PIN_IR_RIGHT_ADC_CH, ADC_SMOOTH_SAMPLES);
                    g_current_ir_state = "CAL_BLACK";
                    blink_led(2, 100);
                    printf("WHITE saved: %u. Place sensor on BLACK, press GP21.\n", g_calib_white);
                    break;
                case CALIBRATED_STOPPED:
                    g_calib_black = adc_read_avg(PIN_IR_RIGHT_ADC_CH, ADC_SMOOTH_SAMPLES);
                    ir_set_calibration_values(g_calib_white, g_calib_black);
                    g_is_calibrated = true;
                    g_current_ir_state = "CAL_STOP";
                    blink_led(2, 500);
                    printf("BLACK saved: %u. CALIBRATED! Press GP20 to start/stop.\n", g_calib_black);
                    printf("Calibration Range: %d (should be > 500)\n", (int)(g_calib_black - g_calib_white));
                    break;
                case LINE_FOLLOW_SINGLE:
                    g_current_ir_state = "S_IDLE";
                    printf("Mode: LINE FOLLOW (Single Sensor PID)\n");
                    break;
            }
        }

        // --- 2. STAGE 1: Calculate Target Speeds (Outer Loop) ---
        if (current_state == LINE_FOLLOW_SINGLE) {
            ir_follow_line_single(LINE_FOLLOW_SPEED,
                                  (float *)&g_target_speed_left_tps, 
                                  (float *)&g_target_speed_right_tps,
                                  (uint16_t *)&g_sensor_R,
                                  (const char **)&g_current_ir_state);
        } else {
            g_target_speed_left_tps = 0.0f;
            g_target_speed_right_tps = 0.0f;
        }

        // --- 3. STAGE 2: Run Motor Speed PIDs (Inner Loop) ---
        
        // We already got delta_ticks from the poll at the top
        float current_speed_left_tps = (float)delta_ticks_left / dt_s;
        float current_speed_right_tps = (float)delta_ticks_right / dt_s;
        
        // Update telemetry
        g_speed_left_rpm = (current_speed_left_tps / TICKS_PER_REVOLUTION) * 60.0f;
        g_speed_right_rpm = (current_speed_right_tps / TICKS_PER_REVOLUTION) * 60.0f;
        
        // Store for printing
        g_current_speed_left_tps = current_speed_left_tps;
        g_current_speed_right_tps = current_speed_right_tps;
        
        // Get total ticks for distance calculation
        int32_t total_ticks_left = encoder_get_total_ticks_left();
        int32_t total_ticks_right = encoder_get_total_ticks_right();
        float avg_total_ticks = (total_ticks_left + total_ticks_right) / 2.0f;
        g_distance_travelled_mm = (avg_total_ticks / TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE_MM;

        // Calculate final power
        float final_power_left = pid_calculate(&pid_motor_left, 
                                               g_target_speed_left_tps, 
                                               current_speed_left_tps, 
                                               dt_s);
                                               
        float final_power_right = pid_calculate(&pid_motor_right,
                                                g_target_speed_right_tps,
                                                current_speed_right_tps,
                                                dt_s);
        
        // Store for printing
        g_final_power_left = final_power_left;
        g_final_power_right = final_power_right;
        
        // --- 4. STAGE 3: Actuate Motors ---
        motor_set_speed(MOTOR_LEFT, final_power_left);
        motor_set_speed(MOTOR_RIGHT, final_power_right);

        // --- 5. Non-blocking print ---
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - g_last_print_time > PRINT_DELAY_MS) {
            g_last_print_time = now_ms;

            if (current_state == LINE_FOLLOW_SINGLE) {
                float normalized = 0.0f;
                if (g_calib_black != g_calib_white) {
                    normalized = (float)(g_sensor_R - g_calib_white) / (float)(g_calib_black - g_calib_white);
                }
                
                printf("LINE: %-9s | Tgt_L:%6.1f Cur_L:%6.1f Pwr_L:%6.1f | Tgt_R:%6.1f Cur_R:%6.1f Pwr_R:%6.1f | ADC:%4u Norm:%.2f\n",
                    g_current_ir_state,
                    g_target_speed_left_tps, g_current_speed_left_tps, g_final_power_left,
                    g_target_speed_right_tps, g_current_speed_right_tps, g_final_power_right,
                    g_sensor_R, normalized);
            } else if (current_state == CALIBRATE_WHITE_START || current_state == CALIBRATE_BLACK_START) {
                g_sensor_R = adc_read_avg(PIN_IR_RIGHT_ADC_CH, ADC_SMOOTH_SAMPLES);
                printf("CALIBRATING... State: %s | Raw ADC: %u\n", g_current_ir_state, g_sensor_R);
            } else {
                 printf("STOPPED | State: %s | Dist: %.0f mm | L_RPM: %.0f | R_RPM: %.0f\n",
                    g_current_ir_state, g_distance_travelled_mm, g_speed_left_rpm, g_speed_right_rpm);
            }
        }

        // --- 6. Yield to other tasks ---
        vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_MS));
    }
}

/**
 * @brief TASK 2: Handles all Wi-Fi and MQTT communication.
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
 */
void vLaunch(void) {
    // --- 1. Initialize Hardware ---
    adc_init();
    adc_gpio_init(PIN_IR_RIGHT_ADC_GPIO); // GP26

    motor_init();
    ir_init();       
    encoder_init();  // <-- This now ONLY sets pin directions
    
    // --- 2. Setup Buttons ---
    gpio_init(BUTTON_MODE_CALIBRATE);
    gpio_set_dir(BUTTON_MODE_CALIBRATE, GPIO_IN);
    gpio_pull_up(BUTTON_MODE_CALIBRATE);
    gpio_init(BUTTON_MODE_START_STOP);
    gpio_set_dir(BUTTON_MODE_START_STOP, GPIO_IN);
    gpio_pull_up(BUTTON_MODE_START_STOP);

    // --- 3. Setup Interrupts (BUTTONS ONLY) ---
    // This single callback will handle all 4 pins
    gpio_set_irq_enabled_with_callback(BUTTON_MODE_CALIBRATE, GPIO_IRQ_EDGE_FALL, true, &gpio_interrupt_handler);
    gpio_set_irq_enabled(BUTTON_MODE_START_STOP, GPIO_IRQ_EDGE_FALL, true);
    
    // --- NO IRQ FOR ENCODERS ---

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
 */
int main() {
    stdio_init_all();
    // --- FIX: Restore the 3-second (3000ms) delay ---
    sleep_ms(3000); // Wait for serial monitor to connect
    printf("Robotic Car Program Started!\n");
    printf("WIFI_SSID: %s\n", WIFI_SSID);
    printf("MQTT_SERVER: %s\n", MQTT_SERVER);

    vLaunch();

    for (;;); // Unreachable
    return 0;
}