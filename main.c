#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "drivers/motor.h"
// #include "drivers/ir.h" // Original include
#include "drivers/ir.c"   // UPDATED INCLUDE PATH AS REQUESTED
#include "drivers/mqtt_client.h"

// --- Pin Definitions for Buttons ---
#define BUTTON_MODE_DUAL    21  // GP21
#define BUTTON_MODE_SINGLE  20  // GP20

// --- Settings ---
#define DEBOUNCE_TIME_US    150000 // 150 milliseconds
#define BASE_SPEED_PERCENT  25.0f  // Keep slow for tuning
#define PRINT_DELAY_MS      100    // Print values every 100ms
#define TEMP_PUBLISH_INTERVAL_MS 10000u // 10 seconds

// --- State Machine ---
typedef enum {
    STATIONARY,
    LINE_FOLLOW_DOUBLE,
    LINE_FOLLOW_SINGLE
} RobotState;

volatile RobotState current_state = STATIONARY;
volatile bool needs_update = true;
volatile uint32_t last_button_press_time = 0;

/**
 * @brief This function is called every time a button pin changes state
 */
void gpio_interrupt_handler(uint gpio, uint32_t events) {
    uint32_t now = time_us_32();

    if (now - last_button_press_time < DEBOUNCE_TIME_US) {
        return;
    }
    last_button_press_time = now;

    if (gpio == BUTTON_MODE_DUAL) {
        current_state = (current_state == STATIONARY) ? LINE_FOLLOW_DOUBLE : STATIONARY;
    } else if (gpio == BUTTON_MODE_SINGLE) {
        current_state = (current_state == STATIONARY) ? LINE_FOLLOW_SINGLE : STATIONARY;
    }

    needs_update = true;
}

// Main application entry point
int main() {
    stdio_init_all();
    sleep_ms(5000);
    printf("Robotic Car Program Started!\n");

    printf("WIFI_SSID: %s\n", WIFI_SSID);
    printf("WIFI_PASSWORD: %s\n", WIFI_PASSWORD);
    printf("MQTT_SERVER: %s\n", MQTT_SERVER);

    printf("Initializing drivers...\n");
    motor_init();
    ir_init();
    // Uncomment to test out MQTT
    // static MQTT_CLIENT_DATA_T state;
    // mqtt_client_init(&state);
    // while (!mqtt_client_try_wifi_connect(&state, 10000)) {
    //     printf("WiFi connection failed, retrying...\n");
    //     sleep_ms(2000);
    // }
    // mqtt_client_start_mqtt(&state); // start DNS / mqtt
    printf("Drivers initialized.\n");

    // --- setup buttons ---
    gpio_init(BUTTON_MODE_DUAL);
    gpio_set_dir(BUTTON_MODE_DUAL, GPIO_IN);
    gpio_pull_up(BUTTON_MODE_DUAL);

    gpio_init(BUTTON_MODE_SINGLE);
    gpio_set_dir(BUTTON_MODE_SINGLE, GPIO_IN);
    gpio_pull_up(BUTTON_MODE_SINGLE);

    // --- setup interrupts ---
    gpio_set_irq_enabled_with_callback(BUTTON_MODE_DUAL, GPIO_IRQ_EDGE_FALL, true, &gpio_interrupt_handler);
    gpio_set_irq_enabled(BUTTON_MODE_SINGLE, GPIO_IRQ_EDGE_FALL, true);

    motor_set_speed(MOTOR_LEFT, 0.0f);
    motor_set_speed(MOTOR_RIGHT, 0.0f);
    printf("Mode: STATIONARY. Press GP21 for Dual-Sensor, GP20 for Single-Sensor.\n");

    needs_update = false;

    // --- Variables for printing ---
    float left_power = 0.0f;
    float right_power = 0.0f;
    uint16_t sensor_L = 0;
    uint16_t sensor_R = 0;
    const char *current_ir_state = "IDLE";
    uint32_t last_print_time = 0;
    // ----------------------------------------
    uint32_t last_temp_publish_ms = to_ms_since_boot(get_absolute_time());
    // --- Main Program Loop ---
    while (true) {

        if (needs_update) {
            needs_update = false;

            ir_reset_pid_state(); // Reset PID memory on mode change

            if (current_state == STATIONARY) {
                motor_set_speed(MOTOR_LEFT, 0.0f);
                motor_set_speed(MOTOR_RIGHT, 0.0f);
                left_power = 0.0f;
                right_power = 0.0f;
                current_ir_state = "IDLE";
                printf("Mode: STATIONARY\n");
            } else if (current_state == LINE_FOLLOW_DOUBLE) {
                printf("Mode: LINE FOLLOW (Dual Sensor)\n");
            } else if (current_state == LINE_FOLLOW_SINGLE) {
                current_ir_state = "S_IDLE";
                printf("Mode: LINE FOLLOW (Single Sensor PID)\n");
            }
        }

        // --- Continuous Actions based on state ---
        if (current_state == LINE_FOLLOW_DOUBLE) {
            ir_follow_line_double(BASE_SPEED_PERCENT,
                                  &left_power, &right_power,
                                  &sensor_L, &sensor_R,
                                  &current_ir_state);
        }
        else if (current_state == LINE_FOLLOW_SINGLE) {
            ir_follow_line_single(BASE_SPEED_PERCENT,
                                  &left_power, &right_power,
                                  &sensor_L,
                                  &current_ir_state);
            sensor_R = 0; // Not used in this mode
        }

        // --- Non-blocking print ---
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - last_print_time > PRINT_DELAY_MS) {

            if (current_state == STATIONARY) {
                sensor_L = adc_read_avg(PIN_IR_LEFT_ADC_CH, ADC_SMOOTH_SAMPLES);
                sensor_R = adc_read_avg(PIN_IR_RIGHT_ADC_CH, ADC_SMOOTH_SAMPLES);
                printf("STATIONARY | L_ADC: %4u | R_ADC: %4u\n", sensor_L, sensor_R);
            }
            else if (current_state == LINE_FOLLOW_DOUBLE) {
                printf("%-9s | L: %4u | R: %4u | Pwr_L: %4.1f%% | Pwr_R: %4.1f%%\n",
                        current_ir_state, sensor_L, sensor_R, left_power, right_power);
            } else if (current_state == LINE_FOLLOW_SINGLE) {
                printf("%-9s | L: %4u | Pwr_L: %4.1f%% | Pwr_R: %4.1f%%\n",
                        current_ir_state, sensor_L, left_power, right_power);
            }
            last_print_time = now_ms;
        }

        // mqtt_client_poll(&state);
        // // Only publish temperature at most once per interval and only when MQTT is online
        // uint32_t now_ms2 = to_ms_since_boot(get_absolute_time());
        // if (mqtt_client_is_online(&state) && (now_ms2 - last_temp_publish_ms >= TEMP_PUBLISH_INTERVAL_MS)) {
        //     last_temp_publish_ms = now_ms2;
        //     mqtt_client_publish_temperature(&state);
        // }
        sleep_ms(5);
    }
}