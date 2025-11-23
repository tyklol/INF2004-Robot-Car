#ifndef SYS_CONFIG_H
#define SYS_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

// ==========================================
// PIN DEFINITIONS
// ==========================================

// --- Motors ---
#define MOTOR_LEFT_FWD_PIN   9
#define MOTOR_LEFT_REV_PIN   8
#define MOTOR_RIGHT_FWD_PIN  10
#define MOTOR_RIGHT_REV_PIN  11

// --- Encoders ---
#define PIN_ENC_L 7 
#define PIN_ENC_R 0 

// --- IR Sensors ---
#define PIN_IR_RIGHT_ADC_GPIO 26
#define PIN_IR_RIGHT_ADC_CH   0

// --- Ultrasonic & Servo ---
#define TRIG_PIN  5
#define ECHO_PIN  4
#define SERVO_PIN 15

// --- Buttons ---
#define BUTTON_MODE_CALIBRATE   21
#define BUTTON_MODE_START_STOP  20 

// --- I2C (IMU) ---
#define I2C_PORT        i2c1
#define I2C_SDA_PIN     2 
#define I2C_SCL_PIN     3 

// --- LEDs (WS2812) ---
#define WS2812_PIN      18
#define NUM_PIXELS      2
#define WS2812_FREQ     800000

// ==========================================
// SYSTEM CONSTANTS
// ==========================================
#define DEBOUNCE_TIME_US              200000 
#define PRINT_DELAY_MS                100    
#define CONTROL_LOOP_MS               10     
#define TELEMETRY_PUBLISH_INTERVAL_MS 200
#define TOTAL_SCENARIOS               4

// ==========================================
// SHARED STATE ENUMS
// ==========================================
typedef enum {
    STATIONARY_NOT_CALIBRATED,
    CALIBRATE_WHITE_WAIT,      
    CALIBRATE_WHITE_SAMPLING,  
    CALIBRATE_BLACK_WAIT,      
    CALIBRATE_BLACK_SAMPLING,  
    CALIBRATED_STOPPED,    
} RobotState;

#endif // SYS_CONFIG_H