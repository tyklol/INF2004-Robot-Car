#include <stdio.h>
#include <math.h> // <-- ADDED for atan2, sqrt, cos, sin
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

/*
 * This example reads and filters data from a GY-511 (LSM303DLHC)
 * and calculates Pitch, Roll, and tilt-compensated Yaw (Heading).
 */

// --- NEW: I2C/IMU Definitions ---
#define I2C_PORT i2c1
#define I2C_SDA_PIN 2 // (Check your wiring - using GP2)
#define I2C_SCL_PIN 3 // (Check your wiring - using GP3)
#define ACCEL_ADDR 0x19
#define MAG_ADDR 0x1E

// --- Filter Configuration ---
const float ALPHA = 0.2; // Smoothing factor (smaller = more smoothing)

// --- Global Variables for Raw and Filtered Data ---
int16_t raw_accel_data[3];
int16_t raw_mag_data[3];
float filtered_accel[3] = {0.0, 0.0, 0.0};
float filtered_mag[3] = {0.0, 0.0, 0.0};
float pitch_deg, roll_deg, yaw_deg;

// --- Function Prototypes ---
void imu_init();
void read_accel(int16_t accel[3]);
void read_mag(int16_t mag[3]);
void apply_ema_filter(int16_t raw_data[3], float filtered_data[3]);
void calculate_angles(float accel[3], float mag[3], float *pitch_out, float *roll_out, float *yaw_out);
float get_filtered_heading(int16_t raw_accel[3], int16_t raw_mag[3], float filtered_accel[3], float filtered_mag[3], 
    float *pitch_deg, float *roll_deg, float *yaw_deg);
float imu_get_yaw_rate(void);

// --- Main Program ---
int imu_main() {
    stdio_init_all();
    sleep_ms(2000);
    printf("Robotic Car Project - GY-511 IMU Test\n");
    printf("Pitch, Roll, and Tilt-Compensated Yaw\n");
    printf("--------------------------------------------------\n");

    // --- I2C Initialization ---
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    printf("I2C Initialized on GP%d (SDA) and GP%d (SCL)\n", I2C_SDA_PIN, I2C_SCL_PIN);

    // --- Sensor Initialization ---
    imu_init();
    printf("LSM303DLHC Initialized.\n\n");

    // --- Main Loop ---
    while (true) {
        // 1. Read raw data
        read_accel(raw_accel_data);
        read_mag(raw_mag_data);

        // 2. Apply filter
        apply_ema_filter(raw_accel_data, filtered_accel);
        apply_ema_filter(raw_mag_data, filtered_mag);

        // 3. Calculate angles
        calculate_angles(filtered_accel, filtered_mag, &pitch_deg, &roll_deg, &yaw_deg);

        // 4. Print all data in a clean block
        printf("--------------------------------------------------\n");

        printf("Raw Accel:     X: %-6d  Y: %-6d  Z: %-6d\n", 
               raw_accel_data[0], raw_accel_data[1], raw_accel_data[2]);
        printf("Filtered Accel:  X: %-6.0f  Y: %-6.0f  Z: %-6.0f\n\n", 
               filtered_accel[0], filtered_accel[1], filtered_accel[2]);
        
        printf("Raw Mag:       X: %-6d  Y: %-6d  Z: %-6d\n", 
               raw_mag_data[0], raw_mag_data[1], raw_mag_data[2]);
        printf("Filtered Mag:    X: %-6.0f  Y: %-6.0f  Z: %-6.0f\n\n", 
               filtered_mag[0], filtered_mag[1], filtered_mag[2]);

        printf("ANGLES:          Pitch: %-6.2f°  Roll: %-6.2f°  Yaw: %-6.2f°\n", 
               pitch_deg, roll_deg, yaw_deg);

        sleep_ms(500); // Slower update rate for readability
    }
    return 0;
}

// --- Function Definitions ---

/**
 * @brief Initializes the Accelerometer and Magnetometer
 */
void imu_init() {
    // I2C Init
    i2c_init(I2C_PORT, 100 * 1000); // 100kHz
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Sensor Init
    uint8_t buf[2];
    // Init Accelerometer (25 Hz, all axes)
    buf[0] = 0x20;
    buf[1] = 0x27;
    i2c_write_blocking(I2C_PORT, ACCEL_ADDR, buf, 2, false);
    // Init Magnetometer (Continuous conversion)
    buf[0] = 0x02;
    buf[1] = 0x00;
    i2c_write_blocking(I2C_PORT, MAG_ADDR, buf, 2, false);
}
/**
 * @brief Reads 6 bytes (X, Y, Z) from the Accelerometer
 */
void read_accel(int16_t accel[3]) {
    uint8_t data[6];
    uint8_t reg = 0xA8; // OUT_X_L_A (28h) with auto-increment
    i2c_write_blocking(I2C_PORT, ACCEL_ADDR, &reg, 1, true); 
    i2c_read_blocking(I2C_PORT, ACCEL_ADDR, data, 6, false);
    accel[0] = (int16_t)((data[1] << 8) | data[0]);
    accel[1] = (int16_t)((data[3] << 8) | data[2]);
    accel[2] = (int16_t)((data[5] << 8) | data[4]);
}

/**
 * @brief Reads 6 bytes (X, Y, Z) from the Magnetometer
 */
void read_mag(int16_t mag[3]) {
    uint8_t data[6];
    uint8_t reg = 0x03; // OUT_X_H_M (03h)
    i2c_write_blocking(I2C_PORT, MAG_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MAG_ADDR, data, 6, false);
    mag[0] = (int16_t)((data[0] << 8) | data[1]); // X
    mag[1] = (int16_t)((data[4] << 8) | data[5]); // Y
    mag[2] = (int16_t)((data[2] << 8) | data[3]); // Z
}

/**
 * @brief Applies an Exponential Moving Average (EMA) low-pass filter
 */
void apply_ema_filter(int16_t raw_data[3], float filtered_data[3]) {
    for (int i = 0; i < 3; i++) {
        filtered_data[i] = (ALPHA * (float)raw_data[i]) + ((1.0 - ALPHA) * filtered_data[i]);
    }
}

/**
 * @brief NEW: Calculates pitch, roll, and yaw from filtered sensor data.
 * * @param accel Filtered accelerometer data [X, Y, Z]
 * @param mag   Filtered magnetometer data [X, Y, Z]
 * @param pitch_out Pointer to store the calculated pitch in degrees
 * @param roll_out  Pointer to store the calculated roll in degrees
 * @param yaw_out   Pointer to store the calculated yaw in degrees
 */
void calculate_angles(float accel[3], float mag[3], float *pitch_out, float *roll_out, float *yaw_out) {
    
    // Get local copies of filtered data
    float ax = accel[0];
    float ay = accel[1];
    float az = accel[2];
    float mx = mag[0];
    float my = mag[1];
    float mz = mag[2];

    // --- 1. Calculate Pitch and Roll from Accelerometer ---
    float roll_rad = atan2(ay, az);
    float pitch_rad = atan2(-ax, sqrt(ay * ay + az * az));

    // --- 2. Calculate Tilt-Compensated Yaw (Heading) ---
    float mag_y_comp = my * cos(roll_rad) - mz * sin(roll_rad);
    float mag_x_comp = mx * cos(pitch_rad) + my * sin(roll_rad) * sin(pitch_rad) + mz * cos(roll_rad) * sin(pitch_rad);

    // --- 3. Calculate Yaw in Radians ---
    float yaw_rad = atan2(mag_y_comp, mag_x_comp);

    // --- 4. Convert all angles from Radians to Degrees ---
    *pitch_out = pitch_rad * (180.0 / M_PI);
    *roll_out = roll_rad * (180.0 / M_PI);
    *yaw_out = yaw_rad * (180.0 / M_PI);

    // --- 5. (Improvement) Normalize Yaw to 0-360 degrees ---
    if (*yaw_out < 0) {
        *yaw_out += 360.0;
    }
}
/**
 * @brief Helper function to get the current filtered heading
 * @return float Current yaw/heading (0-360 degrees)
 */
float get_filtered_heading(int16_t raw_accel[3], int16_t raw_mag[3], float filtered_accel[3], float filtered_mag[3], 
    float *pitch_deg, float *roll_deg, float *yaw_deg) {
    // 1. Read
    read_accel(raw_accel);
    read_mag(raw_mag);

    // 2. Filter
    apply_ema_filter(raw_accel, filtered_accel);
    apply_ema_filter(raw_mag, filtered_mag);

    // 3. Calculate
    calculate_angles(filtered_accel, filtered_mag, pitch_deg, roll_deg, yaw_deg);

    return *yaw_deg;
}
/**
 * @brief Get current yaw rate (angular velocity around Z-axis)
 * @return float Yaw rate in degrees per second
 */
float imu_get_yaw_rate(void) {
    // The gyroscope Z-axis gives rotation rate
    // LSM303DLHC doesn't have gyro - it only has accel + mag
    // If you're using LSM303DLHC, you need to calculate yaw rate from heading changes
    
    static float last_yaw = 0.0f;
    static absolute_time_t last_time;
    static bool first_call = true;
    
    if (first_call) {
        last_yaw = yaw_deg;
        last_time = get_absolute_time();
        first_call = false;
        return 0.0f;
    }
    
    absolute_time_t now = get_absolute_time();
    float dt_s = absolute_time_diff_us(last_time, now) / 1e6f;
    if (dt_s <= 0.0f || dt_s > 0.1f) return 0.0f;
    
    // Calculate yaw rate from heading change
    float yaw_diff = yaw_deg - last_yaw;
    
    // Handle wraparound (360° -> 0° transition)
    if (yaw_diff > 180.0f) yaw_diff -= 360.0f;
    if (yaw_diff < -180.0f) yaw_diff += 360.0f;
    
    float yaw_rate = yaw_diff / dt_s;  // degrees per second
    
    last_yaw = yaw_deg;
    last_time = now;
    
    return yaw_rate;
}