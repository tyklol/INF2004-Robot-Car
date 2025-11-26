#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "imu.h" // Include the new header

// --- I2C/IMU Definitions ---
#define I2C_PORT i2c1
#define I2C_SDA_PIN 2 
#define I2C_SCL_PIN 3 
#define ACCEL_ADDR 0x19
#define MAG_ADDR 0x1E

const float ALPHA = 0.3; 

// Internal Globals
int16_t raw_accel_data[3];
int16_t raw_mag_data[3];
float filtered_accel[3] = {0.0, 0.0, 0.0};
float filtered_mag[3] = {0.0, 0.0, 0.0};
float pitch_deg, roll_deg, yaw_deg;

// --- Helper Prototypes ---
static void read_accel(int16_t accel[3]);
static void read_mag(int16_t mag[3]);
static void apply_ema_filter(int16_t raw_data[3], float filtered_data[3]);
static void calculate_angles(float accel[3], float mag[3], float *pitch_out, float *roll_out, float *yaw_out);

// --- Public API ---

void imu_init(void) {
    // I2C Init
    i2c_init(I2C_PORT, 100 * 1000); 
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    uint8_t buf[2];
    // Init Accelerometer
    buf[0] = 0x20; buf[1] = 0x27;
    i2c_write_blocking(I2C_PORT, ACCEL_ADDR, buf, 2, false);
    // Init Magnetometer
    buf[0] = 0x02; buf[1] = 0x00;
    i2c_write_blocking(I2C_PORT, MAG_ADDR, buf, 2, false);
    
    printf("IMU Initialized.\n");
}

float imu_get_heading(void) {
    // 1. Read Raw
    read_accel(raw_accel_data);
    read_mag(raw_mag_data);

    // 2. Filter
    apply_ema_filter(raw_accel_data, filtered_accel);
    apply_ema_filter(raw_mag_data, filtered_mag);

    // 3. Calculate
    calculate_angles(filtered_accel, filtered_mag, &pitch_deg, &roll_deg, &yaw_deg);
    
    return yaw_deg;
}

// --- Internal Helpers (Same as your previous code) ---

static void read_accel(int16_t accel[3]) {
    uint8_t data[6];
    uint8_t reg = 0xA8; 
    i2c_write_blocking(I2C_PORT, ACCEL_ADDR, &reg, 1, true); 
    i2c_read_blocking(I2C_PORT, ACCEL_ADDR, data, 6, false);
    accel[0] = (int16_t)((data[1] << 8) | data[0]);
    accel[1] = (int16_t)((data[3] << 8) | data[2]);
    accel[2] = (int16_t)((data[5] << 8) | data[4]);
}

static void read_mag(int16_t mag[3]) {
    uint8_t data[6];
    uint8_t reg = 0x03; 
    i2c_write_blocking(I2C_PORT, MAG_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MAG_ADDR, data, 6, false);
    mag[0] = (int16_t)((data[0] << 8) | data[1]); // X
    mag[1] = (int16_t)((data[4] << 8) | data[5]); // Y
    mag[2] = (int16_t)((data[2] << 8) | data[3]); // Z
}

static void apply_ema_filter(int16_t raw_data[3], float filtered_data[3]) {
    for (int i = 0; i < 3; i++) {
        filtered_data[i] = (ALPHA * (float)raw_data[i]) + ((1.0 - ALPHA) * filtered_data[i]);
    }
}

static void calculate_angles(float accel[3], float mag[3], float *pitch_out, float *roll_out, float *yaw_out) {
    float ax = accel[0], ay = accel[1], az = accel[2];
    float mx = mag[0], my = mag[1], mz = mag[2];

    float roll_rad = atan2(ay, az);
    float pitch_rad = atan2(-ax, sqrt(ay * ay + az * az));

    float mag_y_comp = my * cos(roll_rad) - mz * sin(roll_rad);
    float mag_x_comp = mx * cos(pitch_rad) + my * sin(roll_rad) * sin(pitch_rad) + mz * cos(roll_rad) * sin(pitch_rad);

    float yaw_rad = atan2(mag_y_comp, mag_x_comp);

    *pitch_out = pitch_rad * (180.0 / M_PI);
    *roll_out = roll_rad * (180.0 / M_PI);
    *yaw_out = yaw_rad * (180.0 / M_PI);

    if (*yaw_out < 0) *yaw_out += 360.0;
}