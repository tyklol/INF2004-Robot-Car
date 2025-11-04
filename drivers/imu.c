#include "imu.h"
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

// -------- I2C & LSM303DLHC Configuration --------
#define I2C_PORT      i2c1  
#define PIN_SDA       2
#define PIN_SCL       3

// Accelerometer registers
#define LSM303_ACC_ADDR   0x19
#define WHO_AM_I_A        0x0F
#define CTRL_REG1_A       0x20
#define CTRL_REG4_A       0x23
#define OUT_X_L_A         0x28

// Magnetometer registers
#define LSM303_MAG_ADDR   0x1E
#define CRA_REG_M         0x00
#define CRB_REG_M         0x01
#define MR_REG_M          0x02
#define OUT_X_H_M         0x03

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// -------- Internal State --------
// Accelerometer (X/Y only for 2D motion)
static float a_alpha = 0.12f;
static float ax_f = 0.0f, ay_f = 0.0f;  // Filtered X/Y accel
static bool  a_init = false;
static float a_bias_x = 0.0f, a_bias_y = 0.0f;  // DC offsets

// Magnetometer (X/Y/Z - Z needed for tilt compensation!)
static float m_bias_x = 0.0f, m_bias_y = 0.0f, m_bias_z = 0.0f;
static int16_t m_min_x = 32767, m_max_x = -32768;
static int16_t m_min_y = 32767, m_max_y = -32768;
static int16_t m_min_z = 32767, m_max_z = -32768;

// Latest pose
static imu_pose_t latest;

// -------- I2C Utilities --------
static inline void i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t b[2] = {reg, val};
    i2c_write_blocking(I2C_PORT, addr, b, 2, false);
}

static inline void i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t* dst, size_t n) {
    uint8_t r = reg;
    i2c_write_blocking(I2C_PORT, addr, &r, 1, true);
    i2c_read_blocking(I2C_PORT, addr, dst, n, false);
}

// -------- Initialization --------
bool imu_init_i2c0_gp0_gp1(void) {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);
    return true;
}

bool lsm303dlhc_init(void) {
    // Accel: 50Hz, XYZ enable, High-Res + BDU, ±2g
    i2c_write_reg(LSM303_ACC_ADDR, CTRL_REG1_A, 0x47);
    i2c_write_reg(LSM303_ACC_ADDR, CTRL_REG4_A, 0x88);
    sleep_ms(10);

    // Mag: 15 Hz, default gain, continuous mode
    i2c_write_reg(LSM303_MAG_ADDR, CRA_REG_M, 0x10);
    i2c_write_reg(LSM303_MAG_ADDR, CRB_REG_M, 0x20);
    i2c_write_reg(LSM303_MAG_ADDR, MR_REG_M,  0x00);
    sleep_ms(10);

    // WHO_AM_I check
    uint8_t who = 0;
    i2c_read_regs(LSM303_ACC_ADDR, WHO_AM_I_A, &who, 1);
    return (who == 0x33);
}

void imu_set_accel_ema_alpha(float alpha) {
    if (alpha > 0.0f && alpha < 1.0f) a_alpha = alpha;
}

void imu_set_mag_bias(float bx, float by, float bz) {
    m_bias_x = bx;
    m_bias_y = by;
    m_bias_z = bz;
}

// -------- Sensor Reading --------
static void read_accel_xy(int16_t* x12, int16_t* y12) {
    uint8_t raw[6] = {0};
    i2c_read_regs(LSM303_ACC_ADDR, OUT_X_L_A | 0x80, raw, 6);
    int16_t x16 = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t y16 = (int16_t)((raw[3] << 8) | raw[2]);
    *x12 = (x16 >> 4);
    *y12 = (y16 >> 4);
}

static void read_mag_raw(int16_t* mx, int16_t* my, int16_t* mz) {
    uint8_t raw[6] = {0};
    // LSM303DLHC order: X_H, X_L, Z_H, Z_L, Y_H, Y_L
    i2c_read_regs(LSM303_MAG_ADDR, OUT_X_H_M, raw, 6);
    int16_t x = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t z = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t y = (int16_t)((raw[4] << 8) | raw[5]);
    *mx = x;
    *my = y;
    *mz = z;
}

// -------- Calibration --------
void imu_calibrate_bias(void) {
    // --- Stage 1: Accelerometer (X/Y only) ---
    printf("--- ACCEL CALIBRATION (X/Y) ---\n");
    printf("Place robot on LEVEL surface. DO NOT MOVE.\n");
    printf("Starting in 3...\n"); sleep_ms(1000);
    printf("Starting in 2...\n"); sleep_ms(1000);
    printf("Starting in 1...\n"); sleep_ms(1000);
    printf("Calibrating...\n");

    const int N_ACCEL = 200;
    double sax = 0, say = 0;

    for (int i = 0; i < N_ACCEL; i++) {
        int16_t x, y;
        read_accel_xy(&x, &y);
        sax += x;
        say += y;
        sleep_ms(10);
    }
    
    a_bias_x = (float)(sax / N_ACCEL);
    a_bias_y = (float)(say / N_ACCEL);
    a_init = false;
    
    printf("  X_bias: %.2f, Y_bias: %.2f\n\n", a_bias_x, a_bias_y);
    sleep_ms(1000);

    // --- Stage 2: Magnetometer (X/Y/Z for full 3D heading) ---
    printf("--- MAG CALIBRATION (X/Y/Z) ---\n");
    printf("Pick up robot. Slowly rotate it on ALL axes:\n");
    printf("  - Spin horizontally (yaw)\n");
    printf("  - Flip end-over-end (pitch)\n");
    printf("  - Roll side-to-side\n");
    printf("Press GP21 button to start 20-second calibration...\n");

    const uint btn_pin = 21;
    gpio_init(btn_pin);
    gpio_set_dir(btn_pin, GPIO_IN);
    gpio_pull_up(btn_pin);

    // Wait for button press (active-low)
    while (gpio_get(btn_pin) != 0) sleep_ms(10);
    sleep_ms(50);
    while (gpio_get(btn_pin) == 0) sleep_ms(10);
    sleep_ms(50);

    printf("GO! Rotate slowly for 20 seconds...\n");
    
    m_min_x = 32767; m_max_x = -32768;
    m_min_y = 32767; m_max_y = -32768;
    m_min_z = 32767; m_max_z = -32768;

    uint32_t start_time = time_us_32();
    uint32_t cal_duration_ms = 20000;
    uint32_t last_print = 0;

    while ((time_us_32() - start_time) < (cal_duration_ms * 1000)) {
        int16_t mx, my, mz;
        read_mag_raw(&mx, &my, &mz);

        if (mx < m_min_x) m_min_x = mx;
        if (mx > m_max_x) m_max_x = mx;
        if (my < m_min_y) m_min_y = my;
        if (my > m_max_y) m_max_y = my;
        if (mz < m_min_z) m_min_z = mz;
        if (mz > m_max_z) m_max_z = mz;
        
        // Print countdown every second
        uint32_t elapsed_ms = (time_us_32() - start_time) / 1000;
        if (elapsed_ms - last_print >= 1000) {
            uint32_t remaining = (cal_duration_ms - elapsed_ms) / 1000;
            printf("  ... %lu seconds left ...\n", remaining);
            last_print = elapsed_ms;
        }
        
        sleep_ms(20);
    }

    printf("DONE!\n");

    // Calculate hard-iron offsets
    float bias_x = (float)(m_max_x + m_min_x) / 2.0f;
    float bias_y = (float)(m_max_y + m_min_y) / 2.0f;
    float bias_z = (float)(m_max_z + m_min_z) / 2.0f;

    imu_set_mag_bias(bias_x, bias_y, bias_z);
    
    printf("\nMagnetometer Calibration COMPLETE:\n");
    printf("  X_bias: %.1f  (range: %d to %d, span: %d)\n", 
           bias_x, m_min_x, m_max_x, m_max_x - m_min_x);
    printf("  Y_bias: %.1f  (range: %d to %d, span: %d)\n", 
           bias_y, m_min_y, m_max_y, m_max_y - m_min_y);
    printf("  Z_bias: %.1f  (range: %d to %d, span: %d)\n", 
           bias_z, m_min_z, m_max_z, m_max_z - m_min_z);
    printf("\nGood calibration: spans should be 400-1000+ each\n");
    printf("--------------------------------------------\n\n");
    sleep_ms(2000);
}

// -------- Filtering --------
static void accel_filter_push(float x, float y) {
    if (!a_init) {
        ax_f = x;
        ay_f = y;
        a_init = true;
    } else {
        ax_f = a_alpha * x + (1.0f - a_alpha) * ax_f;
        ay_f = a_alpha * y + (1.0f - a_alpha) * ay_f;
    }
}

// -------- Heading Calculation --------
/**
 * @brief Calculate heading with tilt compensation
 * Uses magnetometer X/Y/Z and accelerometer-derived tilt
 * 
 * For 2D robot on flat ground, roll/pitch are small but still matter
 * for accurate heading. Z-axis mag data is critical here.
 */
static float heading_tilt_comp(float mx, float my, float mz, float ax, float ay) {
    // Simplified tilt from accel (assumes mostly level, small tilts)
    // Roll ≈ atan2(ay, g) where g ≈ constant
    // Pitch ≈ atan2(-ax, g)
    // For small angles on flat ground, this is good enough
    
    const float g_nominal = 256.0f;  // Nominal gravity in 12-bit counts
    float roll_rad  = atan2f(ay, g_nominal);
    float pitch_rad = atan2f(-ax, g_nominal);
    
    // Tilt compensation rotation
    float cr = cosf(roll_rad);
    float sr = sinf(roll_rad);
    float cp = cosf(pitch_rad);
    float sp = sinf(pitch_rad);

    // Rotate mag into horizontal plane
    float mxh = mx * cp + mz * sp;
    float myh = mx * sr * sp + my * cr - mz * sr * cp;

    // Heading (0-360°)
    float heading = atan2f(myh, mxh) * 57.29578f;
    if (heading < 0.0f) heading += 360.0f;
    
    return heading;
}

// -------- Main Update Function --------
void imu_update(imu_pose_t* out) {
    int16_t ax_c, ay_c;
    int16_t mx_c, my_c, mz_c;

    // Read sensors
    read_accel_xy(&ax_c, &ay_c);
    read_mag_raw(&mx_c, &my_c, &mz_c);

    // Apply biases
    float ax = (float)ax_c - a_bias_x;
    float ay = (float)ay_c - a_bias_y;
    float mx = (float)mx_c - m_bias_x;
    float my = (float)my_c - m_bias_y;
    float mz = (float)mz_c - m_bias_z;

    // Filter accelerometer
    accel_filter_push(ax, ay);

    // Calculate heading with tilt compensation
    float yaw_deg = heading_tilt_comp(mx, my, mz, ax_f, ay_f);

    // Update latest pose
    latest.ax = ax_f;
    latest.ay = ay_f;
    latest.az = 0.0f;  // Not used for 2D motion
    latest.mx = mx;
    latest.my = my;
    latest.mz = mz;
    latest.roll_deg = 0.0f;   // Not critical for 2D
    latest.pitch_deg = 0.0f;  // Not critical for 2D
    latest.yaw_deg = yaw_deg;

    if (out) *out = latest;
}

void imu_get_latest(imu_pose_t* out) {
    if (out) *out = latest;
}