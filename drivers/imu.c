#include "imu.h"
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

// -------- I2C & LSM303DLHC addresses / regs --------
#define I2C_PORT      i2c1  
#define PIN_SDA       2
#define PIN_SCL       3

// Accel (LSM303DLHC)
#define LSM303_ACC_ADDR   0x19
#define WHO_AM_I_A        0x0F
#define CTRL_REG1_A       0x20
#define CTRL_REG4_A       0x23
#define OUT_X_L_A         0x28   // auto-increment to X_H,Y_L,Y_H,Z_L,Z_H

// Mag (LSM303DLHC)
#define LSM303_MAG_ADDR   0x1E
#define CRA_REG_M         0x00   // data rate config
#define CRB_REG_M         0x01   // gain
#define MR_REG_M          0x02   // mode (continuous)
#define OUT_X_H_M         0x03   // then X_L, Z_H, Z_L, Y_H, Y_L

// -------- internals --------
static float a_alpha = 0.12f;          // EMA smoothing for accel
static float ax_f, ay_f, az_f;         // filtered accel
static bool  a_init = false;

static float a_bias_x=0, a_bias_y=0, a_bias_z=0;  // accel DC offsets (counts)
static float m_bias_x=0, m_bias_y=0, m_bias_z=0;  // mag DC offsets (raw units)

static imu_pose_t latest;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float bx[32], by[32], bz[32];
static int   N = 0, idx = 0, count = 0;
static float sx = 0, sy = 0, sz = 0;

void imu_lpf_init(int window) {
    if (window < 1) window = 1;
    if (window > 32) window = 32;
    N = window; idx = 0; count = 0;
    sx = sy = sz = 0;
    memset(bx, 0, sizeof(bx));
    memset(by, 0, sizeof(by));
    memset(bz, 0, sizeof(bz));
}

void imu_lpf_push(float ax, float ay, float az) {
    // subtract oldest
    sx -= bx[idx]; sy -= by[idx]; sz -= bz[idx];
    // insert newest
    bx[idx] = ax;  by[idx] = ay;  bz[idx] = az;
    sx += ax;      sy += ay;      sz += az;
    // advance
    idx = (idx + 1) % N;
    if (count < N) count++;
}

void imu_lpf_get(float* fax, float* fay, float* faz) {
    float denom = (count > 0) ? (float)count : 1.0f;
    if (fax) *fax = sx / denom;
    if (fay) *fay = sy / denom;
    if (faz) *faz = sz / denom;
}

// Simple tilt from accel (no gyro): roll/pitch only
void imu_tilt_from_accel(float ax, float ay, float az,
                         float* roll_deg, float* pitch_deg) {
    // Protect against divide-by-zero
    float g = sqrtf(ax*ax + ay*ay + az*az) + 1e-6f;

    // Conventional aerospace-ish:
    // roll  = atan2(ay, az)
    // pitch = atan2(-ax, sqrt(ay^2 + az^2))
    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));

    if (roll_deg)  *roll_deg  = roll  * (180.0f / (float)M_PI);
    if (pitch_deg) *pitch_deg = pitch * (180.0f / (float)M_PI);
}

// --- utils ---
static inline void i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t b[2] = {reg, val};
    i2c_write_blocking(I2C_PORT, addr, b, 2, false);
}

static inline void i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t* dst, size_t n) {
    uint8_t r = reg;
    i2c_write_blocking(I2C_PORT, addr, &r, 1, true);
    i2c_read_blocking(I2C_PORT, addr, dst, n, false);
}

bool imu_init_i2c0_gp0_gp1(void) {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);
    return true;
}

bool lsm303dlhc_init(void) {
    // Accel: 50Hz, XYZ enable; High-Res + BDU, ±2g
    // These are more stable than 10Hz/0x00.
    i2c_write_reg(LSM303_ACC_ADDR, CTRL_REG1_A, 0x47); // 0b0100_0111
    i2c_write_reg(LSM303_ACC_ADDR, CTRL_REG4_A, 0x88); // 0b1000_1000 (BDU+HR)
    sleep_ms(10);

    // Mag: 15 Hz (default), gain = +/-1.3G (default), continuous conversion
    i2c_write_reg(LSM303_MAG_ADDR, CRA_REG_M, 0x10); // 0b0001_0000 -> 15Hz
    i2c_write_reg(LSM303_MAG_ADDR, CRB_REG_M, 0x20); // gain setting (default-ish)
    i2c_write_reg(LSM303_MAG_ADDR, MR_REG_M,  0x00); // continuous-conversion
    sleep_ms(10);

    // WHO_AM_I check (accel)
    uint8_t who=0;
    i2c_read_regs(LSM303_ACC_ADDR, WHO_AM_I_A, &who, 1);
    return (who == 0x33);
}

void imu_set_accel_ema_alpha(float alpha) {
    if (alpha > 0.0f && alpha < 1.0f) a_alpha = alpha;
}

void imu_set_mag_bias(float bx, float by, float bz) {
    m_bias_x = bx; m_bias_y = by; m_bias_z = bz;
}

static void read_accel_counts(int16_t* x12, int16_t* y12, int16_t* z12) {
    uint8_t raw[6]={0};
    i2c_read_regs(LSM303_ACC_ADDR, OUT_X_L_A | 0x80, raw, 6); // auto-inc
    int16_t x16 = (int16_t)((raw[1]<<8)|raw[0]);
    int16_t y16 = (int16_t)((raw[3]<<8)|raw[2]);
    int16_t z16 = (int16_t)((raw[5]<<8)|raw[4]);
    *x12 = (x16 >> 4);
    *y12 = (y16 >> 4);
    *z12 = (z16 >> 4);
}

static void read_mag_raw(int16_t* mx, int16_t* my, int16_t* mz) {
    uint8_t raw[6]={0};
    // Note: order is X_H,X_L,Z_H,Z_L,Y_H,Y_L on DLHC
    i2c_read_regs(LSM303_MAG_ADDR, OUT_X_H_M, raw, 6);
    int16_t x = (int16_t)((raw[0]<<8)|raw[1]);
    int16_t z = (int16_t)((raw[2]<<8)|raw[3]);
    int16_t y = (int16_t)((raw[4]<<8)|raw[5]);
    *mx = x; *my = y; *mz = z;
}

void imu_calibrate_bias(void) {
    const int N=200;
    double sax=0, say=0, saz=0;
    double smx=0, smy=0, smz=0;
    for (int i=0;i<N;i++){
        int16_t x,y,z; read_accel_counts(&x,&y,&z);
        int16_t mx,my,mz; read_mag_raw(&mx,&my,&mz);
        sax+=x; say+=y; saz+=z;
        smx+=mx; smy+=my; smz+=mz;
        sleep_ms(10);
    }
    a_bias_x = (float)(sax/N);
    a_bias_y = (float)(say/N);
    a_bias_z = (float)(saz/N);
    m_bias_x = (float)(smx/N);
    m_bias_y = (float)(smy/N);
    m_bias_z = (float)(smz/N);
    a_init = false; // re-seed EMA at next update
}

static void accel_filter_push(float x, float y, float z) {
    // optional: median-3 prefilter could be added here
    if (!a_init) {
        ax_f=x; ay_f=y; az_f=z; a_init=true;
    } else {
        ax_f = a_alpha*x + (1.f - a_alpha)*ax_f;
        ay_f = a_alpha*y + (1.f - a_alpha)*ay_f;
        az_f = a_alpha*z + (1.f - a_alpha)*az_f;
    }
}

static void tilt_from_accel_norm(float ax, float ay, float az, float* roll_deg, float* pitch_deg) {
    // normalize
    float g = sqrtf(ax*ax + ay*ay + az*az) + 1e-6f;
    float nx = ax/g, ny = ay/g, nz = az/g;
    float roll  = atan2f(ny, nz);
    float pitch = atan2f(-nx, sqrtf(ny*ny + nz*nz));
    *roll_deg  = roll  * 57.29578f;
    *pitch_deg = pitch * 57.29578f;
}

static float heading_tilt_comp(float mx_raw, float my_raw, float mz_raw,
                               float roll_deg, float pitch_deg) {
    // Quick tilt compensation using roll/pitch (in radians)
    float cr = cosf(roll_deg  * 0.0174533f);
    float sr = sinf(roll_deg  * 0.0174533f);
    float cp = cosf(pitch_deg * 0.0174533f);
    float sp = sinf(pitch_deg * 0.0174533f);

    // rotate mag vector by inverse tilt (accel frame -> horizontal plane)
    // formulas adapted for LSM303 axis conventions
    float mxh = mx_raw*cp + mz_raw*sp;
    float myh = mx_raw*sr*sp + my_raw*cr - mz_raw*sr*cp;

    float heading = atan2f(myh, mxh) * 57.29578f;
    if (heading < 0) heading += 360.0f;
    return heading;
}

void imu_update(imu_pose_t* out) {
    int16_t ax_c, ay_c, az_c;
    int16_t mx_c, my_c, mz_c;

    read_accel_counts(&ax_c, &ay_c, &az_c);
    read_mag_raw(&mx_c, &my_c, &mz_c);

    // subtract biases
    float ax = (float)ax_c - a_bias_x;
    float ay = (float)ay_c - a_bias_y;
    float az = (float)az_c - a_bias_z;

    float mx = (float)mx_c - m_bias_x;
    float my = (float)my_c - m_bias_y;
    float mz = (float)mz_c - m_bias_z;

    // filter accel
    accel_filter_push(ax, ay, az);

    // roll/pitch from filtered accel
    float roll_deg, pitch_deg;
    tilt_from_accel_norm(ax_f, ay_f, az_f, &roll_deg, &pitch_deg);

    // tilt-compensated heading using mag
    float yaw_deg = heading_tilt_comp(mx, my, mz, roll_deg, pitch_deg);

    latest.ax=ax_f; latest.ay=ay_f; latest.az=az_f;
    latest.mx=mx;   latest.my=my;   latest.mz=mz;
    latest.roll_deg=roll_deg; latest.pitch_deg=pitch_deg; latest.yaw_deg=yaw_deg;

    if (out) *out = latest;
}

void imu_get_latest(imu_pose_t* out) {
    if (out) *out = latest;
}
