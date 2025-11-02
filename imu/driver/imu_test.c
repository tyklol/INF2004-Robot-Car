// src/imu_test.c
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "pico/time.h"         // absolute_time_t, get_absolute_time, etc.
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/cyw43_arch.h"   // cyw43_arch_poll()

#include "net_wifi.h"
#include "mqtt_pub.h"
#include "filter.h"            // imu_lpf_init / imu_lpf_push / imu_lpf_get / imu_tilt_from_accel

// ---------- I2C & IMU config (same as your main.c) ----------
#define I2C_PORT          i2c0
#define PIN_SDA           0
#define PIN_SCL           1

#define LSM303_ACC_ADDR   0x19
#define WHO_AM_I_A        0x0F
#define CTRL_REG1_A       0x20
#define CTRL_REG4_A       0x23
#define OUT_X_L_A         0x28   // X_L,X_H,Y_L,Y_H,Z_L,Z_H (auto-increment)

// ---------- Test parameters (tune if needed) ----------
#define SAMPLE_INTERVAL_MS   100      // 10 Hz sampling
#define RAW_SAMPLE_COUNT     100      // for σ comparison (use 100 for better stats)
#define LPF_WINDOW_N         16       // stronger smoothing; use 8–16
#define MQTT_PERIOD_MS       200      // publish ~5 Hz
#define PAUSE_MS             2000

// ---------- Offsets (quick bias calibration at boot) ----------
static float offx = 0.f, offy = 0.f, offz = 0.f;

// ---------- Helpers ----------
static void pause_between_tests(const char* label) {
    printf("\n[%s] ... pausing %d ms ...\n\n", label, PAUSE_MS);
    sleep_ms(PAUSE_MS);
}

static void i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
}

static void i2c_read_regs(uint8_t addr, uint8_t start_reg, uint8_t *dst, size_t len) {
    uint8_t reg = start_reg | 0x80; // auto-increment
    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);  // repeated start (no stop)
    i2c_read_blocking(I2C_PORT, addr, dst, len, false);
}

static void read_xyz_counts(int16_t* x12, int16_t* y12, int16_t* z12) {
    uint8_t raw[6] = {0};
    i2c_read_regs(LSM303_ACC_ADDR, OUT_X_L_A, raw, 6);
    int16_t x16 = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t y16 = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t z16 = (int16_t)((raw[5] << 8) | raw[4]);
    *x12 = (x16 >> 4);
    *y12 = (y16 >> 4);
    *z12 = (z16 >> 4);
}

static float mean(const float* v, int n) { double s=0; for(int i=0;i<n;i++) s+=v[i]; return (float)(s/n); }
static float stdev(const float* v, int n) {
    if (n<2) return 0.f;
    double m = mean(v,n), s2=0;
    for (int i=0;i<n;i++){ double d=v[i]-m; s2+=d*d; }
    return (float)sqrt(s2/(n-1));
}

// Quick stationary calibration to remove small DC bias in counts
static void imu_calibrate_offsets(void) {
    const int N = 200;
    double sx=0, sy=0, sz=0;
    for (int i=0; i<N; i++) {
        int16_t x,y,z; read_xyz_counts(&x,&y,&z);
        sx += x; sy += y; sz += z;
        sleep_ms(10);
    }
    offx = (float)(sx/N);
    offy = (float)(sy/N);
    offz = (float)(sz/N);
    printf("[CAL] offsets: x=%.1f y=%.1f z=%.1f (counts)\n", offx, offy, offz);
}

// ---------- Tests ----------
int main() {
    // Bring up stdio and make prints immediate
    stdio_init_all();
    sleep_ms(2000);
    setvbuf(stdout, NULL, _IONBF, 0);

    printf("\r\n--- IMU Driver Unit Tests (Week-9) ---\r\n");
    printf("USB stdio ON; UART stdio OFF (avoid I2C GP0/1 clash).\r\n");

    // I2C init (same as your main.c)
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);

    // ========== IMU-UT-01: WHO_AM_I ==========
    printf("=== [IMU-UT-01] Verify I2C WHO_AM_I ===\r\n");
    uint8_t who=0;
    i2c_read_regs(LSM303_ACC_ADDR, WHO_AM_I_A, &who, 1);
    printf("WHO_AM_I_A = 0x%02X (expect 0x33)\r\n", who);
    printf("Result: %s\r\n", (who==0x33) ? "Pass" : "Fail");
    pause_between_tests("IMU-UT-01");

    // Configure accel (same as your main.c)
    i2c_write_reg(LSM303_ACC_ADDR, CTRL_REG1_A, 0x27); // 10 Hz, XYZ on, normal
    i2c_write_reg(LSM303_ACC_ADDR, CTRL_REG4_A, 0x00); // ±2g, little-endian
    sleep_ms(10);

    // Bias calibration (keep board still during this step)
    imu_calibrate_offsets();

    // ========== IMU-UT-02: Raw XYZ ==========
    printf("=== [IMU-UT-02] Raw XYZ reading (tilt board) ===\r\n");
    for (int i=0; i<10; i++) {
        int16_t x,y,z;
        read_xyz_counts(&x,&y,&z);
        printf("RAW x12=%6d y12=%6d z12=%6d\r\n", x,y,z);
        sleep_ms(SAMPLE_INTERVAL_MS);
    }
    printf("Expected: Values vary with tilt.\r\n");
    printf("Result: Manual mark Pass if changed with tilt.\r\n");
    pause_between_tests("IMU-UT-02");

    // ========== IMU-UT-03: LPF noise reduction ==========
    printf("=== [IMU-UT-03] LPF noise reduction (stationary) ===\r\n");
    printf("Keep board still for ~%d samples...\r\n", RAW_SAMPLE_COUNT);
    imu_lpf_init(LPF_WINDOW_N);

    float rx[RAW_SAMPLE_COUNT], ry[RAW_SAMPLE_COUNT], rz[RAW_SAMPLE_COUNT];
    float fx[RAW_SAMPLE_COUNT], fy[RAW_SAMPLE_COUNT], fz[RAW_SAMPLE_COUNT];

    for (int i=0; i<RAW_SAMPLE_COUNT; i++) {
        int16_t x,y,z; read_xyz_counts(&x,&y,&z);
        // subtract offsets before filtering
        float bx = (float)x - offx;
        float by = (float)y - offy;
        float bz = (float)z - offz;

        rx[i]=bx; ry[i]=by; rz[i]=bz;
        imu_lpf_push(bx, by, bz);
        imu_lpf_get(&fx[i], &fy[i], &fz[i]);

        if ((i % 10) == 0) printf(".");   // progress
        sleep_ms(SAMPLE_INTERVAL_MS);
    }
    printf("\r\n");

    float sx_raw=stdev(rx,RAW_SAMPLE_COUNT), sy_raw=stdev(ry,RAW_SAMPLE_COUNT), sz_raw=stdev(rz,RAW_SAMPLE_COUNT);
    float sx_flt=stdev(fx,RAW_SAMPLE_COUNT), sy_flt=stdev(fy,RAW_SAMPLE_COUNT), sz_flt=stdev(fz,RAW_SAMPLE_COUNT);

    printf("StdDev RAW   : X=%.2f Y=%.2f Z=%.2f (counts)\r\n", sx_raw, sy_raw, sz_raw);
    printf("StdDev FILTER: X=%.2f Y=%.2f Z=%.2f (counts)\r\n", sx_flt, sy_flt, sz_flt);

    bool pass_all = (sx_flt <= 0.4f*sx_raw) && (sy_flt <= 0.4f*sy_raw) && (sz_flt <= 0.4f*sz_raw);
    printf("Expected: σ_filtered ≤ 0.4 × σ_raw on all axes.\r\n");
    printf("Result: %s\r\n", pass_all ? "Pass" : "Fail");
    pause_between_tests("IMU-UT-03");

    // ========== IMU-UT-04: Tilt angles ==========
    printf("=== [IMU-UT-04] Roll/Pitch from filtered accel ===\r\n");
    float fax=0, fay=0, faz=0, roll_deg=0, pitch_deg=0;

    // Read a few filtered samples while level
    for (int i=0;i<10;i++){
        int16_t x,y,z; read_xyz_counts(&x,&y,&z);
        imu_lpf_push((float)x - offx, (float)y - offy, (float)z - offz);
        imu_lpf_get(&fax,&fay,&faz);
        sleep_ms(50);
    }

    // (Your filter.h should already compute from accel; otherwise you can normalize here)
    imu_tilt_from_accel(fax, fay, faz, &roll_deg, &pitch_deg);
    printf("LEVEL → roll=%6.1f°, pitch=%6.1f° (expect ~0)\r\n", roll_deg, pitch_deg);

    printf("Now tilt ~+30° about X, hold 2 s...\r\n");
    sleep_ms(2000);
    for (int i=0;i<10;i++){
        int16_t x,y,z; read_xyz_counts(&x,&y,&z);
        imu_lpf_push((float)x - offx, (float)y - offy, (float)z - offz);
        imu_lpf_get(&fax,&fay,&faz);
        sleep_ms(50);
    }
    imu_tilt_from_accel(fax, fay, faz, &roll_deg, &pitch_deg);
    printf("TILT +30° → roll=%6.1f°, pitch=%6.1f° (expect ±30°±5°)\r\n", roll_deg, pitch_deg);
    printf("Result: Manual mark Pass if within tolerance.\r\n");
    pause_between_tests("IMU-UT-04");

    // ========== IMU-UT-05/06: MQTT publish raw & filtered ==========
    printf("=== [IMU-UT-05/06] MQTT publish raw & filtered ===\r\n");
    printf("Connecting Wi-Fi + MQTT...\r\n");
    if (!wifi_init_and_connect()) {
        printf("Fatal: Wi-Fi failed → Fail\r\n");
        while (true) sleep_ms(1000);
    }
    if (!mqtt_start()) {
        printf("Fatal: MQTT connect failed → Fail\r\n");
        while (true) sleep_ms(1000);
    }
    printf("On your PC, run:\r\n  mosquitto_sub -h <PC_IP> -t \"robot/imu/#\" -v\r\n");

    absolute_time_t last_pub = get_absolute_time();
    uint32_t ms = 0;
    while (ms < 10000) { // ~10s window for screenshots
        cyw43_arch_poll();
        sleep_ms(10);
        ms += 10;

        int16_t x,y,z; read_xyz_counts(&x,&y,&z);
        float bx = (float)x - offx;
        float by = (float)y - offy;
        float bz = (float)z - offz;

        imu_lpf_push(bx, by, bz);
        imu_lpf_get(&fax,&fay,&faz);
        imu_tilt_from_accel(fax,fay,faz,&roll_deg,&pitch_deg);

        if (absolute_time_diff_us(last_pub, get_absolute_time()) > (MQTT_PERIOD_MS*1000)) {
            last_pub = get_absolute_time();
            bool ok1 = mqtt_pub_imu_raw((float)x, (float)y, (float)z, 0,0,0, 0,0,0);
            bool ok2 = mqtt_pub_imu_filtered(fax, fay, faz, roll_deg, pitch_deg, 0.0f);
            printf("[PUB] raw=%s filtered=%s | roll=%.1f pitch=%.1f\r\n",
                   ok1?"OK":"ERR", ok2?"OK":"ERR", roll_deg, pitch_deg);
        }
    }
    printf("Expected: Both topics visible on mosquitto_sub and dashboard.\r\n");

    printf("\r\n--- ALL IMU TESTS COMPLETE ---\r\n");
    while (true) sleep_ms(1000);
    return 0;
}
