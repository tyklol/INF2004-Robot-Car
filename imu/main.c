// src/main.c   (build-safe version for Git upload)
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"

#include "imu_heading.h"
#include "net_wifi.h"
#include "mqtt_pub.h"
//#include "drivers/motor.h"   // TODO: enable later when motor driver ready

// ---------- PID (simple) ----------
// NOTE: temporarily renamed to avoid conflict with system pid_t
typedef struct {
    float kp, ki, kd;
    float i, prev_err;
    float out_min, out_max;
} pid_ctrl_t;

static void pid_init(pid_ctrl_t* p, float kp, float ki, float kd,
                     float out_min, float out_max) {
    p->kp=kp; p->ki=ki; p->kd=kd;
    p->i=0; p->prev_err=0;
    p->out_min=out_min; p->out_max=out_max;
}

static float pid_step(pid_ctrl_t* p, float err, float dt) {
    p->i += err * p->ki * dt;
    if (p->i > p->out_max) p->i = p->out_max;
    if (p->i < p->out_min) p->i = p->out_min;
    float d = (err - p->prev_err) / (dt > 0 ? dt : 1e-3f);
    p->prev_err = err;
    float out = p->kp*err + p->i + p->kd*d;
    if (out > p->out_max) out = p->out_max;
    if (out < p->out_min) out = p->out_min;
    return out;
}

// ---------- Config ----------
#define LOOP_HZ     20
#define DT_SEC      (1.0f/LOOP_HZ)
#define BASE_SPEED  35.0f
#define TRIM_LIMIT  20.0f
#define MQTT_MS     200

// ---------- Helpers ----------
static float heading_err_deg(float target, float current) {
    float e = target - current;
    while (e > 180.f) e -= 360.f;
    while (e < -180.f) e += 360.f;
    return e;
}

int main() {
    stdio_init_all();
    sleep_ms(1500);
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("\r\n=== IMU-assisted Straight Drive (build-safe version) ===\r\n");

    // --- I2C + IMU init
    imu_init_i2c0_gp0_gp1();
    if (!lsm303dlhc_init()) {
        printf("IMU init failed (WHO_AM_I != 0x33)\r\n");
        while (1) sleep_ms(1000);
    }
    printf("[IMU] Bias calibration... keep still.\r\n");
    imu_calibrate_bias();
    imu_set_accel_ema_alpha(0.12f);

    // --- TODO: motor_init(); (disabled for now)

    // --- Wi-Fi + MQTT
    if (!wifi_init_and_connect()) {
        printf("[WIFI] failed → continuing without MQTT\r\n");
    } else if (!mqtt_start()) {
        printf("[MQTT] start failed → continuing without MQTT\r\n");
    }

    // --- Heading target setup
    imu_pose_t pose;
    imu_update(&pose);
    float yaw_target = pose.yaw_deg;
    printf("[HEADING] target = %.1f deg\r\n", yaw_target);

    // --- TODO: enable PID + motor trim later
    pid_ctrl_t pid;
    pid_init(&pid, 1.0f, 0.02f, 0.10f, -TRIM_LIMIT, TRIM_LIMIT);

    absolute_time_t last_pub = get_absolute_time();
    uint32_t tick_ms = 0;

    while (true) {
        imu_update(&pose);

        float err  = heading_err_deg(yaw_target, pose.yaw_deg);
        float trim = pid_step(&pid, err, DT_SEC);

        // --- TODO: remove clamps when using real motors
        float left  = BASE_SPEED + trim;
        float right = BASE_SPEED - trim;

        if (left  > 100) left  = 100;
        if (left  < -100) left = -100;
        if (right > 100) right = 100;
        if (right < -100) right = -100;

        // --- Motor control disabled ---
        // motor_set_speed(0, left);
        // motor_set_speed(1, right);

        // --- Debug output only ---
        printf("[SIM] yaw_tgt=%.1f yaw=%.1f err=%6.1f trim=%6.1f  L=%.1f R=%.1f  | roll=%.1f pitch=%.1f\r\n",
               yaw_target, pose.yaw_deg, err, trim, left, right,
               pose.roll_deg, pose.pitch_deg);

        // --- MQTT publish (still active) ---
        if (absolute_time_diff_us(last_pub, get_absolute_time()) > MQTT_MS*1000) {
            last_pub = get_absolute_time();
            if (mqtt_is_connected()) {
                mqtt_pub_imu_raw(pose.ax, pose.ay, pose.az, 0,0,0, pose.mx, pose.my, pose.mz);
                mqtt_pub_imu_filtered(pose.ax, pose.ay, pose.az,
                                      pose.roll_deg, pose.pitch_deg, pose.yaw_deg);
            }
        }

        sleep_ms(1000/LOOP_HZ);
        tick_ms += (1000/LOOP_HZ);
    }
    return 0;
}
