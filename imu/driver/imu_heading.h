// src/imu_heading.h
#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float ax, ay, az;         // accel (filtered, counts or normalized)
    float mx, my, mz;         // magnetometer (raw - bias)
    float roll_deg, pitch_deg, yaw_deg;  // tilt-compensated heading
} imu_pose_t;

bool imu_init_i2c0_gp0_gp1(void);
bool lsm303dlhc_init(void);

// Optional quick stationary bias calibration (keep board still ~2 s)
void imu_calibrate_bias(void);

// Sample sensors, update internal filters & compute pose
void imu_update(imu_pose_t* out);

// Access last pose without re-sampling (optional)
void imu_get_latest(imu_pose_t* out);

// Expose low-pass tuning (0..1, smaller = smoother)
void imu_set_accel_ema_alpha(float alpha);
// Set magnetometer hard-iron bias (if you measured them elsewhere)
void imu_set_mag_bias(float bx, float by, float bz);

#ifdef __cplusplus
}
#endif
