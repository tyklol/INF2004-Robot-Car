#ifndef DRIVERS_IMU_H
#define DRIVERS_IMU_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float ax, ay, az;         // accel (filtered, counts or normalized)
    float mx, my, mz;         // magnetometer (raw - bias)
    float roll_deg, pitch_deg, yaw_deg;  // tilt-compensated heading
} imu_pose_t;

void imu_lpf_init(int window);                        // e.g., 8
void imu_lpf_push(float ax, float ay, float az);      // add one sample
void imu_lpf_get(float* fax, float* fay, float* faz); // get filtered accel

// Compute tilt angles from accel (degrees). Yaw needs mag/gyro; set to 0 for now.
void imu_tilt_from_accel(float ax, float ay, float az,
                         float* roll_deg, float* pitch_deg);

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

#endif // DRIVERS_IMU_H