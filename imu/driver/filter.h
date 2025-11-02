#pragma once
#include <stdbool.h>

void imu_lpf_init(int window);                        // e.g., 8
void imu_lpf_push(float ax, float ay, float az);      // add one sample
void imu_lpf_get(float* fax, float* fay, float* faz); // get filtered accel

// Compute tilt angles from accel (degrees). Yaw needs mag/gyro; set to 0 for now.
void imu_tilt_from_accel(float ax, float ay, float az,
                         float* roll_deg, float* pitch_deg);
