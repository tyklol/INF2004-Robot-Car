#ifndef IMU_H
#define IMU_H
#include "sys_config.h"

extern int16_t raw_accel_data[3];
extern int16_t raw_mag_data[3];
extern float filtered_accel[3];
extern float filtered_mag[3];
extern float pitch_deg, roll_deg, yaw_deg;

void imu_init(void);
float imu_get_yaw_rate(void);
float get_filtered_heading(int16_t raw_accel[3], int16_t raw_mag[3], float filtered_accel[3], float filtered_mag[3], 
    float *pitch_deg, float *roll_deg, float *yaw_deg);

#endif