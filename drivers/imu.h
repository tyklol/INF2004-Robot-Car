#ifndef IMU_H
#define IMU_H

#include <stdint.h>

void imu_init(void);

// Returns the current filtered Yaw (Heading) in degrees (0.0 to 360.0)
float imu_get_heading(void);

extern int16_t raw_accel_data[3];
extern int16_t raw_mag_data[3];
extern float filtered_accel[3];
extern float filtered_mag[3];
extern float pitch_deg, roll_deg, yaw_deg;
#endif // IMU_H