#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "pico/stdlib.h"
#include "sys_config.h"

// --- Tuning Constants (Specific to this sensor) ---
#define CENTER_ANGLE 90
#define BEAM_WIDTH_CORRECTION_DEG 7.0f 
#define DEPTH_TOLERANCE_CM 10.0f 

// --- Function Prototypes ---
// Note: We allow passing pins in init, but we use constants from sys_config in main
void ultrasonic_init(uint trig, uint echo, uint servo);
void set_servo_angle(float angle);
float get_median_distance(int samples);
float calculate_universal_width(float dist_center, int left_raw, int right_raw);
float perform_full_scan_routine(float center_dist);

#endif