#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "pico/stdlib.h"

// --- CONSTANTS ---
#define CENTER_ANGLE 90
#define BEAM_WIDTH_CORRECTION_DEG 7.0f 
#define DEPTH_TOLERANCE_CM 10.0f 

// --- FUNCTION PROTOTYPES ---

// Initialize Pins and PWM
void ultrasonic_init(uint trig, uint echo, uint servo);

// low-level hardware control
void set_servo_angle(float angle);
float get_median_distance(int samples);

// The V28 Math
float calculate_universal_width(float dist_center, int left_raw, int right_raw);

// The Full Scanning Routine (3 Passes)
// Returns the calculated width, or -1.0 if failed
float perform_full_scan_routine(float center_dist);

#endif