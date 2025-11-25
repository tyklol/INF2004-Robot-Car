#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "pico/stdlib.h"
#include <stdint.h>

// --- PIN Definitions ---
#define PIN_ENC_L 7 // Left Encoder (GP7)
#define PIN_ENC_R 0 // Right Encoder (GP0)

// --- CALIBRATION CONSTANTS (TUNE THESE!) ---
// Standard yellow motor wheel diameter is ~6.5cm.
// Encoder disk usually has 20 slots.
// Circumference = 6.5 * PI = ~20.42 cm
// CM per Tick = 20.42 / 20 = ~1.02 cm
#define CM_PER_TICK 1.02f 

void encoder_init(void);
void encoder_reset_total_ticks(void);

// Get raw ticks
int32_t encoder_get_left_ticks(void);
int32_t encoder_get_right_ticks(void);

// --- NEW TELEMETRY FUNCTIONS ---
// Returns total distance averaged between two wheels in cm
float encoder_get_distance_cm(void);

// Returns current speed in cm/s (call this regularly, e.g., every 200ms)
float encoder_calculate_speed_cms(void);

void encoder_handle_interrupt(uint gpio, uint32_t events);
#endif // _ENCODER_H_