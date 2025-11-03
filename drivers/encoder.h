#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "pico/stdlib.h"
#include <stdint.h>

// --- PIN Definitions ---
#define PIN_ENC_L 0 // Left Encoder (GP7)
#define PIN_ENC_R 7 // Right Encoder (GP0)

/**
 * @brief Initializes the encoder GPIO pins as inputs.
 */
void encoder_init(void);

/**
 * @brief Resets the total tick counters to zero.
 * Call this when you change state (e.g., from STOPPED to RUNNING).
 */
void encoder_reset_total_ticks(void);

/**
 * @brief Gets the total accumulated ticks for the LEFT encoder.
 * Used for calculating total distance.
 * @return Total ticks since the last reset.
 */
int32_t encoder_get_total_ticks_left(void);

/**
 * @brief Gets the total accumulated ticks for the RIGHT encoder.
 * Used for calculating total distance.
 * @return Total ticks since the last reset.
 */
int32_t encoder_get_total_ticks_right(void);

/**
 * @brief Polls the encoder pins and updates the tick counts.
 * THIS IS THE MAIN NEW FUNCTION. Call it once per control loop.
 *
 * @param delta_ticks_left Pointer to store the ticks counted *since the last call*.
 * @param delta_ticks_right Pointer to store the ticks counted *since the last call*.
 */
void encoder_update_and_get_ticks(int32_t *delta_ticks_left, int32_t *delta_ticks_right);

#endif // _ENCODER_H_