#include "encoder.h"
#include "hardware/gpio.h"

static volatile int32_t g_total_ticks_left = 0;
static volatile int32_t g_total_ticks_right = 0;
static bool g_last_state_left = false;
static bool g_last_state_right = false;

void encoder_init(void) {
    gpio_init(PIN_ENC_L); gpio_set_dir(PIN_ENC_L, GPIO_IN); gpio_pull_up(PIN_ENC_L);
    gpio_init(PIN_ENC_R); gpio_set_dir(PIN_ENC_R, GPIO_IN); gpio_pull_up(PIN_ENC_R);
    g_last_state_left = gpio_get(PIN_ENC_L);
    g_last_state_right = gpio_get(PIN_ENC_R);
}

/**
 * @brief Resets the tick count for both encoders to zero.
 */
void encoder_reset_total_ticks(void) {
    g_total_ticks_left = 0;
    g_total_ticks_right = 0;
}

/**
 * @brief Gets the total accumulated ticks for the LEFT encoder.
 */
int32_t encoder_get_total_ticks_left(void) {
    return g_total_ticks_left;
}

/**
 * @brief Gets the total accumulated ticks for the RIGHT encoder.
 */
int32_t encoder_get_total_ticks_right(void) {
    return g_total_ticks_right;
}

/**
 * @brief Polls the encoder pins and updates the tick counts.
 * This should be called from a fast loop (e.g., 100Hz).
 */
void encoder_update_and_get_ticks(int32_t *delta_ticks_left, int32_t *delta_ticks_right) {
    // Reset delta counters
    *delta_ticks_left = 0;
    *delta_ticks_right = 0;

    // --- Poll Left Encoder ---
    bool current_state_left = gpio_get(PIN_ENC_L);
    // Check for a rising edge (0 -> 1)
    if (current_state_left && !g_last_state_left) {
        *delta_ticks_left = 1;
        g_total_ticks_left++;
    }
    g_last_state_left = current_state_left;

    // --- Poll Right Encoder ---
    bool current_state_right = gpio_get(PIN_ENC_R);
    // Check for a rising edge (0 -> 1)
    if (current_state_right && !g_last_state_right) {
        *delta_ticks_right = 1;
        g_total_ticks_right++;
    }
    g_last_state_right = current_state_right;
    
    // Note: This simple poll only counts rising edges and assumes
    // forward motion. For full quadrature, you would read Pin B as well.
    // For this project, this is all you need.
}