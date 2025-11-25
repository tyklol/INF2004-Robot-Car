#include "encoder.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

// --- Internal State ---
static volatile int32_t g_ticks_left = 0;
static volatile int32_t g_ticks_right = 0;

// Variables for speed calculation
static uint32_t last_speed_calc_time = 0;
static int32_t last_ticks_avg = 0;

// --- Interrupt Handler ---
void encoder_handle_interrupt(uint gpio, uint32_t events) {
    if (gpio == PIN_ENC_L) {
        g_ticks_left++;
    } else if (gpio == PIN_ENC_R) {
        g_ticks_right++;
    }
}

void encoder_init(void) {
    // 1. Initialize GPIOs
    gpio_init(PIN_ENC_L);
    gpio_set_dir(PIN_ENC_L, GPIO_IN);
    gpio_pull_up(PIN_ENC_L);

    gpio_init(PIN_ENC_R);
    gpio_set_dir(PIN_ENC_R, GPIO_IN);
    gpio_pull_up(PIN_ENC_R);

    // CHANGE 2: Just enable the IRQ, do not set the callback (main.c will handle it)
    gpio_set_irq_enabled(PIN_ENC_L, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(PIN_ENC_R, GPIO_IRQ_EDGE_RISE, true);

    // 3. Reset state
    encoder_reset_total_ticks();
    last_speed_calc_time = to_ms_since_boot(get_absolute_time());
}

void encoder_reset_total_ticks(void) {
    g_ticks_left = 0;
    g_ticks_right = 0;
    last_ticks_avg = 0;
    last_speed_calc_time = to_ms_since_boot(get_absolute_time());
}

int32_t encoder_get_left_ticks(void) {
    return g_ticks_left;
}

int32_t encoder_get_right_ticks(void) {
    return g_ticks_right;
}

// --- MATH IMPLEMENTATION ---

float encoder_get_distance_cm(void) {
    // Average the two wheels to get center distance
    int32_t avg_ticks = (g_ticks_left + g_ticks_right) / 2;
    return (float)avg_ticks * CM_PER_TICK;
}

float encoder_calculate_speed_cms(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    uint32_t dt = now - last_speed_calc_time;

    // Avoid division by zero or super fast updates
    if (dt < 50) return 0.0f; // Return 0 if called too fast

    // Calculate delta ticks (average of L+R)
    int32_t current_avg_ticks = (g_ticks_left + g_ticks_right) / 2;
    int32_t delta_ticks = current_avg_ticks - last_ticks_avg;

    // Distance moved in this interval
    float distance_moved = (float)delta_ticks * CM_PER_TICK;

    // Speed = Distance / Time (seconds)
    float speed = distance_moved / ((float)dt / 1000.0f);

    // Update history
    last_ticks_avg = current_avg_ticks;
    last_speed_calc_time = now;

    return speed;
}