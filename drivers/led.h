#ifndef LED_H
#define LED_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize the PIO hardware for the WS2812 LEDs.
 * Must be called before using update_leds or handle_smart_indicators.
 */
void led_init(void);

/**
 * @brief Directly set the color of the two LEDs.
 * * @param r0 Red component for LED 0 (0-255)
 * @param g0 Green component for LED 0 (0-255)
 * @param b0 Blue component for LED 0 (0-255)
 * @param r1 Red component for LED 1 (0-255)
 * @param g1 Green component for LED 1 (0-255)
 * @param b1 Blue component for LED 1 (0-255)
 */
void update_leds(uint8_t r0, uint8_t g0, uint8_t b0, uint8_t r1, uint8_t g1, uint8_t b1);

/**
 * @brief Logic to handle blinkers based on steering angle.
 * * @param steer The current steering input value.
 * < -12.0f triggers Left Blinker.
 * > 12.0f triggers Right Blinker.
 * Otherwise solid dim blue.
 */
void handle_smart_indicators(float steer);

#endif // LED_H