#ifndef WS2812_H
#define WS2812_H

#include "sys_config.h"
#include "pico/stdlib.h"

void leds_init(void);
void update_leds(uint8_t r0, uint8_t g0, uint8_t b0, uint8_t r1, uint8_t g1, uint8_t b1);
void handle_smart_indicators(float steer);

#endif