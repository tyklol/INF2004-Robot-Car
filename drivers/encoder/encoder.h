#ifndef ENCODER_H
#define ENCODER_H

#include "pico/stdlib.h"
#include <stdint.h>
#include "sys_config.h" 

void encoder_init(void);
void encoder_reset_total_ticks(void);
int32_t encoder_get_total_ticks_left(void);
int32_t encoder_get_total_ticks_right(void);
void encoder_update_and_get_ticks(int32_t *delta_ticks_left, int32_t *delta_ticks_right);

#endif