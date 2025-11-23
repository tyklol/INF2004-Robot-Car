#ifndef MOTOR_H
#define MOTOR_H

#include "sys_config.h" // Defines pins

#define MOTOR_LEFT  1
#define MOTOR_RIGHT 2

void motor_init(void);
void motor_set_speed(int motor_id, float speed);

#endif