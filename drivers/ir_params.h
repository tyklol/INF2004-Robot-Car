#ifndef _IR_PARAMS_H
#define _IR_PARAMS_H

/**
 * @brief Holds all tuning parameters for the IR line follower.
 */
typedef struct {
    float kp;                   // Proportional gain
    float ki;                   // Integral gain
    float kd;                   // Derivative gain
    float steer_clamp_tps;      // Max steering correction (in ticks-per-second)
    float base_speed_scale;     // Overall speed multiplier (e.g., 0.85 = 85%)
    float curve_slowdown;       // How much to slow down in turns (0.0 - 1.0)
    float curve_exp;            // Exponent for slowdown curve ( > 1.0 is more aggressive)
    float min_speed_fraction;   // The slowest the robot will go in a turn (e.g., 0.3 = 30%)
} ir_params_t;


#endif // _IR_PARAMS_H