#pragma once
#include <stdbool.h>

typedef struct {
    // Gains
    float kp, ki, kd;

    // Output limits (map to PWM range, e.g. -1..1 or 0..1)
    float min_output, max_output;

    // Derivative filter time constant (seconds). 0 = no filtering.
    float d_tau_s;

    // ---- Internal state ----
    float integral;       // accumulated error
    float last_meas;      // last measurement for D-on-meas
    float d_filtered;     // filtered derivative of measurement
    bool  d_init;         // derivative state initialized?
} pid_state_t;

/**
 * Initialize PID controller.
 * d_tau_s: 0 for raw derivative, else typical 0.02–0.05 s.
 */
void pid_init(pid_state_t *pid,
              float kp, float ki, float kd,
              float min_out, float max_out,
              float d_tau_s);

/** Zero internal states (integral, derivative, etc.). */
void pid_reset(pid_state_t *pid);

/**
 * Compute control output.
 * target:   desired value (e.g., wheel speed)
 * current:  measured value from encoders
 * dt_s:     loop interval in seconds (must be > 0)
 */
float pid_calculate(pid_state_t *pid, float target, float current, float dt_s);

/** Optional helpers */
static inline void pid_set_gains(pid_state_t *pid, float kp, float ki, float kd) {
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
}
static inline void pid_set_limits(pid_state_t *pid, float min_out, float max_out) {
    pid->min_output = min_out; pid->max_output = max_out;
}
static inline void pid_set_derivative_tau(pid_state_t *pid, float d_tau_s) {
    pid->d_tau_s = d_tau_s;
}
