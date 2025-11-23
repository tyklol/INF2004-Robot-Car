#include "pid_controller.h"
#include <float.h>

float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

void pid_init(pid_state_t *pid,
              float kp, float ki, float kd,
              float min_out, float max_out,
              float d_tau_s)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    // Ensure proper ordering of limits
    pid->min_output = (min_out <= max_out) ? min_out : max_out;
    pid->max_output = (max_out >= min_out) ? max_out : min_out;

    pid->d_tau_s = (d_tau_s < 0.0f) ? 0.0f : d_tau_s;

    pid->integral   = 0.0f;
    pid->last_meas  = 0.0f;
    pid->d_filtered = 0.0f;
    pid->d_init     = false;
}

void pid_reset(pid_state_t *pid) {
    pid->integral   = 0.0f;
    pid->d_filtered = 0.0f;
    pid->d_init     = false;
}

float pid_calculate(pid_state_t *pid, float target, float current, float dt_s)
{
    // Guard against nonpositive dt to avoid NaNs and spikes.
    if (dt_s <= 0.0f) {
        // P-only fallback; keep output bounded.
        float p_term = pid->kp * (target - current);
        return clampf(p_term, pid->min_output, pid->max_output);
    }

    // --- Error & P-term ---
    float error  = target - current;

    if (error > 180.0f) {
        error -= 360.0f;
    } else if (error < -180.0f) {
        error += 360.0f;
    }

    float p_term = pid->kp * error;

    // --- Derivative on measurement (avoids setpoint kicks) ---
    // d_raw = d(measurement)/dt
    float d_raw = (pid->d_init) ? (current - pid->last_meas) / dt_s : 0.0f;
    pid->last_meas = current;

    // Optional 1st-order low-pass on derivative
    float d_used;
    if (pid->d_tau_s > 0.0f) {
        // alpha = dt / (tau + dt)   (stable for varying dt)
        float alpha = dt_s / (pid->d_tau_s + dt_s);
        // Clamp alpha just in case of extreme dt_s
        alpha = clampf(alpha, 0.0f, 1.0f);

        pid->d_filtered = pid->d_init
                            ? (pid->d_filtered + alpha * (d_raw - pid->d_filtered))
                            : 0.0f;
        d_used = pid->d_filtered;
        pid->d_init = true;
    } else {
        d_used = d_raw;
        pid->d_init = true;
    }

    // Negative sign because derivative on measurement opposes error growth
    float d_term = -pid->kd * d_used;

    // --- Provisional PD output (no integral yet) ---
    float u_pd = p_term + d_term;

    // --- Integral with headroom clamping (anti-windup) ---
    // 1) Tentatively integrate error
    float i_term = 0.0f;
    if (pid->ki > 0.0f) {
        pid->integral += error * dt_s;

        // 2) Convert to i_term
        i_term = pid->ki * pid->integral;

        // 3) Compute integral bounds based on remaining headroom
        //    so that u = u_pd + i_term remains within [min, max].
        float i_max = pid->max_output - u_pd;
        float i_min = pid->min_output - u_pd;

        // 4) Clamp i_term to the headroom, and back-compute integral
        float i_term_clamped = clampf(i_term, i_min, i_max);
        if (i_term_clamped != i_term) {
            i_term = i_term_clamped;
            pid->integral = i_term / pid->ki; // back-calculate integral (anti-windup)
        }
    }

    // --- Sum and final clamp ---
    float output = u_pd + i_term;
    output = clampf(output, pid->min_output, pid->max_output);

    return output;
}
