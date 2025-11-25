#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/sync.h"
#include "ultrasonic.h"

// Internal variables for pin mapping
static uint _trig_pin;
static uint _echo_pin;
static uint _servo_pin;
static uint _servo_slice;
static uint _servo_channel;

// Servo Pulse Limits
#define MIN_PULSE_US 500   
#define MAX_PULSE_US 2400  

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --- INITIALIZATION ---
void ultrasonic_init(uint trig, uint echo, uint servo) {
    _trig_pin = trig;
    _echo_pin = echo;
    _servo_pin = servo;

    // GPIO
    gpio_init(_trig_pin); gpio_set_dir(_trig_pin, GPIO_OUT); gpio_put(_trig_pin, 0);
    gpio_init(_echo_pin); gpio_set_dir(_echo_pin, GPIO_IN); gpio_pull_down(_echo_pin);

    // PWM for Servo
    gpio_set_function(_servo_pin, GPIO_FUNC_PWM);
    _servo_slice = pwm_gpio_to_slice_num(_servo_pin);
    _servo_channel = pwm_gpio_to_channel(_servo_pin);
    
    pwm_set_wrap(_servo_slice, 20000); 
    pwm_set_clkdiv(_servo_slice, 125.0f);
    pwm_set_enabled(_servo_slice, true);
}

// --- HARDWARE FUNCTIONS ---

void set_servo_angle(float angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    float pulse_us = MIN_PULSE_US + (angle / 180.0f) * (MAX_PULSE_US - MIN_PULSE_US);
    pwm_set_chan_level(_servo_slice, _servo_channel, (uint16_t)pulse_us);
}

static float read_raw_distance() {
    // 1. Enter Critical Section (Disable Interrupts)
    // This ensures no background task messes up the microsecond timing
    uint32_t status = save_and_disable_interrupts();

    gpio_put(_trig_pin, 1);
    sleep_us(10);
    gpio_put(_trig_pin, 0);

    absolute_time_t timeout = make_timeout_time_ms(60); 
    
    // Wait for Echo HIGH
    while (gpio_get(_echo_pin) == 0) {
        if (time_reached(timeout)) {
            restore_interrupts(status); // Restore before returning
            return -1.0f; 
        }
    }

    absolute_time_t start_time = get_absolute_time();
    
    // Wait for Echo LOW
    while (gpio_get(_echo_pin) == 1) {
        if (absolute_time_diff_us(start_time, get_absolute_time()) > 35000) {
            restore_interrupts(status); // Restore before returning
            return -1.0f; 
        }
    }
    
    int64_t pulse_duration = absolute_time_diff_us(start_time, get_absolute_time());
    
    // 2. Exit Critical Section
    restore_interrupts(status);

    return (float)pulse_duration * 0.0343f / 2.0f;
}

static int compare_floats(const void *a, const void *b) {
    float fa = *(const float*)a;
    float fb = *(const float*)b;
    return (fa > fb) - (fa < fb);
}

float get_median_distance(int samples) {
    float readings[30]; 
    if (samples > 30) samples = 30;
    int valid_count = 0;

    for(int i=0; i<samples; i++) {
        float d = read_raw_distance();
        if (d > 4.0f && d < 300.0f) { 
            readings[valid_count++] = d;
        }
        sleep_ms(12); 
    }

    if (valid_count == 0) return -1.0f;
    qsort(readings, valid_count, sizeof(float), compare_floats);
    return readings[valid_count / 2];
}

// --- MATH LOGIC ---

float calculate_universal_width(float dist_center, int left_raw, int right_raw) {
    int raw_span_int = left_raw - right_raw;
    if (raw_span_int < 0) raw_span_int = 0;
    float raw_span_deg = (float)raw_span_int;

    float corrected_span_deg = raw_span_deg - BEAM_WIDTH_CORRECTION_DEG;
    if (corrected_span_deg < 0.1f) corrected_span_deg = 0.1f;

    float center_angle_raw = (float)(left_raw + right_raw) / 2.0f;
    float tilt_offset = center_angle_raw - 90.0f;
    float perp_dist = dist_center * cosf(tilt_offset * (M_PI / 180.0f));

    float half_span_rad = (corrected_span_deg / 2.0f) * (M_PI / 180.0f);
    float width = 2.0f * perp_dist * tanf(half_span_rad);

    return width;
}

// --- SCANNING ROUTINE ---

float perform_full_scan_routine(float center_dist) {
    float sum_width = 0;
    int valid_scans = 0;

    // Run 3 Passes
    for (int i = 1; i <= 3; i++) {
        int right_raw = CENTER_ANGLE;
        int left_raw = CENTER_ANGLE;
        
        // Scan Right (90 -> 0)
        int misses = 0;
        for (int a = CENTER_ANGLE; a >= 0; a -= 2) {
            set_servo_angle(a);
            sleep_ms(25); 
            float d = get_median_distance(3); 
            
            bool invalid = (d == -1.0f);
            bool deep_jump = (d > (center_dist + DEPTH_TOLERANCE_CM));

            if (invalid || deep_jump) {
                misses++;
                if (misses >= 3) { right_raw = a + 6; break; }
            } else {
                misses = 0; right_raw = a;
            }
        }
        
        set_servo_angle(CENTER_ANGLE);
        sleep_ms(100);

        // Scan Left (90 -> 180)
        misses = 0;
        for (int a = CENTER_ANGLE; a <= 180; a += 2) {
            set_servo_angle(a);
            sleep_ms(25); 
            float d = get_median_distance(3);
            
            bool invalid = (d == -1.0f);
            bool deep_jump = (d > (center_dist + DEPTH_TOLERANCE_CM));

            if (invalid || deep_jump) {
                misses++;
                if (misses >= 3) { left_raw = a - 6; break; }
            } else {
                misses = 0; left_raw = a;
            }
        }

        float w = calculate_universal_width(center_dist, left_raw, right_raw);
        printf("  Pass %d: Width %.2f cm\n", i, w);
        
        if (w > 2.0f) {
            sum_width += w;
            valid_scans++;
        }

        set_servo_angle(CENTER_ANGLE);
        sleep_ms(300);
    }

    if (valid_scans > 0) {
        return sum_width / valid_scans;
    }
    return -1.0f; // Failed
}

int scan_for_clear_path(void) {
    float left_score = 0;
    float right_score = 0;
    
    // 1. Check RIGHT side (approx 45 degrees)
    set_servo_angle(45);
    sleep_ms(300); // Wait for servo to move
    float d_right = get_median_distance(5);
    if (d_right == -1.0f) d_right = 300.0f; // No echo = infinite space
    right_score = d_right;
    
    // 2. Check LEFT side (approx 135 degrees)
    set_servo_angle(135);
    sleep_ms(300);
    float d_left = get_median_distance(5);
    if (d_left == -1.0f) d_left = 300.0f;
    left_score = d_left;

    // 3. Center the servo back
    set_servo_angle(90);
    sleep_ms(200);

    printf("Deciding Path: Left Space=%.1f, Right Space=%.1f\n", left_score, right_score);

    // 4. Decision Logic
    // If Left has significantly more space (buffer of 10cm), go Left.
    if (left_score > (right_score + 10.0f)) {
        return -1; // Go LEFT
    }
    // Otherwise go RIGHT (Default preference if equal)
    else {
        return 1;  // Go RIGHT
    }
}