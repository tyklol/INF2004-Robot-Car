#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "math.h"

// === CONFIG ===
#define SERVO_PIN 15
#define TRIG_PIN 0
#define ECHO_PIN 1
int timeout = 300000; // increased for reliable echo detection

// === SERVO HELPERS (FS90) ===
// FS90 typically uses ~700–2400 µs for full range (≈120–135°)
void set_servo_us(uint pin, uint16_t us) {
    uint slice = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);
    uint16_t level = (uint16_t)(us * 1.953); // 1 µs = 1.953 ticks
    pwm_set_chan_level(slice, channel, level);
}

void init_servo(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_clkdiv(slice, 64.0f);
    pwm_set_wrap(slice, 39062); // 20 ms period = 50 Hz
    pwm_set_enabled(slice, true);
    set_servo_us(pin, 1500); // center
}

// === ULTRASONIC HELPERS ===
void setupUltrasonicPins(uint trigPin, uint echoPin) {
    gpio_init(trigPin);
    gpio_init(echoPin);
    gpio_set_dir(trigPin, GPIO_OUT);
    gpio_set_dir(echoPin, GPIO_IN);
}

uint64_t getPulse(uint trigPin, uint echoPin) {
    gpio_put(trigPin, 0);
    sleep_us(2);
    gpio_put(trigPin, 1);
    sleep_us(20);
    gpio_put(trigPin, 0);

    absolute_time_t startWait = get_absolute_time();
    while (gpio_get(echoPin) == 0) {
        if (absolute_time_diff_us(startWait, get_absolute_time()) > timeout)
            return 0;
    }

    absolute_time_t startTime = get_absolute_time();
    while (gpio_get(echoPin) == 1) {
        if (absolute_time_diff_us(startTime, get_absolute_time()) > timeout)
            return 0;
    }

    return absolute_time_diff_us(startTime, get_absolute_time());
}

uint64_t getCm(uint trigPin, uint echoPin) {
    uint64_t pulseLength = getPulse(trigPin, echoPin);
    if (pulseLength == 0) return 0;
    return pulseLength / 29 / 2;
}

// === ULTRASONIC AVERAGING Filter ===
uint64_t getCmAvg(uint trigPin, uint echoPin, int samples) {
    uint64_t sum = 0;
    int valid = 0;
    for (int i = 0; i < samples; i++) {
        uint64_t cm = getCm(trigPin, echoPin);
        if (cm > 0) {
            sum += cm;
            valid++;
        }
        sleep_ms(10); // small delay between samples
    }
    return (valid > 0) ? (sum / valid) : 0;
}


//======================================Not working correctly==============================
// Map a servo pulse (us) to radians for arbitrary min/max pulses
static inline float us_to_radians(uint16_t us, uint16_t min_us, uint16_t max_us, float max_deg) {
    float center_us = (min_us + max_us) / 2.0f;
    float half_range = (max_us - min_us) / 2.0f;
    float norm = ((float)us - center_us) / half_range; // -1..+1
    float angle_deg = norm * max_deg;
    return angle_deg * (M_PI / 180.0f);
}
//======================================Not working correctly==============================


// === EDGE DETECTION SWEEP ===
// Sweep from start_us toward end_us in steps; stop when a sharp jump observed.
// Returns the us position where jump occurred (edge_us), or -1 if none.
int16_t sweep_stop_on_jump(uint trigPin, uint echoPin,
                           uint16_t start_us, uint16_t end_us,
                           int step_us, int delay_ms,
                           int JUMP_THRESHOLD_CM,
                           int *stableDist)
{
    int prev = (int)getCm(trigPin, echoPin);
    sleep_ms(50);

    if (start_us < end_us) {
        for (uint16_t us = start_us; us <= end_us; us += step_us) {
            set_servo_us(SERVO_PIN, us);
            sleep_ms(delay_ms);

            int cur = (int)getCm(trigPin, echoPin);
            printf("US %u -> %d cm\n", us, cur);

            // check jump
            if (prev > 0 && cur > 0 && abs(cur - prev) >= JUMP_THRESHOLD_CM) {
                // return the last stable distance before jump
                if (stableDist) *stableDist = prev;
                return (int16_t)us;
            }
            prev = cur;
        }
    } else {
        for (uint16_t us = start_us; us >= end_us; us -= step_us) {
            set_servo_us(SERVO_PIN, us);
            sleep_ms(delay_ms);

            int cur = (int)getCm(trigPin, echoPin);
            printf("US %u -> %d cm\n", us, cur);

            if (prev > 0 && cur > 0 && abs(cur - prev) >= JUMP_THRESHOLD_CM) {
                if (stableDist) *stableDist = prev;
                return (int16_t)us;
            }
            prev = cur;
            if (us < (uint16_t)step_us) break;
        }
    }

    if (stableDist) *stableDist = prev; // use last measured value if no jump
    return -1;
}


// === Updated sweep limits for FS90 ===
// === Scan function using law of cosines ===
void scan_center_left_right_and_report() {
    const uint16_t center = 1500;
    const uint16_t left = 700;
    const uint16_t right = 2300;
    const int step_us = 10;
    const int delay_ms = 150;
    const int JUMP_THRESHOLD_CM = 3;
    //const float MAX_ANGLE_DEG = 60.0f;  // FS90 full sweep

    set_servo_us(SERVO_PIN, center);
    sleep_ms(300);

    // === Scan right ===
    int rightStableDist = 0;
    int16_t rightEdge_us = sweep_stop_on_jump(TRIG_PIN, ECHO_PIN, center, right,
                                              step_us, delay_ms, JUMP_THRESHOLD_CM,
                                              &rightStableDist);
    if (rightEdge_us < 0) rightEdge_us = right;
    set_servo_us(SERVO_PIN, center);
    sleep_ms(300);

    // === Scan left ===
    int leftStableDist = 0;
    int16_t leftEdge_us = sweep_stop_on_jump(TRIG_PIN, ECHO_PIN, center, left,
                                             step_us, delay_ms, JUMP_THRESHOLD_CM,
                                             &leftStableDist);
    if (leftEdge_us < 0) leftEdge_us = left;
    set_servo_us(SERVO_PIN, center);
    sleep_ms(400);
    //======================================Not working correctly==============================
    printf("Right edge dist (stable): %d cm, Left edge dist (stable): %d cm\n",
           rightStableDist, leftStableDist);

    // Convert servo pulse widths to angles relative to center
    float thetaR = us_to_radians(rightEdge_us, 700, 2300, 120.0f);
    float thetaL = us_to_radians(leftEdge_us, 700, 2300, 120.0f);

    float widthEstimate = 0.0f;
    float deltaTheta = fabsf(thetaR - thetaL); // angle between edges

    if (leftStableDist > 0 && rightStableDist > 0) {
        // Use law of cosines for wide angles
        if (deltaTheta > (15.0f * M_PI / 180.0f)) { // >15 deg
            widthEstimate = sqrtf(
                leftStableDist * leftStableDist +
                rightStableDist * rightStableDist -
                2.0f * leftStableDist * rightStableDist * cosf(deltaTheta)
            );
        } else {
            // Small angle, approximate with sin projection
            widthEstimate = leftStableDist * sinf(fabsf(thetaL)) +
                            rightStableDist * sinf(fabsf(thetaR));
        }
    }
    //======================================Not working correctly==============================
    //printf("Estimated obstacle width: %.2f cm\n", widthEstimate);

    if (leftStableDist < rightStableDist)
        printf("LEFT side is closer — move RIGHT.\n");
    else if (rightStableDist < leftStableDist)
        printf("RIGHT side is closer — move LEFT.\n");
    else
        printf("Both sides equal distance.\n");
}



// === MAIN ===
int main() {
    stdio_init_all();
    sleep_ms(2000);
    printf("Starting improved edge-detection scan...\n");

    init_servo(SERVO_PIN);
    setupUltrasonicPins(TRIG_PIN, ECHO_PIN);

    while (true) {
        int front = (int)getCmAvg(TRIG_PIN, ECHO_PIN, 5);
        printf("Front: %d cm\n", front);
        if (front > 0 && front <= 10) {
            printf("Obstacle detected (<=10 cm). Running precise scan...\n");
            scan_center_left_right_and_report();
            // avoid immediate re-trigger
            sleep_ms(5000);
        }
        sleep_ms(500);
    }
}
