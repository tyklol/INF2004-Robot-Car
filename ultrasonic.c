#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "ultrasonic.h"

int timeout = 26100;

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

    absolute_time_t endTime = get_absolute_time();
    return absolute_time_diff_us(startTime, endTime);
}

uint64_t getCm(uint trigPin, uint echoPin) {
    uint64_t pulseLength = getPulse(trigPin, echoPin);
    return pulseLength / 29 / 2;
}

uint64_t getInch(uint trigPin, uint echoPin) {
    uint64_t pulseLength = getPulse(trigPin, echoPin);
    return pulseLength / 74 / 2;
}

int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("Ultrasonic test starting...\n");
    int trigPin = 0;
    int echoPin = 1;

    setupUltrasonicPins(trigPin, echoPin);

    while (true) {
        int distanceCM = (int)getCm(trigPin, echoPin);
        printf("Distance: %d cm\n", distanceCM);
        sleep_ms(1000);
    }
}