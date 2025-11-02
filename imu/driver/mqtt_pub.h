#pragma once
#include <stdbool.h>

bool mqtt_start(void);                       // connect to broker (non-blocking)
bool mqtt_is_connected(void);                // true when session up
bool mqtt_publish_utf8(const char* topic, const char* payload);

bool mqtt_pub_imu_raw(float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float mx, float my, float mz);

bool mqtt_pub_imu_filtered(float fax, float fay, float faz,
                           float roll, float pitch, float yaw);
