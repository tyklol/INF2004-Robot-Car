#include "mqtt_pub.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include <stdio.h>
#include <string.h>

#ifndef MQTT_SERVER
#error "MQTT_SERVER not defined (set in CMake)"
#endif

static mqtt_client_t *client = NULL;
static bool connected = false;

static void mqtt_pub_cb(void *arg, err_t result) {
    if (result != ERR_OK) printf("[MQTT] publish err=%d\n", result);
}

static void mqtt_connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status) {
    (void)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        connected = true;
        printf("[MQTT] connected ✅\n");
    } else {
        connected = false;
        printf("[MQTT] connection lost (status=%d) — will retry\n", status);
    }
}

bool mqtt_start(void) {
    ip_addr_t broker_ip;
    if (!ipaddr_aton(MQTT_SERVER, &broker_ip)) {
        printf("[MQTT] invalid broker IP: %s\n", MQTT_SERVER);
        return false;
    }
    if (!client) client = mqtt_client_new();
    if (!client) {
        printf("[MQTT] mqtt_client_new failed\n");
        return false;
    }
    struct mqtt_connect_client_info_t ci = {
        .client_id = "pico_w_robot",
        .keep_alive = 30
    };
    err_t err = mqtt_client_connect(client, &broker_ip, 1883, mqtt_connection_cb, NULL, &ci);
    if (err != ERR_OK) {
        printf("[MQTT] connect err=%d\n", err);
        return false;
    }
    printf("[MQTT] connecting to %s:1883 …\n", MQTT_SERVER);
    return true;
}

bool mqtt_is_connected(void) { return connected; }

bool mqtt_publish_utf8(const char* topic, const char* payload) {
    if (!connected || !client) return false;
    err_t err = mqtt_publish(client, topic, payload, strlen(payload),
                             0 /*qos*/, 0 /*retain*/, mqtt_pub_cb, NULL);
    if (err != ERR_OK) {
        printf("[MQTT] publish err=%d\n", err);
        return false;
    }
    return true;
}

bool mqtt_pub_imu_raw(float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float mx, float my, float mz) {
    char buf[192];
    int n = snprintf(buf, sizeof(buf),
        "{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
        "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,"
        "\"mx\":%.3f,\"my\":%.3f,\"mz\":%.3f}",
        ax, ay, az, gx, gy, gz, mx, my, mz);
    if (n <= 0 || n >= (int)sizeof(buf)) return false;
    return mqtt_publish_utf8("robot/imu/raw", buf);
}

bool mqtt_pub_imu_filtered(float fax, float fay, float faz,
                           float roll, float pitch, float yaw) {
    char buf[192];
    int n = snprintf(buf, sizeof(buf),
        "{\"fax\":%.3f,\"fay\":%.3f,\"faz\":%.3f,"
        "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f}",
        fax, fay, faz, roll, pitch, yaw);
    if (n <= 0 || n >= (int)sizeof(buf)) return false;
    return mqtt_publish_utf8("robot/imu/filtered", buf);
}
