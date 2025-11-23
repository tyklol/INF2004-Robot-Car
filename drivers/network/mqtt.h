#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include "pico/stdlib.h"
#include "lwip/apps/mqtt.h"
#include "ir/ir.h"  // <-- UPDATED PATH
#include "sys_config.h"

#ifndef MQTT_TOPIC_LEN
#define MQTT_TOPIC_LEN 100
#endif

#ifndef MQTT_OUTPUT_RINGBUF_SIZE
#define MQTT_OUTPUT_RINGBUF_SIZE 512
#endif

typedef struct {
    mqtt_client_t* mqtt_client_inst;
    struct mqtt_connect_client_info_t mqtt_client_info;
    char data[MQTT_OUTPUT_RINGBUF_SIZE];
    char topic[MQTT_TOPIC_LEN];
    uint32_t len;
    ip_addr_t mqtt_server_address;
    bool connect_done;
    int subscribe_count;
    bool stop_client;
    char client_id_buf[32];
    char will_topic_buf[MQTT_TOPIC_LEN];
    bool initialized;
} MQTT_CLIENT_DATA_T;

void mqtt_client_init(MQTT_CLIENT_DATA_T *client_data);
bool mqtt_client_try_wifi_connect(MQTT_CLIENT_DATA_T *client_data, uint32_t timeout_ms);
int mqtt_client_start_mqtt(MQTT_CLIENT_DATA_T *client_data);
void mqtt_client_poll(MQTT_CLIENT_DATA_T *client_data);
void mqtt_client_publish_temperature(MQTT_CLIENT_DATA_T *client_data);
void mqtt_pub_imu_raw(MQTT_CLIENT_DATA_T *client_data, float ax, float ay, float az, float mx, float my, float mz);
void mqtt_pub_imu_filtered(MQTT_CLIENT_DATA_T *client_data, float fax, float fay, float faz, float roll, float pitch, float yaw);
void mqtt_pub_telemetry(MQTT_CLIENT_DATA_T *client_data, int g_current_scenario, const char *g_current_ir_state, float g_target_power_left, float g_target_power_right, int32_t g_total_ticks_left, int32_t g_total_ticks_right, uint16_t g_sensor_R, uint32_t g_current_lost_time_ms, int g_current_state);
void mqtt_client_publish(MQTT_CLIENT_DATA_T *client_data, const char *topic, const char *data);
void mqtt_client_subscribe(MQTT_CLIENT_DATA_T *client_data, const char *topic);
void mqtt_client_disconnect(MQTT_CLIENT_DATA_T *client_data);
bool mqtt_client_is_online(MQTT_CLIENT_DATA_T *client_data);

#endif