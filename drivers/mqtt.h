#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include "pico/stdlib.h"
#include "lwip/apps/mqtt.h"

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

    /* internal buffers that must remain valid while MQTT runs */
    char client_id_buf[32];
    char will_topic_buf[MQTT_TOPIC_LEN];

    /* optional state flags */
    bool initialized;
} MQTT_CLIENT_DATA_T;

/* Initialize mqtt client internals and hardware used by the client.
 * This will NOT attempt to connect to WiFi or the MQTT broker.
 * Call mqtt_client_try_wifi_connect() and mqtt_client_start_mqtt()
 * when you want to connect.
 */
void mqtt_client_init(MQTT_CLIENT_DATA_T *client_data);

/* Blocking attempt to connect to WiFi (returns true on success). */
bool mqtt_client_try_wifi_connect(MQTT_CLIENT_DATA_T *client_data, uint32_t timeout_ms);

/* Start MQTT connection (performs DNS and starts client). Returns lwIP err_t:
 * ERR_OK if DNS resolved immediately and start_client invoked,
 * ERR_INPROGRESS if DNS lookup will complete asynchronously via callback,
 * otherwise an lwIP error code.
 */
int mqtt_client_start_mqtt(MQTT_CLIENT_DATA_T *client_data);

/* Non-blocking poll. Call regularly from your main loop to drive CYW43 / lwIP
 * processing (this avoids blocking the rest of your application).
 */
void mqtt_client_poll(MQTT_CLIENT_DATA_T *client_data);

/* Publish current temperature if connected. Safe to call from main loop. */
void mqtt_client_publish_temperature(MQTT_CLIENT_DATA_T *client_data);

void mqtt_publish_imu_raw(MQTT_CLIENT_DATA_T *client_data, float ax, float ay, float az, 
    float gx, float gy, float gz, float mx, float my, float mz);

void mqtt_publish_imu_filtered(MQTT_CLIENT_DATA_T *client_data, float fax, float fay, float faz,
                           float roll, float pitch, float yaw);
                           
void mqtt_publish_speed(MQTT_CLIENT_DATA_T *client_data, const char *g_current_ir_state,
    float g_target_speed_left_tps, float g_current_speed_left_tps, float g_final_power_left,
    float g_target_speed_right_tps, float g_current_speed_right_tps, float g_final_power_right,
    uint16_t g_sensor_R);

void mqtt_publish_obstacle(MQTT_CLIENT_DATA_T *client_data, float distance, float width, const char* side);

void mqtt_publish_state(MQTT_CLIENT_DATA_T *client_data, const char* mode, const char* state, const char* phase);

void mqtt_publish_line(MQTT_CLIENT_DATA_T *client_data, int on_line, float error, float effort);

void mqtt_publish_barcode(MQTT_CLIENT_DATA_T *client_data, const char* command);

/* Publish arbitrary topic/data (wrapper). */
void mqtt_client_publish(MQTT_CLIENT_DATA_T *client_data, const char *topic, const char *data);

/* Subscribe to a topic (wrapper). */
void mqtt_client_subscribe(MQTT_CLIENT_DATA_T *client_data, const char *topic);

/* Request graceful disconnect. */
void mqtt_client_disconnect(MQTT_CLIENT_DATA_T *client_data);

/* Returns true if MQTT is connected and ready. */
bool mqtt_client_is_online(MQTT_CLIENT_DATA_T *client_data);

#endif // MQTT_CLIENT_H