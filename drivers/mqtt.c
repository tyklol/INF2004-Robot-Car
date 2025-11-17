#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/unique_id.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h" // needed to set hostname
#include "lwip/dns.h"
#include "lwip/altcp_tls.h"
#include <string.h>
#include <ctype.h>
#include <math.h>

#include "mqtt.h"

// Temperature
#ifndef TEMPERATURE_UNITS
#define TEMPERATURE_UNITS 'C' // Set to 'F' for Fahrenheit
#endif

#ifndef MQTT_SERVER
#error Need to define MQTT_SERVER
#endif

// This file includes your client certificate for client server authentication
#ifdef MQTT_CERT_INC
#include MQTT_CERT_INC
#endif

#ifndef MQTT_TOPIC_LEN
#define MQTT_TOPIC_LEN 100
#endif

#ifndef DEBUG_printf
#ifndef NDEBUG
#define DEBUG_printf printf
#else
#define DEBUG_printf(...)
#endif
#endif

#ifndef INFO_printf
#define INFO_printf printf
#endif

#ifndef ERROR_printf
#define ERROR_printf printf
#endif

// how often to measure our temperature
#define TEMP_WORKER_TIME_S 5

// keep alive in seconds
#define MQTT_KEEP_ALIVE_S 60

// qos passed to mqtt_subscribe
// At most once (QoS 0)
// At least once (QoS 1)
// Exactly once (QoS 2)
#define MQTT_SUBSCRIBE_QOS 1
#define MQTT_PUBLISH_QOS 1
#define MQTT_PUBLISH_RETAIN 0

// topic used for last will and testament
#define MQTT_WILL_TOPIC "/online"
#define MQTT_WILL_MSG "0"
#define MQTT_WILL_QOS 1

#ifndef MQTT_DEVICE_NAME
#define MQTT_DEVICE_NAME "pico"
#endif

// Set to 1 to add the client name to topics, to support multiple devices using the same server
#ifndef MQTT_UNIQUE_TOPIC
#define MQTT_UNIQUE_TOPIC 0
#endif

/* Internal helpers (kept static) */
static float read_onboard_temperature(const char unit) {
    adc_select_input(4);
    // Read raw ADC value and convert to voltage
    int raw = adc_read(); // raw 12-bit value 0..4095
    const float conversionFactor = 3.3f / (1 << 12); // 3.3 / 4096
    float voltage = (float)raw * conversionFactor;

    // DEBUG: print raw + voltage once in a while or during development
    // printf("ADC raw=%d voltage=%.3fV\n", raw, voltage);

    float tempC = 27.0f - (voltage - 0.706f) / 0.001721f;

    // Correct unit handling: if unit == 'F' convert, otherwise return C
    if (unit == 'F') {
        return tempC * 9.0f / 5.0f + 32.0f;
    }
    return tempC;
}

static void pub_request_cb(__unused void *arg, err_t err) {
    if (err != 0) {
        printf("pub_request_cb failed %d\n", err);
    }
}

static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name) {
#if MQTT_UNIQUE_TOPIC
    static char full_topic_buf[MQTT_TOPIC_LEN];
    snprintf(full_topic_buf, sizeof(full_topic_buf), "/%s%s", state->mqtt_client_info.client_id, name);
    return full_topic_buf;
#else
    (void)state;
    return name;
#endif
}

static void control_led(MQTT_CLIENT_DATA_T *state, bool on) {
    const char* message = on ? "On" : "Off";
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on ? 1 : 0);
    mqtt_publish(state->mqtt_client_inst, full_topic(state, "/led/state"), message, strlen(message), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
}

/* publish_temperature internal (kept static) */
static void publish_temperature_internal(MQTT_CLIENT_DATA_T *state) {
    static float old_temperature;
    const char *temperature_key = full_topic(state, "/temperature");
    float temperature = read_onboard_temperature(TEMPERATURE_UNITS);

    /* publish on change threshold to avoid noise */
    const float THRESHOLD = 0.2f;
    if (isnan(old_temperature) || fabsf(temperature - old_temperature) > THRESHOLD) {
        old_temperature = temperature;

        char payload[128];
        int len = snprintf(payload, sizeof(payload), "{\"temp_c\": %.2f}", temperature);
        if (len < 0) len = 0;
        if (len >= (int)sizeof(payload)) len = sizeof(payload) - 1;
        payload[len] = '\0';
        mqtt_publish(state->mqtt_client_inst, temperature_key, payload, strlen(payload), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
        printf("Publishing %s to %s\n", payload, temperature_key);
    }
}

static void sub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) {
        ERROR_printf("subscribe request failed: %d\n", err);
        return;
    }
    state->subscribe_count++;
}

static void unsub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) {
        ERROR_printf("unsubscribe request failed: %d\n", err);
        return;
    }
    state->subscribe_count--;
    if (state->subscribe_count < 0) {
        ERROR_printf("unsubscribe resulted in negative subscribe_count (%d)\n", state->subscribe_count);
        state->subscribe_count = 0;
    }

    if (state->subscribe_count <= 0 && state->stop_client) {
        mqtt_disconnect(state->mqtt_client_inst);
    }
}

static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub) {
    mqtt_request_cb_t cb = sub ? sub_request_cb : unsub_request_cb;
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/led"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/print"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/ping"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/exit"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
#if MQTT_UNIQUE_TOPIC
    const char *basic_topic = state->topic + strlen(state->mqtt_client_info.client_id) + 1;
#else
    const char *basic_topic = state->topic;
#endif
    strncpy(state->data, (const char *)data, len);
    state->len = len;
    state->data[len] = '\0';

    printf("Topic: %s, Message: %s\n", state->topic, state->data);
    if (strcmp(basic_topic, "/led") == 0)
    {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0)
            control_led(state, true);
        else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0)
            control_led(state, false);
    } else if (strcmp(basic_topic, "/print") == 0) {
        printf("%.*s\n", len, data);
    } else if (strcmp(basic_topic, "/ping") == 0) {
        char buf[11];
        snprintf(buf, sizeof(buf), "%u", to_ms_since_boot(get_absolute_time()) / 1000);
        mqtt_publish(state->mqtt_client_inst, full_topic(state, "/uptime"), buf, strlen(buf), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    } else if (strcmp(basic_topic, "/exit") == 0) {
        state->stop_client = true;
        sub_unsub_topics(state, false);
    }
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    strncpy(state->topic, topic, sizeof(state->topic));
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        state->connect_done = true;
        sub_unsub_topics(state, true);
        if (state->mqtt_client_info.will_topic) {
            mqtt_publish(state->mqtt_client_inst, state->mqtt_client_info.will_topic, "1", 1, MQTT_WILL_QOS, true, pub_request_cb, state);
        }
    } else if (status == MQTT_CONNECT_DISCONNECTED) {
        if (!state->connect_done) {
            ERROR_printf("MQTT disconnected before initial connect completed\n");
            state->connect_done = false;
        } else {
            INFO_printf("MQTT disconnected\n");
            state->connect_done = false;
        }
    } else {
        ERROR_printf("Unexpected mqtt status: %d\n", status);
        state->connect_done = false;
    }
}

static int start_client(MQTT_CLIENT_DATA_T *state) {
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    const int port = MQTT_TLS_PORT;
    INFO_printf("Using TLS\n");
#else
    const int port = MQTT_PORT;
    INFO_printf("Warning: Not using TLS\n");
#endif

    /* Prevent creating a second client instance without cleaning up the first */
    if (state->mqtt_client_inst) {
        ERROR_printf("start_client: mqtt_client_inst already exists\n");
        return ERR_USE;
    }

    state->mqtt_client_inst = mqtt_client_new();
    if (!state->mqtt_client_inst) {
        ERROR_printf("MQTT client instance creation error\n");
        return ERR_MEM;
    }

    INFO_printf("IP address of this device %s\n", ipaddr_ntoa(&(netif_list->ip_addr)));
    INFO_printf("Connecting to mqtt server at %s\n", ipaddr_ntoa(&state->mqtt_server_address));

    cyw43_arch_lwip_begin();
    int err = mqtt_client_connect(state->mqtt_client_inst,
        &state->mqtt_server_address, port, mqtt_connection_cb, state, &state->mqtt_client_info);
    cyw43_arch_lwip_end();

    if (err != ERR_OK) {
        ERROR_printf("MQTT broker connection error: %d\n", err);
        /* Attempt to cleanly disconnect/free the partially created client */
        cyw43_arch_lwip_begin();
        mqtt_disconnect(state->mqtt_client_inst);
        cyw43_arch_lwip_end();
        state->mqtt_client_inst = NULL;
        state->connect_done = false;
        return err;
    }

    printf("MQTT connection initiated...\n");
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    mbedtls_ssl_set_hostname(altcp_tls_context(state->mqtt_client_inst->conn), MQTT_SERVER);
#endif

    cyw43_arch_lwip_begin();
    mqtt_set_inpub_callback(state->mqtt_client_inst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, state);
    cyw43_arch_lwip_end();

    return ERR_OK;
}

/* DNS callback used by mqtt_client_start_mqtt() */
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T*)arg;
    if (ipaddr) {
        state->mqtt_server_address = *ipaddr;

        /* If an existing client instance is present, ensure it is cleaned up
         * before we attempt to start a new one.
         */
        if (state->mqtt_client_inst && !mqtt_client_is_connected(state->mqtt_client_inst)) {
            INFO_printf("dns_found: cleaning up previous mqtt_client_inst before restart\n");
            cyw43_arch_lwip_begin();
            mqtt_disconnect(state->mqtt_client_inst);
            cyw43_arch_lwip_end();
            state->mqtt_client_inst = NULL;
            state->connect_done = false;
            state->subscribe_count = 0;
        }

        int rc = start_client(state);
        if (rc != ERR_OK) {
            ERROR_printf("start_client failed after DNS for %s: %d\n", hostname ? hostname : "(null)", rc);
            state->connect_done = false; // ensure state reflects not connected
        }
    } else {
        ERROR_printf("DNS resolution failed for %s\n", hostname ? hostname : "(null)");
        state->connect_done = false;
    }
}

/* --- Public API implementations --- */

void mqtt_client_init(MQTT_CLIENT_DATA_T *client_data) {
    if (!client_data) return;

    /* Zero and set defaults */
    memset(client_data, 0, sizeof(*client_data));

    adc_set_temp_sensor_enabled(true);

    // if (cyw43_arch_init()) {
    //     ERROR_printf("Failed to initialize CYW43\n");
    //     client_data->initialized = false;
    //     return;
    // }

    /* generate client id: MQTT_DEVICE_NAME + board unique id */
    char unique_id_buf[5];
    pico_get_unique_board_id_string(unique_id_buf, sizeof(unique_id_buf));
    for (int i = 0; i < (int)sizeof(unique_id_buf) - 1; ++i) {
        unique_id_buf[i] = tolower((unsigned char)unique_id_buf[i]);
    }
    snprintf(client_data->client_id_buf, sizeof(client_data->client_id_buf), "%s%s", MQTT_DEVICE_NAME, unique_id_buf);
    client_data->mqtt_client_info.client_id = client_data->client_id_buf;
    client_data->mqtt_client_info.keep_alive = MQTT_KEEP_ALIVE_S;

#if defined(MQTT_USERNAME) && defined(MQTT_PASSWORD)
    client_data->mqtt_client_info.client_user = MQTT_USERNAME;
    client_data->mqtt_client_info.client_pass = MQTT_PASSWORD;
#else
    client_data->mqtt_client_info.client_user = NULL;
    client_data->mqtt_client_info.client_pass = NULL;
#endif

    strncpy(client_data->will_topic_buf, full_topic(client_data, MQTT_WILL_TOPIC), sizeof(client_data->will_topic_buf));
    client_data->mqtt_client_info.will_topic = client_data->will_topic_buf;
    client_data->mqtt_client_info.will_msg = MQTT_WILL_MSG;
    client_data->mqtt_client_info.will_qos = MQTT_WILL_QOS;
    client_data->mqtt_client_info.will_retain = true;

#if LWIP_ALTCP && LWIP_ALTCP_TLS
    // TLS enabled
#ifdef MQTT_CERT_INC
    static const uint8_t ca_cert[] = TLS_ROOT_CERT;
    static const uint8_t client_key[] = TLS_CLIENT_KEY;
    static const uint8_t client_cert[] = TLS_CLIENT_CERT;
    client_data->mqtt_client_info.tls_config = altcp_tls_create_config_client_2wayauth(ca_cert, sizeof(ca_cert),
            client_key, sizeof(client_key), NULL, 0, client_cert, sizeof(client_cert));
#if ALTCP_MBEDTLS_AUTHMODE != MBEDTLS_SSL_VERIFY_REQUIRED
    WARN_printf("Warning: tls without verification is insecure\n");
#endif
#else
    client_data->mqtt_client_info.tls_config = altcp_tls_create_config_client(NULL, 0);
    WARN_printf("Warning: tls without a certificate is insecure\n");
#endif
#endif

    client_data->initialized = true;
}

/* Blocking WiFi connect helper (returns true on success) */
bool mqtt_client_try_wifi_connect(MQTT_CLIENT_DATA_T *client_data, uint32_t timeout_ms) {
    if (!client_data || !client_data->initialized) return false;
    cyw43_arch_enable_sta_mode();
    /* This function blocks for up to timeout_ms, so call only if you accept blocking */
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, timeout_ms)) {
        return false;
    }
    return true;
}

/* Start MQTT connection (performs DNS lookup). Non-blocking DNS via callback. */
int mqtt_client_start_mqtt(MQTT_CLIENT_DATA_T *client_data) {
    if (!client_data || !client_data->initialized) return ERR_VAL;

    cyw43_arch_lwip_begin();
    int err = dns_gethostbyname(MQTT_SERVER, &client_data->mqtt_server_address, dns_found, client_data);
    cyw43_arch_lwip_end();

    if (err == ERR_OK) {
        // DNS resolved synchronously; start_client should be invoked now.
        // Call start_client explicitly to ensure consistent behaviour and capture its return value.
        int rc = start_client(client_data);
        if (rc != ERR_OK) {
            ERROR_printf("mqtt_client_start_mqtt: start_client returned %d\n", rc);
            return rc;
        }
        return ERR_OK;
    } else if (err == ERR_INPROGRESS) {
        // DNS will complete asynchronously via dns_found()
        INFO_printf("mqtt_client_start_mqtt: DNS lookup in progress\n");
        return ERR_INPROGRESS;
    } else {
        ERROR_printf("mqtt_client_start_mqtt: dns_gethostbyname failed immediately: %d\n", err);
        return err;
    }
}

/* Poll function: call from main loop to let cyw43/lwip do background work. */
void mqtt_client_poll(MQTT_CLIENT_DATA_T *client_data) {
    (void)client_data;
    /* Non-blocking poll step */
    cyw43_arch_poll();
}

/* Publish temperature wrapper for main loop usage */
void mqtt_client_publish_temperature(MQTT_CLIENT_DATA_T *client_data) {
    if (!client_data || !client_data->mqtt_client_inst) return;
    publish_temperature_internal(client_data);
}

void mqtt_pub_imu_raw(MQTT_CLIENT_DATA_T *client_data, float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float mx, float my, float mz) {
    char buf[192];
    int n = snprintf(buf, sizeof(buf),
        "{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f}",
        ax, ay, az);
    if (n <= 0 || n >= (int)sizeof(buf)) {
        printf("mqtt_pub_imu_raw: snprintf error or buffer too small\n");
    } else {
        mqtt_client_publish(client_data, "robot/imu/raw", buf);
    }
}

void mqtt_pub_imu_filtered(MQTT_CLIENT_DATA_T *client_data, float fax, float fay, float faz,
                           float roll, float pitch, float yaw) {
    char buf[192];
    int n = snprintf(buf, sizeof(buf),
        "{\"fax\":%.3f,\"fay\":%.3f,\"faz\":%.3f,"
        "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f}",
        fax, fay, faz, roll, pitch, yaw);
    if (n <= 0 || n >= (int)sizeof(buf)) {
        printf("mqtt_pub_imu_filtered: snprintf error or buffer too small\n");
    } else {
        mqtt_client_publish(client_data, "robot/imu/filtered", buf);
    }
}

void mqtt_pub_speed(MQTT_CLIENT_DATA_T *client_data, int g_current_scenario, const char *g_current_ir_state,
                       float g_target_power_left, float g_target_power_right,
                       float local_distance_cm, uint32_t g_current_lost_time_ms, int32_t g_total_ticks_left, int32_t g_total_ticks_right) {
    char buf[192];
    int n = snprintf(buf, sizeof(buf),
        "{\"SCEN\": %d, \"IR_State\": \"%s\", \"Pwr_L\": %.1f, \"Pwr_R\": %.1f, \"Dist\": %.1fcm, \"L_ms\": %u, \"Ticks_L\": %d, \"Ticks_R\": %d}",
         g_current_scenario, g_current_ir_state,
                       g_target_power_left, g_target_power_right,
                       local_distance_cm, g_current_lost_time_ms, g_total_ticks_left, g_total_ticks_right);
    if (n <= 0 || n >= (int)sizeof(buf)) {
        printf("mqtt_pub_speed: snprintf error or buffer too small\n");
    } else {
        mqtt_client_publish(client_data, "robot/speed", buf);
    }
}

void mqtt_pub_obstacle(MQTT_CLIENT_DATA_T *client_data, float distance_cm, float width_cm) {
    char buf[128];
    int n = snprintf(buf, sizeof(buf),
        "{\"distance_cm\": %.1f, \"width_cm\": %.1f}",
        distance_cm, width_cm);
    if (n <= 0 || n >= (int)sizeof(buf)) {
        printf("mqtt_pub_obstacle: snprintf error or buffer too small\n");
    } else {
        mqtt_client_publish(client_data, "robot/obstacle", buf);
    }
}
/* Thin wrappers for publish/subscribe/disconnect */

void mqtt_client_publish(MQTT_CLIENT_DATA_T *client_data, const char *topic, const char *data) {
    if (!client_data || !client_data->mqtt_client_inst) return;
    mqtt_publish(client_data->mqtt_client_inst, topic, data, strlen(data), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, client_data);
}

void mqtt_client_subscribe(MQTT_CLIENT_DATA_T *client_data, const char *topic) {
    if (!client_data || !client_data->mqtt_client_inst) return;
    mqtt_sub_unsub(client_data->mqtt_client_inst, topic, MQTT_SUBSCRIBE_QOS, sub_request_cb, client_data, true);
}

void mqtt_client_disconnect(MQTT_CLIENT_DATA_T *client_data) {
    if (!client_data) return;

    /* If no client instance, just clear flags */
    if (!client_data->mqtt_client_inst) {
        client_data->stop_client = true;
        client_data->connect_done = false;
        client_data->subscribe_count = 0;
        return;
    }

    /* Request graceful disconnect and clear local state.
     * mqtt_disconnect must be called while lwIP is active.
     */
    client_data->stop_client = true;
    cyw43_arch_lwip_begin();
    mqtt_disconnect(client_data->mqtt_client_inst);
    cyw43_arch_lwip_end();

    /* Clear pointers/flags immediately so caller can attempt restart.
     * The actual cleanup in lwIP may finish asynchronously; caller should
     * call mqtt_client_poll() a few times to let lwIP flush timeouts.
     */
    client_data->mqtt_client_inst = NULL;
    client_data->connect_done = false;
    client_data->subscribe_count = 0;
}

bool mqtt_client_is_online(MQTT_CLIENT_DATA_T *client_data) {
    // printf("Checking if MQTT is online...\n");
    if (!client_data) return false;
    bool connected = client_data->connect_done && client_data->mqtt_client_inst && mqtt_client_is_connected(client_data->mqtt_client_inst);
    // printf("MQTT is %s\n", connected ? "online" : "offline");
    return connected;
}
