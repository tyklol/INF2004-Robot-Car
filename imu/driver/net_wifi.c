#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include <stdio.h>

#ifndef WIFI_SSID
#error "WIFI_SSID not defined (set in CMake)"
#endif
#ifndef WIFI_PASSWORD
#error "WIFI_PASSWORD not defined (set in CMake)"
#endif

bool wifi_init_and_connect(void) {
    if (cyw43_arch_init()) {
        printf("[WIFI] cyw43 init failed\n");
        return false;
    }
    cyw43_arch_enable_sta_mode();
    printf("[WIFI] Connecting to SSID: %s\n", WIFI_SSID);

    int rc = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000
    );
    if (rc) {
        printf("[WIFI] connect failed (%d)\n", rc);
        return false;
    }
    printf("[WIFI] connected ✅\n");
    return true;
}
