# Find all source files (.c) in this directory AND subdirectories
file(GLOB_RECURSE SOURCES "*.c")

# Create the library
add_library(car_drivers ${SOURCES})

# Include directories for the drivers themselves
# This allows main.c to do #include "motor/motor.h"
target_include_directories(car_drivers
    PUBLIC
        .           # Allow includes relative to drivers/
)

# Allow drivers to include "sys_config.h" from root include/
target_include_directories(car_drivers PRIVATE ../include)
target_include_directories(car_drivers PRIVATE ../config)

# Include LWIP headers
target_include_directories(car_drivers
    PUBLIC
        ${PICO_SDK_PATH}/lib/lwip/src/include
)

# Link Dependencies
target_link_libraries(car_drivers
    PUBLIC
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_i2c
    hardware_adc
    hardware_timer
    hardware_pio
    pico_cyw43_arch_lwip_sys_freertos
    pico_lwip_mqtt
    pico_mbedtls
    pico_lwip_mbedtls
    hardware_sync
    FreeRTOS-Kernel-Heap4
)

# Pass credentials to drivers (needed for MQTT/Wifi driver)
target_compile_definitions(car_drivers PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
    MQTT_SERVER=\"${MQTT_SERVER}\"
    NO_SYS=0
)