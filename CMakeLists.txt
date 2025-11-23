cmake_minimum_required(VERSION 3.13)

# ----- WIFI/MQTT CREDENTIALS -----
if (DEFINED ENV{WIFI_SSID} AND (NOT WIFI_SSID))
    set(WIFI_SSID $ENV{WIFI_SSID})
endif()
if (DEFINED ENV{WIFI_PASSWORD} AND (NOT WIFI_PASSWORD))
    set(WIFI_PASSWORD $ENV{WIFI_PASSWORD})
endif()
set(WIFI_SSID "${WIFI_SSID}" CACHE INTERNAL "WiFi SSID")
set(WIFI_PASSWORD "${WIFI_PASSWORD}" CACHE INTERNAL "WiFi Password")

if (DEFINED ENV{MQTT_SERVER} AND (NOT MQTT_SERVER))
    set(MQTT_SERVER $ENV{MQTT_SERVER})
endif()
set(MQTT_SERVER "${MQTT_SERVER}" CACHE INTERNAL "MQTT Server IP")

# ----- SDK INIT -----
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(robotic_car_firmware C CXX ASM)
set(PICO_BOARD "pico_w")

pico_sdk_init()

# ----- INCLUDES -----
# Add 'config' and 'include' to global search paths
include_directories(config)
include_directories(include)

# Import FreeRTOS
include(config/FreeRTOS_Kernel_import.cmake)

# Add Drivers Subdirectory
add_subdirectory(drivers)

# ----- EXECUTABLE -----
add_executable(${CMAKE_PROJECT_NAME}
    main.c
)

# ----- DEFINES -----
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
    MQTT_SERVER=\"${MQTT_SERVER}\"
    NO_SYS=0
)

# ----- LIBRARIES -----
target_link_libraries(${CMAKE_PROJECT_NAME}
    PRIVATE
    car_drivers          # Our custom library
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_timer
    hardware_adc
    hardware_i2c
    hardware_pio
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    hardware_sync
)

# ----- STDIO -----
pico_enable_stdio_usb(${CMAKE_PROJECT_NAME} 1)
pico_enable_stdio_uart(${CMAKE_PROJECT_NAME} 0)

pico_add_extra_outputs(${CMAKE_PROJECT_NAME})