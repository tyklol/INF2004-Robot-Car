# File: CMakeLists.txt (in the root project folder)

cmake_minimum_required(VERSION 3.13)

# ----- ADD ALL YOUR WIFI/MQTT LOGIC HERE (Moved from drivers) -----
if (DEFINED ENV{WIFI_SSID} AND (NOT WIFI_SSID))
    set(WIFI_SSID $ENV{WIFI_SSID})
    message("Using WIFI_SSID from environment ('${WIFI_SSID}')")
endif()

if (DEFINED ENV{WIFI_PASSWORD} AND (NOT WIFI_PASSWORD))
    set(WIFI_PASSWORD $ENV{WIFI_PASSWORD})
    message("Using WIFI_PASSWORD from environment")
endif()
set(WIFI_SSID "${WIFI_SSID}" CACHE INTERNAL "WiFi SSID for examples")
set(WIFI_PASSWORD "${WIFI_PASSWORD}" CACHE INTERNAL "WiFi password for examples")

if (DEFINED ENV{MQTT_SERVER} AND (NOT MQTT_SERVER))
    set(MQTT_SERVER $ENV{MQTT_SERVER})
    message("Using MQTT_SERVER from environment ('${MQTT_SERVER}')")
endif()
set(MQTT_SERVER "${MQTT_SERVER}" CACHE INTERNAL "MQTT server for examples")
# ... (add the MQTT_USERNAME and MQTT_PASSWORD logic here too) ...


# Find and initialize the Pico SDK
include($ENV{PICO_SDK_PATH}/pico-sdk/pico_sdk_init.cmake)

project(robotic_car_firmware C CXX ASM)
set(PICO_BOARD "pico_w")

# Add the 'config' directory (relative to this file)
# to the global include path.
include_directories(config)

# Initialize the SDK
pico_sdk_init()

include(config/FreeRTOS_Kernel_import.cmake)

# Tell CMake to look in the "drivers" folder for its own CMakeLists.txt
add_subdirectory(drivers)

# --- Add our executable target ---
add_executable(${CMAKE_PROJECT_NAME}
    main.c
)


target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
    MQTT_SERVER=\"${MQTT_SERVER}\"
    NO_SYS=0
)


# --- Link all required libraries ---
target_link_libraries(${CMAKE_PROJECT_NAME}
    PRIVATE
    car_drivers
    pico_stdlib
    FreeRTOS-Kernel  # <-- ADD THIS
    FreeRTOS-Kernel-Heap4
    hardware_sync
    
)

# --- Enable USB/UART stdio for printf() ---
pico_enable_stdio_usb(${CMAKE_PROJECT_NAME} 1)
pico_enable_stdio_uart(${CMAKE_PROJECT_NAME} 0)

# --- Create the .uf2 file for drag-and-drop programming ---
pico_add_extra_outputs(${CMAKE_PROJECT_NAME})