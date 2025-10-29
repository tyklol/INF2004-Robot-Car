# File: CMakeLists.txt (in the root project folder)

cmake_minimum_required(VERSION 3.13)

# Find and initialize the Pico SDK
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(robotic_car_firmware C CXX ASM)
set(PICO_BOARD "pico_w")
# Initialize the SDK
pico_sdk_init()

# Tell CMake to look in the "drivers" folder for its own CMakeLists.txt
# This will create our "car_drivers" library
add_subdirectory(drivers)

# --- Add our executable target ---
# The executable is built *only* from main.c
add_executable(${CMAKE_PROJECT_NAME}
    main.c
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
    MQTT_SERVER=\"${MQTT_SERVER}\"
)

# --- Link all required libraries ---
# We link our main executable against the "car_drivers" library.
# CMake is smart enough to also link all the SDK libraries 
# that "car_drivers" depends on (like hardware_pwm, etc.).
target_link_libraries(${CMAKE_PROJECT_NAME}
    PRIVATE
    car_drivers
    pico_stdlib # main.c also needs stdlib for printf/sleep
)

# --- Enable USB/UART stdio for printf() ---
pico_enable_stdio_usb(${CMAKE_PROJECT_NAME} 1)
pico_enable_stdio_uart(${CMAKE_PROJECT_NAME} 0)

# --- Create the .uf2 file for drag-and-drop programming ---
pico_add_extra_outputs(${CMAKE_PROJECT_NAME})