# File: drivers/CMakeLists.txt

# Find all source files (.c) in this directory
file(GLOB SOURCES "*.c")

# Create a static library named "car_drivers" from these sources
add_library(car_drivers
    ${SOURCES}
)
# ----- REMOVE "add_subdirectory(freertos)" -----


# Tell any target that links "car_drivers"
# where to find its public header files (motor.h, ir.h, etc.)
target_include_directories(car_drivers
    PUBLIC
        . # The current directory (drivers/)
)
# Ensure LWIP headers are visible to consumers of car_drivers
target_include_directories(car_drivers
    PUBLIC
        ${PICO_SDK_PATH}/lib/lwip/src/include
)

# Link this library against all the Pico SDK libraries it needs.
target_link_libraries(car_drivers
    PUBLIC
    pico_stdlib
    hardware_pwm     # For motors
    hardware_gpio    # For buttons, etc.
    hardware_i2c     # For IMU (future)
    hardware_adc     # For IR sensors
    hardware_timer   # For ultrasonic (future)
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_mqtt
    pico_mbedtls
    pico_lwip_mbedtls
    hardware_sync
)

# ----- MOVE all the credential logic (WIFI_SSID, etc.) to your ROOT CMakeLists.txt -----
# ----- We still need the definitions for the C code in this library -----

target_compile_definitions(car_drivers PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
    MQTT_SERVER=\"${MQTT_SERVER}\"
)