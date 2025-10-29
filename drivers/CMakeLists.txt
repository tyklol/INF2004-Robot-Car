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
if (NOT MQTT_SERVER)
    message("Skipping MQTT example as MQTT_SERVER is not defined")
    return()
endif()
set(MQTT_SERVER "${MQTT_SERVER}" CACHE INTERNAL "MQTT server for examples")

if (DEFINED ENV{MQTT_USERNAME} AND (NOT MQTT_USERNAME))
    set(MQTT_USERNAME $ENV{MQTT_USERNAME})
    message("Using MQTT_USERNAME from environment ('${MQTT_USERNAME}')")
endif()
set(MQTT_USERNAME "${MQTT_USERNAME}" CACHE INTERNAL "MQTT user name for examples")
if (DEFINED ENV{MQTT_PASSWORD} AND (NOT MQTT_PASSWORD))
    set(MQTT_PASSWORD $ENV{MQTT_PASSWORD})
    message("Using MQTT_PASSWORD from environment")
endif()
set(MQTT_PASSWORD "${MQTT_PASSWORD}" CACHE INTERNAL "MQTT password for examples")
# Find all source files (.c) in this directory
file(GLOB SOURCES "*.c")

# Create a static library named "car_drivers" from these sources
add_library(car_drivers
    ${SOURCES}
)

# Tell any target that links "car_drivers"
# where to find its public header files (motor.h, ir.h, etc.)
target_include_directories(car_drivers
    PUBLIC
        . # The current directory (drivers/)
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
)

target_compile_definitions(car_drivers PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
    MQTT_SERVER=\"${MQTT_SERVER}\"
)