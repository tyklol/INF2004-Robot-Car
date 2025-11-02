cmake_minimum_required(VERSION 3.13)

# ---------- Secrets / Network ----------
set(WIFI_SSID     "vhse" CACHE STRING "")
set(WIFI_PASSWORD "pass"       CACHE STRING "")
set(MQTT_SERVER   "192.168.0.211"     CACHE STRING "")
add_compile_definitions(
    WIFI_SSID="${WIFI_SSID}"
    WIFI_PASSWORD="${WIFI_PASSWORD}"
    MQTT_SERVER="${MQTT_SERVER}"
)

# ---------- SDK / Board (before import/init) ----------
set(PICO_SDK_PATH "C:/Users/zongy/Desktop/pico/pico-sdk")
set(PICO_BOARD pico_w)
include(pico_sdk_import.cmake)

project(gyro C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# ---------- Sources ----------
add_executable(gyro
    src/main.c
    src/imu_heading.c 
    src/net_wifi.c
    src/mqtt_pub.c
    src/filter.c
    #driver.c
)

# Headers in src/
target_include_directories(gyro PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    # Safety net; remove later once things are stable:
    ${PICO_SDK_PATH}/src/rp2_common/pico_cyw43_arch/include
    ${PICO_SDK_PATH}/src/rp2_common/pico_lwip/include  
)

# ---------- Linking ----------
target_link_libraries(gyro
    pico_stdlib
    hardware_i2c
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_mqtt
    m
    hardware_pwm 
    pico_lwip   
)

# ---------- STDIO ----------
pico_enable_stdio_usb(gyro 1)
pico_enable_stdio_uart(gyro 0)

# ---------- Extra outputs (UF2, etc.) ----------
pico_add_extra_outputs(gyro)
