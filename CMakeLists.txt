cmake_minimum_required(VERSION 3.13)

# Include the Pico SDK import script
include(pico_sdk_import.cmake)

# Project name
project(robot_car C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# Add the main executable
add_executable(robot_car
    main.c
)

# Link libraries
target_link_libraries(robot_car
    pico_stdlib
    pico_cyw43_arch_none
    hardware_pwm
    hardware_gpio
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(robot_car 1)
pico_enable_stdio_uart(robot_car 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(robot_car)
