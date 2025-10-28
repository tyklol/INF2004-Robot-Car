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
)