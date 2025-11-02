add_library(pico_ultrasonic ultrasonic.c ultrasonic.h)
target_link_libraries(pico_ultrasonic pico_stdlib hardware_gpio hardware_timer)
target_include_directories(pico_ultrasonic PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

add_library(pico_ultrasonic_object ultrasonic.c ultrasonic.h ultrasonicObject.cpp ultrasonicObject.h)
target_link_libraries(pico_ultrasonic_object pico_stdlib hardware_gpio hardware_timer)
target_include_directories(pico_ultrasonic_object PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(ultrasonic_test
    ultrasonic.c           # or main.cpp
)

target_link_libraries(ultrasonic_test
    pico_stdlib
    pico_ultrasonic   # <-- choose which one to use
)
pico_enable_stdio_usb(ultrasonic_test 1)
pico_enable_stdio_uart(ultrasonic_test 0)
pico_add_extra_outputs(ultrasonic_test)