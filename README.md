# INF2004-Robot-Car

A robot car project built with the Raspberry Pi Pico W microcontroller. This project demonstrates motor control, GPIO operations, and embedded C programming for robotics applications.

## Hardware Requirements

- Raspberry Pi Pico W
- Motor driver module (e.g., L298N or similar)
- DC motors (2x or 4x)
- Robot car chassis
- Power supply (batteries)
- USB cable for programming

## Software Requirements

- [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
- CMake (version 3.13 or higher)
- ARM GCC compiler (arm-none-eabi-gcc)

## Setup Instructions

### 1. Install the Pico SDK

Follow the official Raspberry Pi Pico SDK installation guide:
- [Getting Started with Raspberry Pi Pico](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf)

Set the `PICO_SDK_PATH` environment variable:
```bash
export PICO_SDK_PATH=/path/to/pico-sdk
```

### 2. Build the Project

```bash
mkdir build
cd build
cmake ..
make
```

This will generate a `robot_car.uf2` file in the build directory.

### 3. Flash the Pico W

1. Hold the BOOTSEL button on the Pico W while connecting it to your computer via USB
2. The Pico W will appear as a mass storage device
3. Copy the `robot_car.uf2` file to the Pico W
4. The Pico W will automatically reboot and run the program

## Pin Configuration

The default pin configuration is:

- **Motor Left Forward**: GPIO 2
- **Motor Left Backward**: GPIO 3
- **Motor Right Forward**: GPIO 4
- **Motor Right Backward**: GPIO 5

You can modify these pins in `main.c` to match your hardware setup.

## Features

- Basic motor control (forward, backward, turn left, turn right, stop)
- LED status indicator
- USB serial output for debugging
- Modular code structure for easy expansion

## Project Structure

```
INF2004-Robot-Car/
├── CMakeLists.txt          # Main CMake configuration
├── pico_sdk_import.cmake   # Pico SDK import script
├── main.c                  # Main program source code
├── .gitignore              # Git ignore file
└── README.md               # This file
```

## Usage

Once flashed, the robot car will run a demo program that:
1. Moves forward for 2 seconds
2. Stops for 1 second
3. Turns right for 1 second
4. Stops for 1 second
5. Moves backward for 2 seconds
6. Stops for 1 second
7. Turns left for 1 second
8. Repeats the cycle

The onboard LED will blink to indicate the program is running.

## Extending the Project

You can extend this project by:
- Adding ultrasonic sensors for obstacle detection
- Implementing line-following capabilities
- Adding Wi-Fi connectivity for remote control
- Implementing PID control for smooth motor operation
- Adding encoders for precise movement tracking

## Troubleshooting

- **Build errors**: Ensure `PICO_SDK_PATH` is set correctly
- **Upload errors**: Make sure to hold BOOTSEL while connecting the Pico W
- **Motor not working**: Check your motor driver connections and power supply
- **No output**: Verify USB cable supports data transfer (not just charging)

## License

This project is open source and available for educational purposes.

## References

- [Raspberry Pi Pico Documentation](https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html)
- [Pico SDK Documentation](https://raspberrypi.github.io/pico-sdk-doxygen/)
- [CMake Documentation](https://cmake.org/documentation/)