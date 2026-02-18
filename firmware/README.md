# Hexabot Firmware

This directory contains the firmware source code for the Hexapod robot, designed to run on an Arduino (e.g., Uno/Nano/Mega) using the PlatformIO build system. It handles the low-level servo control using PCA9685 PWM drivers.

## Directory Structure
- **`src/`**: Contains the source code.
    - **`main.cpp`**: The main firmware entry point. It initializes the PCA9685 drivers, implements the IK logic on the microcontroller, and receives commands (or runs a pre-programmed sequence).
- **`include/`**: Header files.
    - **`hexabot_pose.h`**: Definitions for robot poses and potentially stored gait sequences.
- **`platformio.ini`**: Configuration file for PlatformIO, defining the board type, framework (Arduino), and libraries.

## Functionality
- **Servo Driver**: Uses the `Adafruit_PWMServoDriver` library to communicate with PCA9685 modules via I2C.
- **IK Implementation**: The firmware includes an onboard Inverse Kinematics solver (`driveIK` function in `main.cpp`). This allows the robot to accept high-level coordinate targets (Tip X, Y, Z) rather than just raw servo angles, making motion smoother and more adaptable.
- **Service Limits**: Defines hardware safety limits (`HIP_MIN_US`, `KNEE_MAX_US`, etc.) to prevent mechanical damage.

## Usage
1.  Open this folder in VS Code with the PlatformIO extension installed.
2.  Connect your Arduino.
3.  Build and Upload the firmware using the PlatformIO toolbar.
