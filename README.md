# Basics Inverse Kinematics - Hexapod Robot

This project contains the codebase for simulating, controlling, and analyzing the inverse kinematics (IK) and gait generation for a 6-legged (hexapod) robot. The project covers everything from low-level geometric calculations and single-leg simulations to full robot gait coordination and Arduino firmware for hardware control.

## Project Structure

The project is organized into the following main directories:

- **`basics inverse kinematics/` (Root)**: Contains core IK solvers, visualization tools, and main simulation scripts.
- **`connectingjupyter/`**: Scripts and notebooks related to connecting simulations with Jupyter, gait generation, and serial communication with the robot.
- **`firmware/`**: PlatformIO project for the Arduino-based firmware (likely ATmega328P/Uno) controlling the servos via PCA9685 drivers.
- **`hexabot one leg simulation/`**: Dedicated tools for analyzing and simulating the movement of a single leg.
- **`hexabot positions/`**: Scripts for calculating and visualizing specific static poses and leg configurations.

## Root Directory Files

Here is a guide to the key files in this directory:

### Core Kinematics & Simulation
- **`hexabot_sim.py`**: The main simulation script for the hexapod. It visualizes the 3D kinematics of the robot, handles IK calculations for all legs, and can export joint angles/coordinates.
- **`calculate_joints.py`**: A dedicated script for calculating joint angles given target coordinates. It implements the geometric IK math.
- **`find_valid_pose.py`**: A utility to sweep through potential coordinates (Z-heights, X-radii) to find valid, reachable workspace configurations given the physical servo limits.
- **`ik_gui.py`**: A graphical user interface (using Matplotlib widgets) to interactively play with the IK parameters (X, Y, L1, L2) and visualize the resulting arm/leg pose.
- **`inverse_kinematics.py`**: Contains helper functions and classes for the IK logic.
- **`ik simulation hexanbot.cpp`**: A C++ implementation of the IK logic, likely used for reference or porting to the firmware.

### Notebooks & Data
- **`basicsik.ipynb`**: A Jupyter notebook for experimenting with basic IK concepts and converting 3D coordinates to servo angles.
- **`joint_positions.txt`**: Generated output file containing calculated joint angles for specific poses.
- **`exported_coordinates.txt` / `exported_poses.txt`**: Output files from simulations used for debugging or importing into other tools.

## Getting Started

1.  **Dependencies**: Ensure you have Python installed along with `numpy` and `matplotlib`.
2.  **Run Simulation**:
    ```bash
    python hexabot_sim.py
    ```
    This will open a 3D plot showing the robot's configuration.
3.  **Check Reachability**:
    ```bash
    python find_valid_pose.py
    ```
    Use this to determine the optimal body height and stride length for your specific servo limits.

## Key Concepts
- **Inverse Kinematics (IK)**: Calculating the required joint angles (Coxa, Femur, Tibia) to place the foot tip at a specific (X, Y, Z) coordinate.
- **Forward Kinematics (FK)**: Calculating the foot tip position based on the current joint angles.
- **Coordinate System**: Typically, the robot body center is (0,0,0). Leg coordinates are often computed in a local frame relative to the hip and then transformed to the global body frame.
