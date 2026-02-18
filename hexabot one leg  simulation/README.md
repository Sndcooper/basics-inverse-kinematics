# Hexabot One Leg Simulation

This directory contains specialized scripts and notebooks for simulating and testing the kinematics of a single leg. Isolating one leg is crucial for debugging IK math and validating reachability before controlling the full 6-legged robot.

## Key Files

### Simulations
- **`leg4_sim.py`**: A focused simulation for "Leg 4" (typically the middle-left or rear-left leg, depending on the index convention). It handles the specific mounting angle and coordinate transformations for that leg.
- **`oneLeg.py`**: A generic class or script for single-leg IK/FK calculations.
- **`onelegsim.ipynb`**: A Jupyter notebook for interactive visualization and testing of single-leg movements.
- **`onelegextended.py`**: Likely containing extended logic or testing for extreme range of motion (extended leg positions).

### Output Data
- **`one_leg_coordinates.txt`**: Exported coordinate data from the single-leg simulations.
- **`one_leg_poses_extended.txt`**: Data regarding specific extended poses.

## Usage
Use these scripts to:
1.  **Debug IK Math**: If a leg isn't moving as expected, isolate it here to inspect the math without the complexity of the full robot body.
2.  **Visualization**: Run `leg4_sim.py` or the notebooks to see a 3D representation of the leg and its workspace.
