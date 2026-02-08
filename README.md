# Basics of Inverse Kinematics

This repo is a compact study notebook for planar 2-link inverse kinematics (IK). It includes a small script to compute angles for a target point, a slider GUI to explore targets and link lengths interactively, and a scratch notebook where equations and plots were tested.

## Files
- inverse_kinematics.py: compute angles for a target and plot the arm
- ik_gui.py: interactive sliders for X, Y, L1, L2 with live angle readout
- basicsik.ipynb: scratch notebook experiments

## Run
```bash
python inverse_kinematics.py
python ik_gui.py
```

## Math Summary (2-Link Planar)
Given link lengths L1 and L2 and a target point (x, y):

- Distance to target: r = sqrt(x^2 + y^2)
- Reachable if |L1 - L2| <= r <= L1 + L2
- Elbow angle:
	- cos(theta2) = (r^2 - L1^2 - L2^2) / (2 L1 L2)
- Shoulder angle:
	- phi = atan2(y, x)
	- psi = atan2(L2 sin(theta2), L1 + L2 cos(theta2))
	- theta1 = phi - psi

Elbow-up vs elbow-down is handled by the sign of theta2.

## What The GUI Shows
- Arm pose for the current sliders
- Red target point
- Outer reach circle: L1 + L2
- Inner reach circle: |L1 - L2|
- Angle readouts in radians and degrees

## Tips
- If the target is unreachable, move the target inside the outer circle and outside the inner circle.
- If the arm flips the other way, negate theta2 to switch elbow configuration.
- Use small slider steps to avoid jitter near the reach limits.

## Troubleshooting
- If a plot window hangs, close it and run the script again.
- If you see invalid value warnings for acos, the target is unreachable or numeric rounding pushed the cosine outside [-1, 1].

## Next Ideas
- Add joint limits and enforce them in IK
- Add a third link or a wrist angle
- Animate a path instead of a single target
