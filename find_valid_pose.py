
from math import acos, atan2, cos, hypot, sin, degrees, sqrt, pi
import numpy as np

# Geometric constants
L1 = 26.0 # Coxa
L2 = 57.0 # Femur
L3 = 122.0 # Tibia

# Hardware Limits with Offset -90
SERVO_OFFSET = -90.0

# Knee (Femur Pitch)
KNEE_MIN = np.deg2rad(-42.63 + SERVO_OFFSET) # -132.6
KNEE_MAX = np.deg2rad(61.58 + SERVO_OFFSET)  # -28.4

# Ankle (Tibia Pitch relative to Femur? Or global?)
# Standard 3DOF: Tibia angle is relative to Femur.
ANKLE_MIN = np.deg2rad(-61.58 + SERVO_OFFSET) # -151.6
ANKLE_MAX = np.deg2rad(37.89 + SERVO_OFFSET)  # -52.1

def within_limits(value, min_value, max_value):
    return min_value <= value <= max_value

# Standard 3DOF IK Solver (Coxa fixed horizontal)
def solve_ik_3dof(x, z):
    # x is radial distance from Hip origin (cylindrical r)
    # z is vertical distance (down is negative)
    
    # 1. Wrist position relative to Femur joint
    # Coxa L1 is fixed in r-z plane.
    # Femur pivot is at (L1, 0).
    r_target = x - L1
    z_target = z
    
    # Distance from Femur pivot to Tip
    d2 = r_target**2 + z_target**2
    d = sqrt(d2)
    
    if d > L2 + L3:
        return None, "Too Far"
    if d < abs(L2 - L3):
        return None, "Too Close"
        
    # Law of Cosines for Tibia Angle (gamma)
    # c^2 = a^2 + b^2 - 2ab cos(C)
    # d^2 = L2^2 + L3^2 - 2*L2*L3*cos(PI - tibia_angle)
    # cos(PI - tibia) = (L2^2 + L3^2 - d^2) / (2*L2*L3)
    # beta is angle opposite to d? No.
    # We want angle at Knee (between L2 and L3).
    # Let alpha be Femur angle above r_target line.
    # Let beta be angle of Tibia relative to Femur line.
    
    # Interior angle at Knee (between Femur and Tibia segments)
    cos_knee_interior = (L2**2 + L3**2 - d2) / (2 * L2 * L3)
    cos_knee_interior = max(-1.0, min(1.0, cos_knee_interior))
    knee_interior = acos(cos_knee_interior)
    
    # Tibia angle relative to Femur extension:
    # If segments straight, angle is 0.
    # If bent 90, angle is 90.
    # tibia_rel = PI - knee_interior.
    # Usually "Knee" servo measures this deviation from straight.
    # But direction? Hexapod standard: Tibia bends UNDER Femur -> Negative angle?
    # Or Positive?
    # User Limit: [-151, -52].
    # This implies Tibia is High Negative.
    # This matches "Bent Under".
    # So `tibia_angle = -(pi - knee_interior)`.
    tibia_angle = -(pi - knee_interior)
    
    # Femur Angle (alpha)
    # Angle of line d relative to horizon: atan2(z_target, r_target)
    # Angle of Femur relative to line d: from Law of Cosines
    # L3^2 = L2^2 + d^2 - 2*L2*d*cos(femur_offset)
    az = atan2(z_target, r_target)
    cos_femur_offset = (L2**2 + d2 - L3**2) / (2 * L2 * d)
    cos_femur_offset = max(-1.0, min(1.0, cos_femur_offset))
    femur_offset = acos(cos_femur_offset)
    
    # Knee Up solution (Femur higher than line d) -> `az + femur_offset`.
    # Knee Down solution -> `az - femur_offset`.
    # Hexapods usually Knee Up (Spider style).
    femur_angle = az + femur_offset
    
    # Solution 2 (Knee Down?)
    # femur_angle_2 = az - femur_offset
    # tibia_angle_2 = (pi - knee_interior) # Bend other way?
    
    return (femur_angle, tibia_angle), "Success"

print(f"Searching for valid configuration...")
print(f"Limits: Knee [{degrees(KNEE_MIN):.1f}, {degrees(KNEE_MAX):.1f}], Ankle [{degrees(ANKLE_MIN):.1f}, {degrees(ANKLE_MAX):.1f}]")

# Sweep Z and X
best_z = None
max_x_range = 0
best_min_x = 0
best_max_x = 0

print(f"{'Z':<6} {'Min X':<8} {'Max X':<8} {'Range':<8}")
print("-" * 35)

for z in range(-250, -50, 5):
    valid_ros = []
    for r in range(10, 300, 2): # Check r (x) from 10 to 300
        res, msg = solve_ik_3dof(r, z)
        if res:
            femur, tibia = res
            if within_limits(femur, KNEE_MIN, KNEE_MAX) and within_limits(tibia, ANKLE_MIN, ANKLE_MAX):
                valid_ros.append(r)
    
    if valid_ros:
        min_x = min(valid_ros)
        max_x = max(valid_ros)
        x_rng = max_x - min_x
        print(f"{z:<6} {min_x:<8} {max_x:<8} {x_rng:<8}")
        
        if x_rng > max_x_range:
            max_x_range = x_rng
            best_z = z
            best_min_x = min_x
            best_max_x = max_x

print("-" * 35)
if best_z is not None:
    print(f"Best Z: {best_z}")
    print(f"Max Range: {max_x_range} (from {best_min_x} to {best_max_x})")
    mid_x = (best_min_x + best_max_x) / 2
    print(f"Suggested Base X: {mid_x}")
    print(f"Suggested Stride: +/- {max_x_range / 2}")
else:
    print("No valid Z found.")
