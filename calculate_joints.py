from math import acos, atan2, cos, hypot, sin, pi
import numpy as np

# Copied constants and functions from hexabot_sim.py
L1 = 26.0
L2 = 57.0
L3 = 122.0
HIP_RADIUS = 137.5
TGT_LOCAL_X = 130.0
TGT_LOCAL_Y = 0.0
TGT_LOCAL_Z = -80.0

def solve_ik_3d(x, y, z, l1, l2, l3, foot_angle):
    base_yaw = atan2(y, x)
    r = hypot(x, y)
    r_eff = r - l3 * cos(foot_angle)
    z_eff = z - l3 * sin(foot_angle)
    planar_dist = hypot(r_eff, z_eff)
    if planar_dist > l1 + l2 or planar_dist < abs(l1 - l2):
        return None
    cos_a2 = (planar_dist**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cos_a2 = max(-1.0, min(1.0, cos_a2))
    angle2 = -acos(cos_a2)
    phi = atan2(z_eff, r_eff)
    psi = atan2(l2 * sin(angle2), l1 + l2 * cos(angle2))
    angle1 = phi - psi
    angle3 = foot_angle - (angle1 + angle2)
    return base_yaw, angle1, angle2, angle3, phi, psi, r_eff

def forward_kinematics(base_yaw, angle1, angle2, angle3, l1, l2, l3):
    dir_x = cos(base_yaw)
    dir_y = sin(base_yaw)
    p0 = (0.0, 0.0, 0.0) # Hip Joint (Body/Base)
    p1 = p0              # Start of Link 1
    r1 = l1 * cos(angle1)
    z1 = l1 * sin(angle1)
    r2 = r1 + l2 * cos(angle1 + angle2)
    z2 = z1 + l2 * sin(angle1 + angle2)
    r3 = r2 + l3 * cos(angle1 + angle2 + angle3)
    z3 = z2 + l3 * sin(angle1 + angle2 + angle3)
    
    p2 = (p1[0] + r1 * dir_x, p1[1] + r1 * dir_y, p1[2] + z1)  # Knee Joint
    p3 = (p1[0] + r2 * dir_x, p1[1] + r2 * dir_y, p1[2] + z2)  # Ankle Joint
    p4 = (p1[0] + r3 * dir_x, p1[1] + r3 * dir_y, p1[2] + z3)  # Tip
    
    return p0, p2, p3, p4

def offset_points(points, offset):
    ox, oy, oz = offset
    return [(p[0] + ox, p[1] + oy, p[2] + oz) for p in points]

# Main calculation
foot_angle = 0.0
result = solve_ik_3d(TGT_LOCAL_X, TGT_LOCAL_Y, TGT_LOCAL_Z, L1, L2, L3, foot_angle)

if result is None:
    print("Error: Target unreachable with current parameters.")
    exit()

base_yaw, angle1, angle2, angle3, phi, psi, r = result
local_points = forward_kinematics(base_yaw, angle1, angle2, angle3, L1, L2, L3)

# Leg Angles: 0, 60, 120, 180, 240, 300 degrees
leg_angles_deg = [0, 60, 120, 180, 240, 300]
leg_angles_rad = np.deg2rad(leg_angles_deg)


with open("joint_positions.txt", "w") as f:
    f.write(f"{'Leg':<5} {'Joint':<10} {'X':>10} {'Y':>10} {'Z':>10}\n")
    f.write("-" * 50 + "\n")

    for i, angle_rad in enumerate(leg_angles_rad):
        hip_pos = (HIP_RADIUS * cos(angle_rad), HIP_RADIUS * sin(angle_rad), 0.0)
        
        # Rotate local points
        ca = cos(angle_rad)
        sa = sin(angle_rad)
        
        rotated_points = []
        for p in local_points:
            lx, ly, lz = p
            gx = lx * ca - ly * sa
            gy = lx * sa + ly * ca
            gz = lz
            rotated_points.append((gx, gy, gz))
            
        final_points = offset_points(rotated_points, hip_pos)
        
        # p0=Hip(Base), p2=Knee, p3=Ankle, p4=Tip
        leg_name = f"L{i}"
        f.write(f"{leg_name:<5} {'Hip':<10} {final_points[0][0]:10.2f} {final_points[0][1]:10.2f} {final_points[0][2]:10.2f}\n")
        f.write(f"{'':<5} {'Knee':<10} {final_points[1][0]:10.2f} {final_points[1][1]:10.2f} {final_points[1][2]:10.2f}\n")
        f.write(f"{'':<5} {'Ankle':<10} {final_points[2][0]:10.2f} {final_points[2][1]:10.2f} {final_points[2][2]:10.2f}\n")
        f.write(f"{'':<5} {'Tip':<10} {final_points[3][0]:10.2f} {final_points[3][1]:10.2f} {final_points[3][2]:10.2f}\n")
        f.write("-" * 50 + "\n")

