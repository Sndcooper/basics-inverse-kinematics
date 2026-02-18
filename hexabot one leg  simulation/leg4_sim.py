import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

BODY_RADIUS = 137.5
L_COXA = 26.0
L_FEMUR = 57.0
L_TIBIA = 122.0

# Leg 4 Mounting Angle (-90 degrees / -PI/2 radians) - Facing -Y
MOUNT_ANGLE = -np.pi / 2

def calculate_ik_leg4(x, y, z):
    """
    Calculates joint angles for Leg 4 (on -Y axis) given a target (x,y,z) in Body Frame.
    x, y, z: Coordinates in Body Frame.
    Returns: (theta_hip, theta_femur, phi2_internal) in Radians.
    """
    
    # 1. Coordinate Transformation (Body Center -> Hip Mount Point)
    # Leg 4 Mount Point Calculation
    mount_x = BODY_RADIUS * np.cos(MOUNT_ANGLE)
    mount_y = BODY_RADIUS * np.sin(MOUNT_ANGLE)
    
    # Vector from Mount Point to Target
    diff_x = x - mount_x
    diff_y = y - mount_y
    
    # 2. Hip Angle (Yaw)
    # Global angle of the vector from Mount to Target
    theta_global = np.arctan2(diff_y, diff_x)
    
    # Relative Hip Angle = Global Angle - Mount Angle
    theta_hip = theta_global - MOUNT_ANGLE
    
    # Normalize to [-PI, PI]
    while theta_hip <= -np.pi: theta_hip += 2*np.pi
    while theta_hip > np.pi: theta_hip -= 2*np.pi
    
    # 3. Planar Distance Components
    # Distance from Hip Mount to Target on XY plane
    r_total = np.sqrt(diff_x**2 + diff_y**2)
    
    # Distance from Knee Pivot to Target (Projected)
    # Subtract Coxa Length
    r_eff = r_total - L_COXA
    
    # 4. IK for Femur and Tibia (Vertical Plane)
    # Virtual link length (Hypotenuse)
    L_virtual = np.sqrt(r_eff**2 + z**2)
    
    # Reachability Check
    if L_virtual > (L_FEMUR + L_TIBIA):
        print("Warning: Target out of reach. Clamping.")
        ratio = (L_FEMUR + L_TIBIA) / L_virtual
        L_virtual = L_FEMUR + L_TIBIA
        r_eff *= ratio # Scale r_eff to bring target closer
        
    # Alpha (Angle of virtual link from horizon)
    alpha_global = np.arctan2(z, r_eff)
    
    # Law of Cosines for Knee (phi1)
    cos_phi1 = (L_virtual**2 + L_FEMUR**2 - L_TIBIA**2) / (2 * L_virtual * L_FEMUR)
    cos_phi1 = np.clip(cos_phi1, -1.0, 1.0)
    phi1 = np.arccos(cos_phi1)
    
    # Law of Cosines for Foot (phi2 - Internal Angle)
    cos_phi2 = (L_FEMUR**2 + L_TIBIA**2 - L_virtual**2) / (2 * L_FEMUR * L_TIBIA)
    cos_phi2 = np.clip(cos_phi2, -1.0, 1.0)
    phi2 = np.arccos(cos_phi2) # Internal angle
    
    # Theta Femur (Pitch from Horizon)
    # Standard: Knee Up -> theta = alpha + phi1
    theta_femur = alpha_global + phi1
    
    return theta_hip, theta_femur, phi2

def calculate_fk_leg4(theta_hip, theta_femur, phi2):
    """
    Calculates 3D coordinates of leg joints based on angles.
    Returns list of points: [BodyCenter, HipJoint, KneeJoint, FootJoint, FootTip]
    """
    points = []
    
    # 1. Body Center
    p0 = np.array([0.0, 0.0, 0.0])
    points.append(p0)
    
    # 2. Hip Joint (Body Radius at Mount Angle)
    # This point is fixed relative to body
    hip_x = BODY_RADIUS * np.cos(MOUNT_ANGLE)
    hip_y = BODY_RADIUS * np.sin(MOUNT_ANGLE)
    hip_z = 0.0
    p1 = np.array([hip_x, hip_y, hip_z])
    points.append(p1)
    
    # Global Yaw of the leg = Mount Angle + Hip Angle
    global_yaw = MOUNT_ANGLE + theta_hip
    
    # 3. Knee Joint
    # Displaced by Coxa Length in direction of Global Yaw (Horizontal)
    knee_x = hip_x + L_COXA * np.cos(global_yaw)
    knee_y = hip_y + L_COXA * np.sin(global_yaw)
    knee_z = hip_z # Coxa is flat
    p2 = np.array([knee_x, knee_y, knee_z])
    points.append(p2)
    
    # 4. Foot Joint
    # Displaced by Femur Length.
    # Orientation: Yaw = global_yaw, Pitch = theta_femur
    # Projected length on plane = L_FEMUR * cos(theta_femur)
    # Z component = L_FEMUR * sin(theta_femur)
    femur_proj = L_FEMUR * np.cos(theta_femur)
    
    foot_x = knee_x + femur_proj * np.cos(global_yaw)
    foot_y = knee_y + femur_proj * np.sin(global_yaw)
    foot_z = knee_z + L_FEMUR * np.sin(theta_femur)
    p3 = np.array([foot_x, foot_y, foot_z])
    points.append(p3)
    
    # 5. Foot Tip
    # Angle of Tibia relative to horizon?
    # phi2 is internal angle between Femur and Tibia.
    # Angle of Tibia = theta_femur - (180 - phi2) ?
    # Let's verify: 
    # If phi2 = 180 (Straight), Tibia angle = theta_femur.
    # If phi2 = 90 (Down), Tibia angle = theta_femur - 90.
    # Formula: theta_tibia = theta_femur - (np.pi - phi2)
    theta_tibia = theta_femur - (np.pi - phi2)
    
    tibia_proj = L_TIBIA * np.cos(theta_tibia)
    
    tip_x = foot_x + tibia_proj * np.cos(global_yaw)
    tip_y = foot_y + tibia_proj * np.sin(global_yaw)
    tip_z = foot_z + L_TIBIA * np.sin(theta_tibia)
    p4 = np.array([tip_x, tip_y, tip_z])
    points.append(p4)
    
    return np.array(points)

def generate_line_trajectory(start_point, end_point, steps):
    """
    Generates a list of 3D points linearly interpolated between start and end.
    """
    trajectory = []
    # Create valid interpolation steps
    for t in np.linspace(0, 1, steps):
        # Linear Interpolation: P = P_start + t * (P_end - P_start)
        point = start_point + t * (end_point - start_point)
        trajectory.append(point)
    
    return np.array(trajectory)

def main():
    # User-defined Start and End points for the Trajectory
    # Moving along X axis (Stride)
    p_start = np.array([-50.0, -220.5, -122.0])
    p_end   = np.array([ 50.0, -220.5, -122.0])
    steps   = 20
    
    print(f"Generating Linear Trajectory from {p_start} to {p_end} with {steps} steps.")
    
    # 1. Generate Target Path (Straight Line)
    target_path = generate_line_trajectory(p_start, p_end, steps)
    
    # 2. Iterate and Calculate IK for each point
    calculated_path = []
    
    print("\nCalculating IK for path...")
    
    # Setup Plotting
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot Body Center and Radius
    ax.scatter(0, 0, 0, s=100, c='k', marker='+', label='Body Center')
    theta = np.linspace(0, 2*np.pi, 50)
    xc = BODY_RADIUS * np.cos(theta)
    yc = BODY_RADIUS * np.sin(theta)
    ax.plot(xc, yc, 0, 'k--', alpha=0.3, label='Body Outline')
    
    # Store just the tip positions for plotting the result curve
    result_tips = []
    
    for i, point in enumerate(target_path):
        x, y, z = point
        
        # Calculate IK
        theta_hip, theta_femur, phi2 = calculate_ik_leg4(x, y, z)
        
        # Calculate FK (to verify)
        points = calculate_fk_leg4(theta_hip, theta_femur, phi2)
        tip_fk = points[-1]
        result_tips.append(tip_fk)
        
        # Plot every 5th step to avoid clutter, or plot all as faint lines
        if i % 4 == 0 or i == steps-1:
            # Draw leg
            ax.plot(points[1:,0], points[1:,1], points[1:,2], 'o-', alpha=0.5, linewidth=1, markersize=3)
            
    result_tips = np.array(result_tips)
    
    # 3. Plot Trajectories (Target vs Result)
    # Target Line
    ax.plot(target_path[:,0], target_path[:,1], target_path[:,2], 'm--', linewidth=2, label='Target Trajectory (Linear)')
    ax.scatter(target_path[0,0], target_path[0,1], target_path[0,2], c='g', marker='^', s=80, label='Start')
    ax.scatter(target_path[-1,0], target_path[-1,1], target_path[-1,2], c='r', marker='v', s=80, label='End')

    # Result Line (FK)
    ax.plot(result_tips[:,0], result_tips[:,1], result_tips[:,2], 'b-', linewidth=1, label='FK Result Path')
    
    # Error Analysis
    error_max = np.max(np.linalg.norm(target_path - result_tips, axis=1))
    print(f"Max Tracking Error: {error_max:.4f} mm")
    
    # Styling
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title(f'Hexapod Leg 4 Linear Trajectory\nMax Error: {error_max:.2f} mm')
    
    limit = 300
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)
    
    ax.legend()
    plt.show()

if __name__ == "__main__":
    main()
