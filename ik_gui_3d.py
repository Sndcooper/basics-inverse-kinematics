from math import acos, atan2, cos, hypot, sin
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider


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


def deg(rad):
    return rad * 180.0 / np.pi


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def within_limits(value, min_value, max_value):
    return min_value <= value <= max_value


def compute_workspace_bounds(l1, l2, l3, hip_min, hip_max, knee_min, knee_max, ankle_min, ankle_max, samples=41):
    hip_vals = np.linspace(hip_min, hip_max, samples)
    knee_vals = np.linspace(knee_min, knee_max, samples)
    ankle_vals = np.linspace(ankle_min, ankle_max, samples)

    r_min = float("inf")
    r_max = float("-inf")
    z_min = float("inf")
    z_max = float("-inf")

    for a1 in hip_vals:
        for a2 in knee_vals:
            for a3 in ankle_vals:
                r = l1 * cos(a1) + l2 * cos(a1 + a2) + l3 * cos(a1 + a2 + a3)
                z = l1 * sin(a1) + l2 * sin(a1 + a2) + l3 * sin(a1 + a2 + a3)
                r_min = min(r_min, r)
                r_max = max(r_max, r)
                z_min = min(z_min, z)
                z_max = max(z_max, z)

    return r_min, r_max, z_min, z_max


def forward_kinematics(base_yaw, angle1, angle2, angle3, l1, l2, l3):
    dir_x = cos(base_yaw)
    dir_y = sin(base_yaw)

    p0 = (0.0, 0.0, 0.0)
    p1 = p0
    r1 = l1 * cos(angle1)
    z1 = l1 * sin(angle1)
    r2 = r1 + l2 * cos(angle1 + angle2)
    z2 = z1 + l2 * sin(angle1 + angle2)
    r3 = r2 + l3 * cos(angle1 + angle2 + angle3)
    z3 = z2 + l3 * sin(angle1 + angle2 + angle3)

    p2 = (p1[0] + r1 * dir_x, p1[1] + r1 * dir_y, p1[2] + z1)
    p3 = (p1[0] + r2 * dir_x, p1[1] + r2 * dir_y, p1[2] + z2)
    p4 = (p1[0] + r3 * dir_x, p1[1] + r3 * dir_y, p1[2] + z3)
    return p0, p1, p2, p3, p4

fig = plt.figure(figsize=(7, 6))
ax = fig.add_subplot(111, projection="3d")
plt.subplots_adjust(left=0.12, bottom=0.32)

line, = ax.plot([], [], [], marker="o", markersize=6, label="leg")
target, = ax.plot([], [], [], marker="x", color="red", label="target")
info = ax.text2D(0.02, 0.98, "", transform=ax.transAxes, va="top", ha="left", fontsize=9)

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
ax.set_title("3D hexapod leg inverse kinematics")
ax.legend(loc="upper right")

L1 = 26.0
L2 = 57.0
L3 = 122.0

HIP_MIN = np.deg2rad(-80.0)
HIP_MAX = np.deg2rad(80.0)
KNEE_MIN = np.deg2rad(-30.0)
KNEE_MAX = np.deg2rad(90.0)
ANKLE_MIN = np.deg2rad(-140.0)
ANKLE_MAX = np.deg2rad(20.0)

DEFAULT_BASE_YAW = 0.0
DEFAULT_HIP = 0.0
DEFAULT_KNEE = 0.0
DEFAULT_ANKLE = np.deg2rad(90.0)

r_min, r_max, z_min, z_max = compute_workspace_bounds(
    L1,
    L2,
    L3,
    HIP_MIN,
    HIP_MAX,
    KNEE_MIN,
    KNEE_MAX,
    ANKLE_MIN,
    ANKLE_MAX,
)

xy_max = max(abs(r_min), abs(r_max))

ax_x = plt.axes([0.12, 0.22, 0.78, 0.03])
ax_y = plt.axes([0.12, 0.17, 0.78, 0.03])
ax_z = plt.axes([0.12, 0.12, 0.78, 0.03])
s_x = Slider(ax_x, "X", -xy_max, xy_max, valinit=xy_max * 0.3, valstep=0.1)
s_y = Slider(ax_y, "Y", -xy_max, xy_max, valinit=xy_max * 0.2, valstep=0.1)
s_z = Slider(ax_z, "Z", z_min, z_max, valinit=(z_min + z_max) * 0.5, valstep=0.1)

def update(_):
    x = s_x.val
    y = s_y.val
    z = s_z.val
    l1 = L1
    l2 = L2
    l3 = L3

    foot_angle = np.deg2rad(0.0)
    result = solve_ik_3d(x, y, z, l1, l2, l3, foot_angle)

    maxr = max(abs(x), abs(y), abs(z), l1 + l2 + l3) + 0.5
    ax.set_xlim(-maxr, maxr)
    ax.set_ylim(-maxr, maxr)
    ax.set_zlim(-maxr, maxr)

    target.set_data([x], [y])
    target.set_3d_properties([z])

    default_points = forward_kinematics(
        DEFAULT_BASE_YAW,
        DEFAULT_HIP,
        DEFAULT_KNEE,
        DEFAULT_ANKLE,
        l1,
        l2,
        l3,
    )

    if result is None:
        xs = [p[0] for p in default_points]
        ys = [p[1] for p in default_points]
        zs = [p[2] for p in default_points]
        line.set_data(xs, ys)
        line.set_3d_properties(zs)
        info.set_text("Target unreachable")
        fig.canvas.draw_idle()
        return

    base_yaw, angle1, angle2, angle3, phi, psi, r = result

    if not (
        within_limits(angle1, HIP_MIN, HIP_MAX)
        and within_limits(angle2, KNEE_MIN, KNEE_MAX)
        and within_limits(angle3, ANKLE_MIN, ANKLE_MAX)
    ):
        xs = [p[0] for p in default_points]
        ys = [p[1] for p in default_points]
        zs = [p[2] for p in default_points]
        line.set_data(xs, ys)
        line.set_3d_properties(zs)
        info.set_text("Target unreachable (limits)")
        fig.canvas.draw_idle()
        return

    p0, p1, p2, p3, p4 = forward_kinematics(base_yaw, angle1, angle2, angle3, l1, l2, l3)

    dx = x - p4[0]
    dy = y - p4[1]
    dz = z - p4[2]
    eff_err = hypot(dx, dy, dz)

    xs = [p0[0], p1[0], p2[0], p3[0], p4[0]]
    ys = [p0[1], p1[1], p2[1], p3[1], p4[1]]
    zs = [p0[2], p1[2], p2[2], p3[2], p4[2]]

    line.set_data(xs, ys)
    line.set_3d_properties(zs)

    info.set_text(
        "\n".join(
            [
                f"base_yaw = {base_yaw:.3f} rad ({deg(base_yaw):.1f} deg)",
                f"angle1   = {angle1:.3f} rad ({deg(angle1):.1f} deg)",
                f"angle2   = {angle2:.3f} rad ({deg(angle2):.1f} deg)",
                f"angle3   = {angle3:.3f} rad ({deg(angle3):.1f} deg)",
                f"phi      = {phi:.3f} rad ({deg(phi):.1f} deg)",
                f"psi      = {psi:.3f} rad ({deg(psi):.1f} deg)",
                f"r        = {r:.3f}",
                f"tgt = ({x:.2f}, {y:.2f}, {z:.2f})",
                f"eff = ({p4[0]:.2f}, {p4[1]:.2f}, {p4[2]:.2f})",
                f"err = {eff_err:.3f} (dx={dx:.2f}, dy={dy:.2f}, dz={dz:.2f})",
            ]
        )
    )
    print(
        " ".join(
            [
                f"tgt=({x:.2f}, {y:.2f}, {z:.2f})",
                f"eff=({p4[0]:.2f}, {p4[1]:.2f}, {p4[2]:.2f})",
                f"err={eff_err:.3f} (dx={dx:.2f}, dy={dy:.2f}, dz={dz:.2f})",
            ]
        )
    )
    fig.canvas.draw_idle()


s_x.on_changed(update)
s_y.on_changed(update)
s_z.on_changed(update)

update(None)
plt.show()