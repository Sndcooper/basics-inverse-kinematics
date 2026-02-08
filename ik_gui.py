from math import acos, atan2, cos, hypot, sin

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle
from matplotlib.widgets import Slider


def solve_ik(x, y, l1, l2):
    r = hypot(x, y)
    if r > l1 + l2 or r < abs(l1 - l2):
        return None

    cos_a2 = (r**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cos_a2 = max(-1.0, min(1.0, cos_a2))
    angle2 = -acos(cos_a2)

    phi = atan2(y, x)
    psi = atan2(l2 * sin(angle2), l1 + l2 * cos(angle2))
    angle1 = phi - psi
    return angle1, angle2, phi, psi


def deg(rad):
    return rad * 180.0 / np.pi


fig, ax = plt.subplots(figsize=(6, 6))
plt.subplots_adjust(left=0.12, bottom=0.32)

line, = ax.plot([], [], marker="o", markersize=10, label="arm")
target, = ax.plot([], [], marker="x", color="red", label="target")
info = ax.text(
    0.02,
    0.98,
    "",
    transform=ax.transAxes,
    va="top",
    ha="left",
    fontsize=9,
)

ax.set_aspect("equal", "box")
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_title("inverse kinematics")
ax.grid(True)
outer_reach = Circle((0, 0), 1.0, fill=False, color="red", linewidth=1.2, alpha=0.7)
inner_reach = Circle((0, 0), 0.5, fill=False, color="gray", linewidth=1.0, alpha=0.5)
ax.add_patch(outer_reach)
ax.add_patch(inner_reach)

ax.legend(loc="upper right")

ax_x = plt.axes([0.12, 0.22, 0.78, 0.03])
ax_y = plt.axes([0.12, 0.17, 0.78, 0.03])
ax_l1 = plt.axes([0.12, 0.12, 0.78, 0.03])
ax_l2 = plt.axes([0.12, 0.07, 0.78, 0.03])

s_x = Slider(ax_x, "X", -5.0, 5.0, valinit=2.0, valstep=0.1)
s_y = Slider(ax_y, "Y", -5.0, 5.0, valinit=2.0, valstep=0.1)
s_l1 = Slider(ax_l1, "L1", 0.5, 5.0, valinit=2.0, valstep=0.1)
s_l2 = Slider(ax_l2, "L2", 0.5, 5.0, valinit=3.0, valstep=0.1)


def update(_):
    x = s_x.val
    y = s_y.val
    l1 = s_l1.val
    l2 = s_l2.val

    result = solve_ik(x, y, l1, l2)
    maxr = max(l1 + l2, abs(x), abs(y)) + 0.5
    ax.set_xlim(-maxr, maxr)
    ax.set_ylim(-maxr, maxr)
    outer_reach.set_radius(l1 + l2)
    inner_reach.set_radius(abs(l1 - l2))
    target.set_data([x], [y])

    if result is None:
        line.set_data([0], [0])
        info.set_text("Target unreachable")
        fig.canvas.draw_idle()
        return

    angle1, angle2, phi, psi = result
    arm1_x = l1 * cos(angle1)
    arm1_y = l1 * sin(angle1)
    arm2_x = arm1_x + l2 * cos(angle1 + angle2)
    arm2_y = arm1_y + l2 * sin(angle1 + angle2)

    line.set_data([0, arm1_x, arm2_x], [0, arm1_y, arm2_y])
    info.set_text(
        "\n".join(
            [
                f"angle1 = {angle1:.3f} rad ({deg(angle1):.1f} deg)",
                f"angle2 = {angle2:.3f} rad ({deg(angle2):.1f} deg)",
                f"phi    = {phi:.3f} rad ({deg(phi):.1f} deg)",
                f"psi    = {psi:.3f} rad ({deg(psi):.1f} deg)",
            ]
        )
    )
    fig.canvas.draw_idle()


s_x.on_changed(update)
s_y.on_changed(update)
s_l1.on_changed(update)
s_l2.on_changed(update)

update(None)
plt.show()
