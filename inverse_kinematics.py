from math import *
import numpy as np
import matplotlib.pyplot as plt

X, Y = 2, 2

L1 = 2
L2 = 3

r = sqrt(X**2 + Y**2)
cosA2 = r**2 - L1**2 - L2**2
cosA2 /= 2 * L1 * L2
angle2 = -acos(cosA2)  # elbow angle negative if u want it upside

phi = atan2(Y, X)  # angle bw x-axis and line from origin to (X,Y)
psi = atan2(L2 * sin(angle2), L1 + L2 * cos(angle2))  # angle bw line from origin to (X,Y) and line from origin to arm1

angle1 = phi - psi  # shoulder angle

arm1X = L1 * cos(angle1)
arm1Y = L1 * sin(angle1)
arm2X = arm1X + L2 * cos(angle2 + angle1)
arm2Y = arm1Y + L2 * sin(angle2 + angle1)

x = np.array([0, arm1X, arm2X])
y = np.array([0, arm1Y, arm2Y])

plt.plot(x, y, marker='o', markersize=10)
plt.xlabel("x")
plt.ylabel("y")
plt.title("inverse kinematics")

plt.xticks([-4, -3, -2, -1, 0, 1, 2, 3, 4])
plt.yticks([-4, -3, -2, -1, 0, 1, 2, 3, 4])

plt.legend(["arm"])
plt.grid()
plt.show()
