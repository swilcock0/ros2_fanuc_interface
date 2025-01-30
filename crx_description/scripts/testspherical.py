import numpy as np
import matplotlib.pyplot as plt

n = 8 # Number of points

def testPose(phi, theta, r=1.0, x_c=0.0, y_c=0.0, z_c=0.0):
    """Convert spherical coordinates to Cartesian coordinates."""
    st = np.sin(theta)
    x = r * np.cos(phi) * st + x_c
    y = r * np.sin(phi) * st + y_c
    z = r * np.cos(theta) + z_c
    return x, y, z

# Generate points
points = []
for phi in np.linspace(0, 2 * np.pi, n, endpoint=True):
    for theta in np.linspace(0, np.pi, n, endpoint=True):
        points.append(testPose(phi, theta))

# Convert to numpy array
points = np.array(points)

# 3D Plot
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

print(len(points))

ax.scatter(points[:, 0], points[:, 1], points[:, 2], color="r", marker="o")

# Labels & Formatting
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Spherical Points on a Sphere")

plt.show()
