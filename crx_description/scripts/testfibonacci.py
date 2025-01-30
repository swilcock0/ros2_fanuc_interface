import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

def fibonacci_sphere(samples=1):
    points = []
    phi = np.pi * (3. - np.sqrt(5.))  # golden angle in radians

    for i in range(samples):
        y = 1 - (i / float(samples - 1)) * 2  # y goes from 1 to -1
        radius = np.sqrt(1 - y * y)  # radius at y

        theta = phi * i  # golden angle increment

        x = np.cos(theta) * radius
        z = np.sin(theta) * radius

        points.append((x, y, z))

    return points

# Generate points
N = 8
points = fibonacci_sphere(N)

# Extract x, y, z coordinates
x_coords = [p[0] for p in points]
y_coords = [p[1] for p in points]
z_coords = [p[2] for p in points]

# Plot the points in 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
sc = ax.scatter(x_coords, y_coords, z_coords, s=1)

highlight, = ax.plot([x_coords[0]], [y_coords[0]], [z_coords[0]], 'ro')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Fibonacci Sphere')

def update(n):
    highlight.set_data([x_coords[n]], [y_coords[n]])
    highlight.set_3d_properties([z_coords[n]])
    return highlight,

# Create animation
ani = FuncAnimation(fig, update, frames=range(N), interval=100, blit=True, repeat=True)

plt.show()