import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.spatial.transform import Rotation as R

def slerp(q1, q2, t):
    """ Perform Spherical Linear Interpolation (SLERP) between two quaternions. """
    dot = np.dot(q1, q2)
    if dot < 0.0:  # Ensure shortest path
        q2 = -q2
        dot = -dot

    if dot > 0.9995:  # If very close, do linear interpolation
        return (1 - t) * q1 + t * q2

    theta_0 = np.arccos(dot)  # Initial angle
    sin_theta_0 = np.sin(theta_0)
    
    theta_t = theta_0 * t
    sin_theta_t = np.sin(theta_t)
    
    s1 = np.sin(theta_0 - theta_t) / sin_theta_0
    s2 = sin_theta_t / sin_theta_0
    
    return s1 * q1 + s2 * q2

def slerp_animation(target, num_frames=50):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Normalize target vector
    target = np.array(target) / np.linalg.norm(target)
    
    # Initial and target rotations
    v0 = np.array([0, 0, 1])
    if np.allclose(v0, target):
        return print("Target is the same as start vector, no rotation needed.")

    axis = np.cross(v0, target)  # Rotation axis
    angle = np.arccos(np.dot(v0, target))  # Rotation angle
    r1 = R.from_rotvec(axis * angle)  # Rotation from v0 to target
    q1 = np.array([0, 0, 0, 1])  # Identity quaternion
    q2 = r1.as_quat()  # Target quaternion

    # Setup plot
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Plot sphere for reference
    u = np.linspace(0, 2 * np.pi, 30)
    v = np.linspace(0, np.pi, 30)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x, y, z, color='c', alpha=0.2, edgecolors='k', linewidth=0.2)

    # Initial vector plot
    quiver = ax.quiver(0, 0, 0, 0, 0, 1, color='r', linewidth=2)

    def update(i):
        nonlocal quiver
        t = i / (num_frames - 1)  # Interpolation fraction
        qt = slerp(q1, q2, t)  # Interpolate quaternion
        rt = R.from_quat(qt)  # Convert back to rotation
        rot_vec = rt.apply(v0)  # Rotate vector
        quiver.remove()
        quiver = ax.quiver(0, 0, 0, *rot_vec, color='r', linewidth=2)
        return quiver,

    ani = animation.FuncAnimation(fig, update, frames=num_frames, interval=50, blit=False)
    plt.show()

# Example: Animate rotation to [0.707, 0.707, 0]
slerp_animation([0.707, 0.707, 0])
