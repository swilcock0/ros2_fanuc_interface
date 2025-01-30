import numpy as np
from scipy.spatial.transform import Rotation as R

def vector_to_euler(target):
    # Normalize target to ensure it's on the unit sphere
    target = np.array(target) / np.linalg.norm(target)

    # Extract spherical coordinates
    theta = np.arctan2(target[1], target[0])  # Azimuth angle
    phi = np.arccos(target[2])  # Inclination angle from z-axis

    # Create rotation matrix
    R_z = R.from_euler('z', theta)  # Rotate around z
    R_y = R.from_euler('y', phi)  # Rotate around y

    # Combine rotations
    R_total = R_y * R_z

    # Convert to Euler angles (ZYX convention)
    euler_angles = R_total.as_euler('zyx', degrees=True)  # or degrees=False for radians
    return euler_angles

# Example: Find Euler angles to rotate [0,0,1] to [0.707, 0.707, 0]
target_vector = [0.707, 0.707, 0]
euler_angles = vector_to_euler(target_vector)
print("Euler Angles (ZYX):", euler_angles)
