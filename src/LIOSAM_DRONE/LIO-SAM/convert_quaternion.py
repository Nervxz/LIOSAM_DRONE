#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as R

# Quaternion from IMU analysis: [x, y, z, w]
recommended_quat = [0.000000, 0.000000, -0.995263, 0.097218]

print("Converting quaternion to rotation matrix...")
print(f"Input quaternion: {recommended_quat}")

# Convert to rotation matrix
rotation = R.from_quat(recommended_quat)
rot_matrix = rotation.as_matrix()

print("\n3x3 Rotation Matrix:")
print(rot_matrix)

print(f"\nLIO-SAM extrinsicRot format:")
print(f"extrinsicRot: [{rot_matrix[0,0]:.6f}, {rot_matrix[0,1]:.6f}, {rot_matrix[0,2]:.6f},")
print(f"               {rot_matrix[1,0]:.6f}, {rot_matrix[1,1]:.6f}, {rot_matrix[1,2]:.6f},")
print(f"               {rot_matrix[2,0]:.6f}, {rot_matrix[2,1]:.6f}, {rot_matrix[2,2]:.6f} ]")

# Verify the conversion
euler = rotation.as_euler('xyz', degrees=True)
print(f"\nVerification - Euler angles: {euler}")
print(f"Z-axis rotation: {euler[2]:.1f}°") 