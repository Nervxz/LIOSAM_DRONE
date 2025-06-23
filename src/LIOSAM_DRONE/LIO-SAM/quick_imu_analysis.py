#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

class IMUAnalyzer(Node):
    def __init__(self):
        super().__init__('imu_analyzer')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
        self.samples = []
        self.max_samples = 30
        
    def imu_callback(self, msg):
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.samples.append(quat)
        
        print(f"Sample {len(self.samples)}: x={quat[0]:.3f}, y={quat[1]:.3f}, z={quat[2]:.3f}, w={quat[3]:.3f}")
        
        if len(self.samples) >= self.max_samples:
            self.analyze_and_recommend()
            rclpy.shutdown()
    
    def analyze_and_recommend(self):
        print("\n" + "="*50)
        print("IMU ORIENTATION ANALYSIS")
        print("="*50)
        
        # Average the quaternions
        avg_quat = np.mean(self.samples, axis=0)
        avg_quat = avg_quat / np.linalg.norm(avg_quat)  # Normalize
        
        print(f"Average Quaternion: [{avg_quat[0]:.3f}, {avg_quat[1]:.3f}, {avg_quat[2]:.3f}, {avg_quat[3]:.3f}]")
        
        # Convert to rotation matrix and extract angles
        rotation = R.from_quat(avg_quat)
        euler = rotation.as_euler('xyz', degrees=True)
        
        print(f"Euler angles (XYZ, degrees): [{euler[0]:.1f}, {euler[1]:.1f}, {euler[2]:.1f}]")
        
        # Determine the needed correction
        z_rotation = euler[2]
        
        print(f"\nIMU has {z_rotation:.1f}° rotation around Z-axis")
        
        # Calculate correction rotation (opposite direction)
        correction_angle = -z_rotation
        correction_rad = math.radians(correction_angle)
        
        print(f"Needed correction: {correction_angle:.1f}° around Z-axis")
        
        # Generate the corrected extrinsic rotation
        corrected_rotation = R.from_euler('z', correction_rad)
        corrected_quat = corrected_rotation.as_quat()
        
        print(f"\nRECOMMENDED EXTRINSIC ROTATION:")
        print(f"extrinsicRot: [{corrected_quat[0]:.6f}, {corrected_quat[1]:.6f}, {corrected_quat[2]:.6f}, {corrected_quat[3]:.6f}]")
        
        print(f"\nCopy this line into your params.yaml:")
        print(f"  extrinsicRot: [{corrected_quat[0]:.6f}, {corrected_quat[1]:.6f}, {corrected_quat[2]:.6f}, {corrected_quat[3]:.6f}]")
        
        return corrected_quat

def main():
    rclpy.init()
    
    print("Analyzing IMU orientation...")
    print("Please keep IMU stationary and level...")
    print("Collecting 30 samples...")
    
    analyzer = IMUAnalyzer()
    
    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        print("\nAnalysis interrupted.")
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 