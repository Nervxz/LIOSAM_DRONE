#!/usr/bin/env python3

"""
IMU Orientation Analysis Script
Analyzes WitMotion HWT905 IMU data to determine correct calibration
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

class IMUOrientationAnalyzer(Node):
    def __init__(self):
        super().__init__('imu_analyzer')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
        self.samples = []
        self.sample_count = 0
        self.max_samples = 50
        
        print("🔍 IMU Orientation Analyzer Started")
        print("📊 Collecting IMU data to determine orientation...")
        print("📌 Keep the IMU STATIONARY for accurate analysis")
        print()

    def imu_callback(self, msg):
        if self.sample_count < self.max_samples:
            # Extract quaternion
            quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            
            # Extract linear acceleration (should show gravity when stationary)
            accel = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
            
            # Store sample
            self.samples.append({
                'quaternion': quat,
                'acceleration': accel,
                'timestamp': time.time()
            })
            
            self.sample_count += 1
            
            if self.sample_count % 10 == 0:
                print(f"📊 Collected {self.sample_count}/{self.max_samples} samples...")
        
        elif self.sample_count == self.max_samples:
            self.sample_count += 1  # Prevent multiple analysis
            self.analyze_orientation()

    def analyze_orientation(self):
        print("\n" + "="*50)
        print("🔬 ANALYZING IMU ORIENTATION")
        print("="*50)
        
        # Average the samples
        avg_quat = np.mean([s['quaternion'] for s in self.samples], axis=0)
        avg_accel = np.mean([s['acceleration'] for s in self.samples], axis=0)
        
        print(f"📊 Average Quaternion: [{avg_quat[0]:.3f}, {avg_quat[1]:.3f}, {avg_quat[2]:.3f}, {avg_quat[3]:.3f}]")
        print(f"📊 Average Acceleration: [{avg_accel[0]:.3f}, {avg_accel[1]:.3f}, {avg_accel[2]:.3f}]")
        
        # Convert quaternion to rotation matrix
        r = R.from_quat(avg_quat)
        rotation_matrix = r.as_matrix()
        euler_angles = r.as_euler('xyz', degrees=True)
        
        print(f"📐 Euler Angles (XYZ): [{euler_angles[0]:.1f}°, {euler_angles[1]:.1f}°, {euler_angles[2]:.1f}°]")
        print("\n🧭 Rotation Matrix:")
        for i, row in enumerate(rotation_matrix):
            print(f"   [{row[0]:8.3f}, {row[1]:8.3f}, {row[2]:8.3f}]")
        
        # Analyze gravity direction
        gravity_magnitude = np.linalg.norm(avg_accel)
        gravity_direction = avg_accel / gravity_magnitude
        
        print(f"\n🌍 Gravity Analysis:")
        print(f"   Magnitude: {gravity_magnitude:.2f} m/s²")
        print(f"   Direction: [{gravity_direction[0]:.3f}, {gravity_direction[1]:.3f}, {gravity_direction[2]:.3f}]")
        
        # Determine the dominant axis
        dominant_axis = np.argmax(np.abs(gravity_direction))
        axis_names = ['X', 'Y', 'Z']
        print(f"   Dominant gravity axis: {axis_names[dominant_axis]} ({'positive' if gravity_direction[dominant_axis] > 0 else 'negative'})")
        
        # Suggest calibration based on Z-rotation
        z_rotation_angle = euler_angles[2]
        print(f"\n🎯 CALIBRATION SUGGESTIONS:")
        print(f"   Primary Z-axis rotation: {z_rotation_angle:.1f}°")
        
        # Generate suggested rotation matrix for LIO-SAM
        # We need the INVERSE of the IMU rotation to correct it
        suggested_matrix = rotation_matrix.T  # Transpose = inverse for rotation matrices
        
        print(f"\n📋 SUGGESTED EXTRINSIC ROTATION MATRIX:")
        print("   Copy this to your LIO-SAM config:")
        print("   extrinsicRot: [", end="")
        for i in range(3):
            for j in range(3):
                if i == 2 and j == 2:
                    print(f"{suggested_matrix[i,j]:8.3f}]")
                else:
                    print(f"{suggested_matrix[i,j]:8.3f}, ", end="")
            if i < 2:
                print("\n                   ", end="")
        
        print("\n📝 YAML Format:")
        print("extrinsicRot: [", end="")
        flat_matrix = suggested_matrix.flatten()
        for i, val in enumerate(flat_matrix):
            if i == len(flat_matrix) - 1:
                print(f"{val:8.3f}]")
            else:
                print(f"{val:8.3f},", end="")
                if (i + 1) % 3 == 0:
                    print("\n               ", end="")
                else:
                    print(" ", end="")
        
        # Provide quick config recommendations
        print(f"\n🚀 QUICK TEST RECOMMENDATIONS:")
        
        if abs(z_rotation_angle - 90) < 15:
            print("   ✅ Try config 2 (90° Z-rotation)")
        elif abs(z_rotation_angle + 90) < 15:
            print("   ✅ Try config 5 (-90° Z-rotation)")
        elif abs(abs(z_rotation_angle) - 180) < 15:
            print("   ✅ Try config 3 (180° Z-rotation)")
        elif abs(z_rotation_angle) < 15:
            print("   ✅ Try config 1 (standard mount)")
        else:
            print(f"   ⚠️  Custom rotation needed: {z_rotation_angle:.1f}°")
        
        print("\n" + "="*50)
        print("🎯 Analysis complete! Use the suggested matrix above.")
        print("="*50)
        
        # Stop the node
        rclpy.shutdown()

def main():
    rclpy.init()
    
    print("🔍 Starting IMU Orientation Analysis")
    print("📌 Make sure your IMU is STATIONARY during analysis")
    print("⏱️  This will take about 5 seconds to collect data")
    print()
    
    analyzer = IMUOrientationAnalyzer()
    
    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        print("\n❌ Analysis interrupted")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        analyzer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 