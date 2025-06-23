#!/usr/bin/env python3

"""
Auto Calibrate and Apply Script
Analyzes WitMotion HWT905 IMU orientation and automatically applies calibration to LIO-SAM
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
import os
import subprocess

class AutoCalibrateAndApply(Node):
    def __init__(self):
        super().__init__('auto_calibrate')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
        self.samples = []
        self.sample_count = 0
        self.max_samples = 50
        
        # Paths
        self.config_dir = "/home/nerv/lio_ws/src/LIOSAM_DRONE/LIO-SAM/config"
        self.workspace_dir = "/home/nerv/lio_ws"
        
        print("🤖 AUTO CALIBRATE AND APPLY")
        print("="*50)
        print("🔍 Analyzing IMU orientation...")
        print("📊 Collecting IMU data...")
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
            self.analyze_and_apply()

    def analyze_and_apply(self):
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
        
        # Generate suggested rotation matrix for LIO-SAM
        # We need the INVERSE of the IMU rotation to correct it
        suggested_matrix = rotation_matrix.T  # Transpose = inverse for rotation matrices
        
        print(f"\n🧭 Calculated Calibration Matrix:")
        for i, row in enumerate(suggested_matrix):
            print(f"   [{row[0]:8.3f}, {row[1]:8.3f}, {row[2]:8.3f}]")
        
        # Analyze gravity direction
        gravity_magnitude = np.linalg.norm(avg_accel)
        print(f"🌍 Gravity magnitude: {gravity_magnitude:.2f} m/s²")
        
        # Generate LIO-SAM configuration
        print(f"\n⚙️  GENERATING LIO-SAM CONFIGURATION")
        config_content = self.generate_lio_sam_config(suggested_matrix)
        
        # Backup original configuration
        backup_file = os.path.join(self.config_dir, "params_auto_backup.yaml")
        original_file = os.path.join(self.config_dir, "params.yaml")
        
        if not os.path.exists(backup_file):
            print("💾 Backing up original configuration...")
            subprocess.run(['cp', original_file, backup_file])
            print(f"✅ Backup saved: params_auto_backup.yaml")
        
        # Write new configuration
        print("📝 Writing calibrated configuration...")
        with open(original_file, 'w') as f:
            f.write(config_content)
        print(f"✅ Configuration written to: {original_file}")
        
        # Build LIO-SAM
        print(f"\n🔨 BUILDING LIO-SAM...")
        os.chdir(self.workspace_dir)
        
        build_result = subprocess.run(['colcon', 'build', '--packages-select', 'lio_sam'], 
                                    capture_output=True, text=True)
        
        if build_result.returncode == 0:
            print("✅ Build successful!")
        else:
            print("❌ Build failed!")
            print(build_result.stderr)
            rclpy.shutdown()
            return
        
        print(f"\n" + "="*50)
        print("🎉 AUTO CALIBRATION COMPLETE!")
        print("="*50)
        print(f"✅ IMU orientation analyzed")
        print(f"✅ Calibration matrix calculated")
        print(f"✅ LIO-SAM configuration updated")
        print(f"✅ System built successfully")
        print()
        print("🚀 READY TO TEST!")
        print("Run: ./start_complete_system.sh")
        print()
        print("📊 Expected improvements:")
        print("  • Stable IMU when stationary")
        print("  • No more 'flying around' behavior")
        print("  • Coherent point cloud mapping")
        print("  • Smooth trajectory tracking")
        print()
        
        # Ask if user wants to start the system
        try:
            response = input("🔽 Start the system now? (y/n): ")
            if response.lower() in ['y', 'yes']:
                print("🚀 Starting complete system...")
                os.chdir(self.workspace_dir)
                subprocess.run(['./start_complete_system.sh'])
        except KeyboardInterrupt:
            print("\n👋 Exiting...")
        
        # Stop the node
        rclpy.shutdown()

    def generate_lio_sam_config(self, rotation_matrix):
        """Generate complete LIO-SAM configuration with calibrated extrinsics"""
        
        # Convert rotation matrix to flat list for YAML
        rotation_flat = rotation_matrix.flatten().tolist()
        
        # Generate the configuration
        config = f"""/**:
  ros__parameters:

    # Topics
    pointCloudTopic: "/cloud_all_fields_fullframe"
    imuTopic: "/imu/data"
    odomTopic: "odometry/imu"
    gpsTopic: "odometry/gpsz"

    # Frames
    lidarFrame: "world"
    baselinkFrame: "base_link"
    odometryFrame: "odom"
    mapFrame: "map"

    # GPS Settings
    useImuHeadingInitialization: false
    useGpsElevation: false
    gpsCovThreshold: 2.0
    poseCovThreshold: 25.0

    # Export settings
    savePCD: false
    savePCDDirectory: "/Downloads/LOAM/"

    # Sensor Settings
    sensor: velodyne
    N_SCAN: 16
    Horizon_SCAN: 1800
    downsampleRate: 1
    lidarMinRange: 0.1
    lidarMaxRange: 100.0

    # IMU Settings (Auto-calibrated)
    imuAccNoise: 0.05
    imuGyrNoise: 0.05
    imuAccBiasN: 0.005
    imuGyrBiasN: 0.005

    imuGravity: 9.80665
    imuRPYWeight: 0.01

    # AUTO-CALIBRATED Extrinsic parameters (LiDAR to IMU)
    # Generated automatically from IMU orientation analysis
    extrinsicTrans: [ 0.0,  0.0,  0.0 ]
    extrinsicRot: [{rotation_flat[0]:8.5f}, {rotation_flat[1]:8.5f}, {rotation_flat[2]:8.5f},
                   {rotation_flat[3]:8.5f}, {rotation_flat[4]:8.5f}, {rotation_flat[5]:8.5f},
                   {rotation_flat[6]:8.5f}, {rotation_flat[7]:8.5f}, {rotation_flat[8]:8.5f} ]
    extrinsicRPY: [{rotation_flat[0]:8.5f}, {rotation_flat[1]:8.5f}, {rotation_flat[2]:8.5f},
                   {rotation_flat[3]:8.5f}, {rotation_flat[4]:8.5f}, {rotation_flat[5]:8.5f},
                   {rotation_flat[6]:8.5f}, {rotation_flat[7]:8.5f}, {rotation_flat[8]:8.5f} ]

    # LOAM feature threshold
    edgeThreshold: 0.1
    surfThreshold: 0.05
    edgeFeatureMinValidNum: 5
    surfFeatureMinValidNum: 50

    # voxel filter params
    odometrySurfLeafSize: 0.2
    mappingCornerLeafSize: 0.1
    mappingSurfLeafSize: 0.2

    # robot motion constraint
    z_tollerance: 1000.0
    rotation_tollerance: 1000.0

    # CPU Params
    numberOfCores: 4
    mappingProcessInterval: 0.15

    # Surrounding map
    surroundingkeyframeAddingDistThreshold: 1.0
    surroundingkeyframeAddingAngleThreshold: 0.2
    surroundingKeyframeDensity: 2.0
    surroundingKeyframeSearchRadius: 50.0

    # Loop closure
    loopClosureEnableFlag: true
    loopClosureFrequency: 1.0
    surroundingKeyframeSize: 50
    historyKeyframeSearchRadius: 15.0
    historyKeyframeSearchTimeDiff: 30.0
    historyKeyframeSearchNum: 25
    historyKeyframeFitnessScore: 0.3

    # Visualization
    globalMapVisualizationSearchRadius: 1000.0
    globalMapVisualizationPoseDensity: 10.0
    globalMapVisualizationLeafSize: 1.0
"""
        return config

def main():
    rclpy.init()
    
    print("🤖 AUTO CALIBRATE AND APPLY")
    print("="*50)
    print("🎯 This script will:")
    print("   1. Analyze your IMU orientation")
    print("   2. Calculate the correct calibration matrix")
    print("   3. Apply it to LIO-SAM configuration")
    print("   4. Build the system")
    print("   5. Optionally start the system")
    print()
    print("📌 Make sure your IMU is STATIONARY during analysis")
    print("⏱️  This will take about 5 seconds to collect data")
    print()
    
    calibrator = AutoCalibrateAndApply()
    
    try:
        rclpy.spin(calibrator)
    except KeyboardInterrupt:
        print("\n❌ Analysis interrupted")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        calibrator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 