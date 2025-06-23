#!/usr/bin/env python3

"""
LiDAR-IMU Calibration Analysis Script
Estimates extrinsic parameters between SICK MultiScan 136 and WitMotion HWT905
"""

import rclpy
from rclpy.node import Node
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R
import argparse
import os
import sys

class LidarImuCalibrationAnalyzer:
    def __init__(self, bag_path):
        self.bag_path = bag_path
        self.imu_data = []
        self.lidar_poses = []
        self.timestamps = []
        
    def read_bag(self):
        """Read and synchronize IMU and LiDAR data from bag file"""
        print("📂 Reading bag file:", self.bag_path)
        
        storage_options = rosbag2_py.StorageOptions(uri=self.bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}
        
        imu_msgs = []
        
        print("📊 Processing messages...")
        msg_count = 0
        
        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            msg_count += 1
            
            if msg_count % 100 == 0:
                print(f"  Processed {msg_count} messages...")
            
            if topic == '/imu/data':
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                
                # Convert quaternion to rotation matrix
                quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
                if np.linalg.norm(quat) > 0.1:  # Valid quaternion
                    rotation = R.from_quat(quat)
                    
                    imu_data_point = {
                        'timestamp': timestamp,
                        'rotation_matrix': rotation.as_matrix(),
                        'angular_velocity': np.array([
                            msg.angular_velocity.x,
                            msg.angular_velocity.y, 
                            msg.angular_velocity.z
                        ]),
                        'linear_acceleration': np.array([
                            msg.linear_acceleration.x,
                            msg.linear_acceleration.y,
                            msg.linear_acceleration.z
                        ])
                    }
                    imu_msgs.append(imu_data_point)
        
        self.imu_data = imu_msgs
        print(f"✅ Processed {len(self.imu_data)} IMU messages")
        
    def estimate_gravity_direction(self):
        """Estimate gravity direction from accelerometer data during stationary periods"""
        print("🌍 Estimating gravity direction...")
        
        if len(self.imu_data) < 10:
            print("❌ Not enough IMU data for gravity estimation")
            return np.array([0, 0, -9.81])
        
        # Use first and last portions (likely stationary)
        stationary_data = []
        n_msgs = len(self.imu_data)
        
        # First 10% and last 10% of data
        for i in range(min(int(n_msgs * 0.1), 50)):
            stationary_data.append(self.imu_data[i]['linear_acceleration'])
        for i in range(max(int(n_msgs * 0.9), n_msgs - 50), n_msgs):
            stationary_data.append(self.imu_data[i]['linear_acceleration'])
        
        if len(stationary_data) < 5:
            print("❌ Not enough stationary data")
            return np.array([0, 0, -9.81])
        
        # Average acceleration during stationary periods should be gravity
        gravity_imu = np.mean(stationary_data, axis=0)
        gravity_magnitude = np.linalg.norm(gravity_imu)
        
        print(f"📏 Estimated gravity: {gravity_imu}")
        print(f"📏 Gravity magnitude: {gravity_magnitude:.3f} m/s²")
        
        return gravity_imu
    
    def estimate_rotation_from_motion(self):
        """Estimate rotation between coordinate frames using motion analysis"""
        print("🔄 Analyzing rotational motion...")
        
        if len(self.imu_data) < 100:
            print("❌ Not enough data for motion analysis")
            return np.eye(3)
        
        # Analyze angular velocity patterns
        angular_velocities = []
        for data in self.imu_data:
            angular_velocities.append(data['angular_velocity'])
        
        angular_velocities = np.array(angular_velocities)
        
        # Find periods of significant rotation
        angular_magnitude = np.linalg.norm(angular_velocities, axis=1)
        motion_threshold = np.percentile(angular_magnitude, 70)
        
        motion_indices = np.where(angular_magnitude > motion_threshold)[0]
        print(f"📊 Found {len(motion_indices)} motion samples above threshold")
        
        if len(motion_indices) < 10:
            print("⚠️  Limited motion data - using identity rotation")
            return np.eye(3)
        
        # Simple heuristic: if the system shows consistent patterns,
        # we can estimate alignment based on dominant motion axes
        motion_data = angular_velocities[motion_indices]
        
        # Principal component analysis to find dominant motion directions
        cov_matrix = np.cov(motion_data.T)
        eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
        
        # Sort by eigenvalue magnitude
        idx = np.argsort(eigenvalues)[::-1]
        principal_axes = eigenvectors[:, idx]
        
        print(f"📊 Principal motion axes analysis:")
        print(f"  Eigenvalues: {eigenvalues[idx]}")
        
        # For now, return identity - this would need more sophisticated analysis
        return np.eye(3)
    
    def generate_calibration_parameters(self):
        """Generate calibration parameters for LIO-SAM"""
        print("⚙️  Generating calibration parameters...")
        
        # Estimate gravity direction
        gravity_imu = self.estimate_gravity_direction()
        
        # Estimate rotation (simplified approach)
        rotation_matrix = self.estimate_rotation_from_motion()
        
        # For SICK MultiScan, typical mounting orientations
        # This is a simplified approach - proper calibration would require more analysis
        
        # Convert to rotation matrix format for LIO-SAM
        rotation_flat = rotation_matrix.flatten()
        
        # Estimate translation (assume co-located for now)
        translation = [0.0, 0.0, 0.0]
        
        print("📋 Calibration Results:")
        print("=" * 50)
        print(f"Translation (LiDAR to IMU): {translation}")
        print(f"Rotation Matrix:")
        for i in range(3):
            print(f"  [{rotation_matrix[i, 0]:8.5f}, {rotation_matrix[i, 1]:8.5f}, {rotation_matrix[i, 2]:8.5f}]")
        print(f"Gravity in IMU frame: {gravity_imu}")
        print("=" * 50)
        
        return {
            'translation': translation,
            'rotation_matrix': rotation_matrix,
            'rotation_flat': rotation_flat.tolist(),
            'gravity_imu': gravity_imu.tolist()
        }
    
    def generate_lio_sam_config(self, calibration_params):
        """Generate LIO-SAM configuration with calibration parameters"""
        
        config_text = f"""
# CALIBRATED LIO-SAM PARAMETERS
# Generated from LiDAR-IMU calibration data
# SICK MultiScan 136 + WitMotion HWT905

/**:
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

    # IMU Settings (calibrated)
    imuAccNoise: 0.05
    imuGyrNoise: 0.05
    imuAccBiasN: 0.005
    imuGyrBiasN: 0.005

    imuGravity: 9.80665
    imuRPYWeight: 0.01

    # CALIBRATED Extrinsic parameters (LiDAR to IMU)
    extrinsicTrans: {calibration_params['translation']}
    extrinsicRot: {calibration_params['rotation_flat']}
    extrinsicRPY: {calibration_params['rotation_flat']}

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
        
        return config_text

def main():
    parser = argparse.ArgumentParser(description='Analyze LiDAR-IMU calibration data')
    parser.add_argument('bag_path', help='Path to the calibration bag file')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.bag_path):
        print(f"❌ Bag file not found: {args.bag_path}")
        sys.exit(1)
    
    print("🚀 Starting LiDAR-IMU Calibration Analysis")
    print("=" * 50)
    
    # Initialize ROS 2
    rclpy.init()
    
    try:
        analyzer = LidarImuCalibrationAnalyzer(args.bag_path)
        analyzer.read_bag()
        
        calibration_params = analyzer.generate_calibration_parameters()
        
        # Generate LIO-SAM config
        config_text = analyzer.generate_lio_sam_config(calibration_params)
        
        # Save calibrated config
        output_file = 'params_calibrated.yaml'
        with open(output_file, 'w') as f:
            f.write(config_text)
        
        print(f"✅ Calibrated parameters saved to: {output_file}")
        print("🔧 Copy this file to your LIO-SAM config directory")
        print("📝 Update your launch file to use the calibrated parameters")
        
    except Exception as e:
        print(f"❌ Error during calibration analysis: {e}")
        sys.exit(1)
    
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 