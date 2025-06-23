# LIO-SAM with SICK MultiScan 136 and WitMotion HWT905

This guide provides comprehensive instructions for setting up and running LIO-SAM (LiDAR Inertial Odometry via Smoothing and Mapping) with SICK MultiScan 136 LiDAR and WitMotion HWT905 IMU.

## Table of Contents
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation](#installation)
- [Configuration](#configuration)
- [Running the System](#running-the-system)
- [Visualization in RViz](#visualization-in-rviz)
- [Troubleshooting](#troubleshooting)
- [System Architecture](#system-architecture)

## Hardware Requirements

- **LiDAR**: SICK MultiScan 136
  - IP Address: 169.254.83.177
  - UDP Address: 169.254.148.106
  - Data Rate: ~7.5 Hz
  
- **IMU**: WitMotion HWT905
  - Connection: USB Serial
  - Data Rate: 200 Hz
  - Device: `/dev/ttyUSB0` (may vary)

## Software Requirements

- Ubuntu 20.04 or later
- ROS2 Humble
- GTSAM library
- PCL (Point Cloud Library)
- Eigen3

## Installation

### 1. Install Dependencies

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS2 dependencies
sudo apt install -y \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-tf2-eigen \
  ros-humble-tf2-sensor-msgs \
  ros-humble-xacro \
  ros-humble-robot-state-publisher

# Install build tools
sudo apt install -y \
  build-essential \
  cmake \
  git \
  libeigen3-dev \
  libboost-all-dev \
  libtbb-dev
```

### 2. Build GTSAM

GTSAM is already built in the workspace. If you need to rebuild:

```bash
cd ~/lio_ws
colcon build --packages-select gtsam
```

### 3. Build SICK Scanner Driver

```bash
cd ~/lio_ws
colcon build --packages-select sick_scan_xd
```

### 4. Build WitMotion IMU Driver

```bash
cd ~/lio_ws
colcon build --packages-select witmotion_ros
```

### 5. Build LIO-SAM

```bash
cd ~/lio_ws
colcon build --packages-select lio_sam
source install/setup.bash
```

## Configuration

### 1. LIO-SAM Parameters

The main configuration file is located at:
```
~/lio_ws/src/LIOSAM_DRONE/lio_sam/config/params.yaml
```

Key parameters that must be set correctly:

```yaml
# Topics
pointCloudTopic: "/cloud_all_fields_fullframe"  # SICK LiDAR topic
imuTopic: "/imu/data"                           # WitMotion IMU topic

# Frames
lidarFrame: "world"
imuFrame: "imu_link"

# Sensor Settings (CRITICAL - Use Ouster for SICK MultiScan)
sensor: ouster        # Works best for SICK MultiScan
N_SCAN: 128          # Number of scan lines
Horizon_SCAN: 1024   # Points per scan line
downsampleRate: 1
lidarMinRange: 0.3
lidarMaxRange: 100.0

# IMU Settings
imuType: 1           # 0: 6-axis, 1: 9-axis
imuRate: 200.0
imuAccNoise: 0.01
imuGyrNoise: 0.001
imuAccBiasN: 0.0002
imuGyrBiasN: 0.00003
imuGravity: 9.80511
imuRPYWeight: 0.01

# Extrinsic Parameters (LiDAR to IMU)
# Calibrated for -85° rotation around Z-axis
extrinsicRot: [ 0.087156,  0.996195,  0.0,
               -0.996195,  0.087156,  0.0,
                0.0,       0.0,       1.0]
extrinsicRPY: [0, 0, 0]
```

### 2. SICK LiDAR Configuration

The SICK scanner uses the following launch file:
```
~/lio_ws/install/sick_scan_xd/share/sick_scan_xd/launch/sick_multiscan.launch.py
```

Default parameters work well, but ensure:
- Hostname: 169.254.83.177
- Cloud topic: `/cloud_all_fields_fullframe`

### 3. WitMotion IMU Configuration

The IMU publishes to `/imu/data` by default. No additional configuration needed.

## Running the System

### Step-by-Step Launch Procedure

#### 1. Source the workspace
```bash
cd ~/lio_ws
source install/setup.bash
```

#### 2. Launch SICK LiDAR Scanner
```bash
ros2 launch sick_scan_xd sick_multiscan.launch.py hostname:=169.254.83.177
```
Wait for "Startup sequence completed" message.

#### 3. Launch WitMotion IMU
```bash
ros2 launch witmotion_ros witmotion_ros.launch.py
```
Verify IMU is publishing at 200 Hz.

#### 4. Launch LIO-SAM
```bash
ros2 launch lio_sam run.launch.py
```

### Alternative: Use the Auto-Calibrate Script

For automatic system startup with calibration:
```bash
cd ~/lio_ws
./auto_calibrate_and_apply.py
```

### Verify System Status

Check that all topics are publishing:
```bash
# In a new terminal
ros2 topic list
```

Expected topics:
- `/cloud_all_fields_fullframe` (LiDAR data)
- `/imu/data` (IMU data)
- `/lio_sam/mapping/cloud_registered` (Registered point cloud)
- `/lio_sam/mapping/odometry` (Odometry output)
- `/lio_sam/mapping/path` (Trajectory)

Check data rates:
```bash
ros2 topic hz /cloud_all_fields_fullframe  # Should be ~7.5 Hz
ros2 topic hz /imu/data                    # Should be ~200 Hz
```

## Visualization in RViz

### Launch RViz
```bash
rviz2
```

### Configure RViz

1. **Global Options**
   - Fixed Frame: `odom`

2. **Add Displays**
   - **PointCloud2** for `/cloud_all_fields_fullframe`
     - Color Transformer: Intensity
     - Size: 0.05
   
   - **PointCloud2** for `/lio_sam/mapping/cloud_registered`
     - Color Transformer: Intensity
     - Size: 0.1
   
   - **Path** for `/lio_sam/mapping/path`
     - Line Style: Lines
     - Color: Green
   
   - **TF** to visualize coordinate frames
   
   - **Odometry** for `/lio_sam/mapping/odometry`
     - Shape: Arrow
     - Color: Red

3. **Save Configuration**
   - File → Save Config As → `lio_sam_config.rviz`

## Troubleshooting

### Common Issues and Solutions

#### 1. IMU Not Publishing Data
```bash
# Check if device exists
ls -la /dev/ttyUSB*

# Check permissions
sudo chmod 666 /dev/ttyUSB0

# Kill any conflicting processes
ps aux | grep witmotion
sudo pkill -f witmotion
```

#### 2. LiDAR Connection Failed
```bash
# Test network connection
ping 169.254.83.177

# Check network interface
ip addr show

# Restart network if needed
sudo systemctl restart NetworkManager
```

#### 3. IMU "Flying Around" in RViz
This indicates incorrect extrinsic calibration. Run the calibration analysis:
```bash
cd ~/lio_ws
python3 analyze_imu_orientation.py
```

#### 4. No Map Building
Check if all LIO-SAM nodes are running:
```bash
ros2 node list | grep lio_sam
```

Should see:
- `/lio_sam_imuPreintegration`
- `/lio_sam_imageProjection`
- `/lio_sam_featureExtraction`
- `/lio_sam_mapOptimization`

#### 5. Low Frame Rate Warning
The SICK MultiScan publishes at ~7.5 Hz which is lower than typical. This is normal and LIO-SAM can handle it.

### Performance Optimization

1. **Reduce Point Cloud Size**
   ```yaml
   downsampleRate: 2  # Downsample by factor of 2
   ```

2. **Limit Range**
   ```yaml
   lidarMaxRange: 50.0  # Reduce from 100m to 50m
   ```

3. **Adjust Mapping Frequency**
   ```yaml
   mappingProcessInterval: 0.2  # Process every 0.2 seconds
   ```

## System Architecture

```
┌─────────────────┐     ┌─────────────────┐
│  SICK MultiScan │     │ WitMotion IMU   │
│   136 LiDAR     │     │    HWT905       │
└────────┬────────┘     └────────┬────────┘
         │                       │
         │ 7.5 Hz               │ 200 Hz
         │                       │
    ┌────▼────────┐         ┌───▼────────┐
    │ sick_scan_xd│         │witmotion_ros│
    │   driver    │         │   driver    │
    └────┬────────┘         └───┬────────┘
         │                       │
         │ /cloud_all_          │ /imu/data
         │ fields_fullframe     │
         │                       │
    ┌────▼───────────────────────▼────────┐
    │           LIO-SAM Package           │
    │  ┌─────────────────────────────┐   │
    │  │     IMU Preintegration      │   │
    │  └─────────────┬───────────────┘   │
    │  ┌─────────────▼───────────────┐   │
    │  │     Image Projection        │   │
    │  └─────────────┬───────────────┘   │
    │  ┌─────────────▼───────────────┐   │
    │  │    Feature Extraction       │   │
    │  └─────────────┬───────────────┘   │
    │  ┌─────────────▼───────────────┐   │
    │  │     Map Optimization        │   │
    │  └─────────────────────────────┘   │
    └─────────────┬───────────────────────┘
                  │
         ┌────────▼────────┐
         │   Output Topics │
         │  - odometry     │
         │  - path         │
         │  - map          │
         └─────────────────┘
```

## Data Recording

To record data for offline processing:
```bash
ros2 bag record \
  /cloud_all_fields_fullframe \
  /imu/data \
  /lio_sam/mapping/odometry \
  /lio_sam/mapping/cloud_registered \
  /tf \
  /tf_static
```

## Tips for Best Results

1. **Warm-up Period**: Let the IMU run for 30-60 seconds before moving to allow bias estimation
2. **Initial Movement**: Start with slow, smooth movements
3. **Loop Closures**: Return to previously visited areas for better mapping
4. **Avoid Rapid Rotation**: The system works best with gradual turns
5. **Feature-Rich Environment**: Ensure the environment has sufficient geometric features

## Contact and Support

For issues specific to this setup:
- Check the calibration logs in `~/lio_ws/`
- Review the launch output for error messages
- Ensure all hardware connections are secure

## License

This integration uses:
- LIO-SAM: BSD License
- SICK Scanner ROS2 Driver: Apache 2.0
- WitMotion ROS Driver: MIT License

---
Last Updated: December 2024 