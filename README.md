# LIO-SAM with SICK MultiScan 136 and WitMotion HWT905

This guide provides comprehensive instructions for setting up and running LIO-SAM (LiDAR Inertial Odometry via Smoothing and Mapping) with SICK MultiScan 136 LiDAR and WitMotion HWT905 IMU.

## Quick Start

If you have Ubuntu 22.04 with ROS2 Humble already installed:

```bash
# Clone and build the project
mkdir -p ~/lio_ws && cd ~/lio_ws
git clone https://github.com/Nervxz/LIOSAM_DRONE.git .
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# Run the system (3 terminals needed)
# Terminal 1: LiDAR
ros2 launch sick_scan_xd sick_multiscan.launch.py hostname:=192.168.31.240 udp_receiver_ip:=192.168.31.146 #hostname= lidar ip; udp_ip= received ip
# Or use the convenience script:
./run_lidar.sh

# Terminal 2: IMU  
ros2 launch witmotion_ros final_demo.launch.py

# Terminal 3: LIO-SAM
ros2 launch lio_sam run.launch.py
```

For detailed installation from scratch, see the full guide below.

## Table of Contents
- [Quick Start](#quick-start)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation](#installation)
  - [System Prerequisites](#1-system-prerequisites)
  - [ROS2 Installation](#2-install-ros2-humble)
  - [Dependencies](#3-install-system-dependencies)
  - [Workspace Setup](#5-create-workspace-and-clone-repository)
  - [Building](#7-build-gtsam-library)
  - [Hardware Setup](#14-hardware-setup)
  - [Troubleshooting Build Issues](#troubleshooting-build-issues)
- [Configuration](#configuration)
- [Running the System](#running-the-system)
- [Visualization in RViz](#visualization-in-rviz)
- [Troubleshooting](#troubleshooting)
- [System Architecture](#system-architecture)

## Hardware Requirements

- **LiDAR**: SICK MultiScan 136
  - IP Address: 192.168.31.240
  - UDP Receiver IP: 192.168.31.146
  - Data Rate: ~7.5 Hz
  
- **IMU**: WitMotion HWT905
  - Connection: USB Serial
  - Data Rate: 200 Hz
  - Device: `/dev/ttyUSB0` (may vary)

## Software Requirements

### Operating System
- **Ubuntu 22.04 LTS (Jammy)** - Recommended
- **Ubuntu 20.04 LTS (Focal)** - Also supported
- **Minimum 8GB RAM** - 16GB recommended for smooth building
- **20GB free disk space** for complete installation

### Core Dependencies
- **ROS2 Humble** - Latest LTS version with full desktop installation
- **GTSAM 4.1+** - Georgia Tech Smoothing and Mapping library
- **PCL 1.10+** - Point Cloud Library for 3D data processing
- **Eigen3 3.3+** - Linear algebra library
- **OpenCV 4.2+** - Computer vision library
- **Boost 1.71+** - C++ libraries collection

### Build Tools
- **CMake 3.16+** - Build system generator
- **GCC 9+** or **Clang 10+** - C++ compiler with C++17 support
- **Python 3.8+** - For ROS2 and build tools
- **Colcon** - ROS2 build tool

### Hardware Interface Libraries
- **Serial communication libraries** - For WitMotion IMU
- **Network libraries** - For SICK LiDAR Ethernet communication

### Optional but Recommended
- **RViz2** - 3D visualization (included in ros-humble-desktop)
- **rqt tools** - ROS2 GUI tools (included in ros-humble-desktop)
- **Plotjuggler** - Data visualization tool
- **Foxglove Studio** - Modern robotics visualization

## Installation

This guide assumes you're starting from a fresh Ubuntu system. Follow these steps in order for a complete installation.

### Estimated Time Requirements
- **Fresh Ubuntu + ROS2 Installation**: 30-45 minutes
- **Dependencies Installation**: 15-20 minutes
- **Workspace Clone and Build**: 20-30 minutes (depending on hardware)
- **Hardware Setup and Testing**: 10-15 minutes
- **Total**: 1.5-2 hours for complete setup

### 1. System Prerequisites

```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install essential system tools
sudo apt install -y \
  curl \
  wget \
  gnupg2 \
  lsb-release \
  software-properties-common \
  apt-transport-https \
  ca-certificates
```

### 2. Install ROS2 Humble

```bash
# Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package index
sudo apt update

# Install ROS2 Humble Desktop
sudo apt install -y ros-humble-desktop

# Install additional ROS2 packages
sudo apt install -y \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-tf2-eigen \
  ros-humble-tf2-sensor-msgs \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-robot-localization \
  ros-humble-nav2-common \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup

# Install colcon build tools
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool
```

### 3. Install System Dependencies

```bash
# Install build tools and libraries
sudo apt install -y \
  build-essential \
  cmake \
  git \
  libeigen3-dev \
  libboost-all-dev \
  libtbb-dev \
  libpcl-dev \
  libopencv-dev \
  libgoogle-glog-dev \
  libgflags-dev \
  libatlas-base-dev \
  libomp-dev

# Install Python dependencies
sudo apt install -y \
  python3-pip \
  python3-numpy \
  python3-matplotlib \
  python3-yaml

# Install GTSAM dependencies
sudo apt install -y \
  libboost-all-dev \
  libtbb-dev \
  libmetis-dev
```

### 4. Setup ROS2 Environment

```bash
# Add ROS2 sourcing to bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Source ROS2 for current session
source /opt/ros/humble/setup.bash

# Initialize rosdep
sudo rosdep init
rosdep update
```

### 5. Create Workspace and Clone Repository

```bash
# Create workspace directory
mkdir -p ~/lio_ws
cd ~/lio_ws

# Clone the complete LIO-SAM project
git clone https://github.com/Nervxz/LIOSAM_DRONE.git .

# Verify the structure
ls -la
# You should see: src/, README.md, visualize/, *.py, *.sh files
```

### 6. Install Dependencies with rosdep

```bash
# Install all ROS dependencies
cd ~/lio_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 7. Build GTSAM Library

```bash
cd ~/lio_ws

# Build GTSAM first (required by LIO-SAM)
colcon build --packages-select gtsam --cmake-args \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON

# Source the workspace
source install/setup.bash
```

### 8. Build SICK Scanner Driver

```bash
# Build SICK scanner driver
colcon build --packages-select sick_scan_xd

# Source again after build
source install/setup.bash
```

### 9. Build WitMotion IMU Driver

```bash
# Build WitMotion IMU driver  
colcon build --packages-select witmotion_ros

# Source again after build
source install/setup.bash
```

### 10. Build LIO-SAM

```bash
# Build LIO-SAM package
colcon build --packages-select lio_sam

# Final sourcing
source install/setup.bash
```

### 11. Build All Packages (Alternative)

If you prefer to build everything at once:

```bash
cd ~/lio_ws

# Build all packages in dependency order
colcon build --symlink-install --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON

# Source the complete workspace
source install/setup.bash
```

### 12. Verify Installation

```bash
# Check if all packages built successfully
colcon list

# Verify ROS2 can find the packages
ros2 pkg list | grep -E "(lio_sam|sick_scan|witmotion)"

# Should show:
# lio_sam
# sick_scan_xd  
# witmotion_ros
```

### 13. Setup Auto-sourcing (Recommended)

```bash
# Add workspace sourcing to bashrc
echo "source ~/lio_ws/install/setup.bash" >> ~/.bashrc

# Source for current session
source ~/.bashrc
```

### 14. Hardware Setup

#### SICK MultiScan 136 LiDAR Setup
```bash
# Configure network interface for LiDAR
sudo ip addr add 192.168.31.146/24 dev eth0  # Replace eth0 with your interface

# Test connection
ping 192.168.31.240
# Should get replies if LiDAR is connected
```

#### WitMotion IMU Setup
```bash
# Check if IMU is detected
ls -la /dev/ttyUSB*

# Set permissions (may need to run after each reboot)
sudo chmod 666 /dev/ttyUSB0

# Add user to dialout group (permanent solution)
sudo usermod -a -G dialout $USER
# Log out and back in for this to take effect
```

### 15. Build Verification Test

```bash
# Quick test to verify everything works
cd ~/lio_ws

# Test SICK scanner launch file exists
ls install/sick_scan_xd/share/sick_scan_xd/launch/sick_multiscan.launch.py

# Test LIO-SAM launch file exists  
ls install/lio_sam/share/lio_sam/launch/run.launch.py

# Test WitMotion launch file exists
ls ros2_imu/witmotion_ros/launch/final_demo.launch.py

# All should exist if build was successful
```

### Troubleshooting Build Issues

#### Common Build Errors and Solutions:

**GTSAM Build Error:**
```bash
# If GTSAM fails to build, try:
sudo apt install -y libtbb-dev libmetis-dev
cd ~/lio_ws
rm -rf build/ install/ log/
colcon build --packages-select gtsam --cmake-args -DGTSAM_USE_SYSTEM_EIGEN=ON
```

**Missing Dependencies:**
```bash
# If you get missing package errors:
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

**Permission Errors:**
```bash
# If you get permission errors:
sudo chown -R $USER:$USER ~/lio_ws
```

**Build Memory Issues:**
```bash
# If build fails due to memory (common on systems with <8GB RAM):
colcon build --parallel-workers 1 --packages-select gtsam
colcon build --parallel-workers 2  # For other packages
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
- Hostname: 192.168.31.240
- UDP Receiver IP: 192.168.31.146
- Cloud topic: `/cloud_all_fields_fullframe`

You can also use the convenience script for launching:
```bash
./run_lidar.sh
```

### 3. WitMotion IMU Configuration

The IMU uses a custom launch file located at:
```
~/lio_ws/ros2_imu/witmotion_ros/launch/final_demo.launch.py
```

This launch file publishes to `/imu/data` by default. No additional configuration needed.

### 4. Convenience Scripts

For easier system startup, use these convenience scripts:

**LiDAR Launch Script:**
```bash
./run_lidar.sh
```
This script automatically launches the SICK scanner with the correct network parameters.

**Manual Launch Commands:**
```bash
# LiDAR (full command)
ros2 launch sick_scan_xd sick_multiscan.launch.py hostname:=192.168.31.240 udp_receiver_ip:=192.168.31.146

# IMU (custom launch file)
ros2 launch witmotion_ros final_demo.launch.py

# LIO-SAM (standard launch)
ros2 launch lio_sam run.launch.py
```

## Running the System

### Step-by-Step Launch Procedure

#### 1. Source the workspace
```bash
cd ~/lio_ws
source install/setup.bash
```

#### 2. Launch SICK LiDAR Scanner
```bash
ros2 launch sick_scan_xd sick_multiscan.launch.py hostname:=192.168.31.240 udp_receiver_ip:=192.168.31.146
```
Or use the convenience script:
```bash
./run_lidar.sh
```
Wait for "Startup sequence completed" message.

#### 3. Launch WitMotion IMU
```bash
ros2 launch witmotion_ros final_demo.launch.py
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
ping 192.168.31.240

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
