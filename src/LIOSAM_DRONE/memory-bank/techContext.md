# Technical Context: LIO-SAM with SICK MultiScan & WitMotion IMU

## Project Directory Structure
- **ros2_imu_ws**: Working workspace for WitMotion HWT905 IMU driver
  - Functional driver implementation
  - Publishing IMU data to ROS 2 topics
  - RViz2 visualization configured
  
- **sick_scan_ws**: Working workspace for SICK MultiScan LiDAR driver
  - Functional driver implementation
  - Publishing point cloud data to ROS 2 topics
  - RViz2 visualization configured
  
- **lio_sam**: Workspace for LIO-SAM algorithm
  - Repository freshly cloned from GitHub
  - Needs configuration for SICK MultiScan and WitMotion integration
  - Will contain core SLAM implementation and custom configuration

## Core Technologies

### Operating System
- **Ubuntu 22.04 LTS** (Jammy Jellyfish)
- Target platforms: x86-64 (development) and ARM64 (Jetson Orin)

### ROS Framework
- **ROS 2 Humble Hawksbill** (May 2022 release, LTS support until May 2027)
- Using standard middleware (DDS)
- Using C++ and Python implementations

### SLAM Algorithm
- **LIO-SAM** (LiDAR Inertial Odometry via Smoothing And Mapping)
- Factor-graph based optimization
- Loop closure capability
- Tightly-coupled IMU integration

### Dependency Libraries
- **PCL** (Point Cloud Library) for point cloud processing
- **GTSAM** (Georgia Tech Smoothing And Mapping) for factor graph optimization
- **Ceres Solver** for non-linear optimization
- **Eigen** for linear algebra operations
- **TF2** for coordinate frame management

## Hardware Components

### LiDAR Sensor
- **SICK multiScan 100/136**
  - 3D LiDAR scanning on multiple planes
  - 10 Hz scan rate
  - Dual-return capability
  - Range: Up to 100m (MS100) / 150m (MS136)
  - TCP/IP communication protocol
  - Network connection established and operational

### IMU Sensor
- **WitMotion HWT905**
  - 9-axis IMU (3-axis accelerometer, gyroscope, magnetometer)
  - 200 Hz output rate
  - USB-UART connection at 230,400 bps
  - Connection established and operational

### Computing Platforms
- **Development**: Any x86-64 system with Ubuntu 22.04
- **Deployment**: NVIDIA Jetson Orin 8GB
  - 8-core Arm Cortex-A78AE CPU
  - NVIDIA Ampere architecture GPU with 1024 CUDA cores
  - Power management through nvpmodel (-m 2 setting)

## Implemented Software Components

### LiDAR Driver
- **sick_scan_xd** package successfully installed and configured
- Publishing point cloud data to ROS 2 topics
- Visualization in RViz2 working
- Network connection operational

### IMU Driver
- **witmotion_hwt905_driver** package successfully installed and configured
- Publishing IMU data to ROS 2 topics
- Visualization in RViz2 working
- USB-UART connection operational

## Pending Software Components

### LIO-SAM Components
- **ROS 2 port** of original LIO-SAM freshly cloned
- Needs configuration for SICK MultiScan LiDAR
- Needs parameter tuning for optimal performance
- Requires transformation setup for sensor integration

### Integrated Visualization
- Need RViz2 configuration for complete LIO-SAM system
- Need to combine visualizations of different components
- Need to display mapping and localization outputs

### Deployment System
- Systemd service for automatic startup (not started)
- Comprehensive testing framework (not started)
- Performance optimization for Jetson Orin (not started)

## Development Tools

### Build System
- **colcon** (collective construction) for ROS 2 package building
- **CMake** for C++ compilation
- **symlink-install** for efficient development workflow

### Version Control
- **Git** for source code management
- Workspace organized with multiple repositories

### Testing
- **bash** scripts for automated testing
- **ros2 bag** for data recording and playback
- **self_test** tools for system verification

### Deployment
- **systemd** for service management
- **udev** rules for device management
- **netplan** for network configuration

## Configuration Management

### Current Configuration
- Individual component configuration complete
  - IMU driver configuration
  - LiDAR driver configuration

### Planned Configuration
- LIO-SAM parameter file (`params_multiscan.yaml`) for SICK MultiScan
- Launch file for integrated system
- Static transformations between sensor frames
- RViz2 configuration for complete system visualization 