# Project Brief: LIO-SAM Integration with SICK MultiScan & WitMotion IMU

## Project Vision
Create a production-ready LIO-SAM (LiDAR-Inertial Odometry via Smoothing and Mapping) stack that fuses data from SICK MultiScan LiDAR and WitMotion IMU for precise 3D mapping and localization.

## Current State
- **ros2_imu_ws**: Fully functional WitMotion HWT905 IMU driver
  - IMU data successfully published to ROS 2 topics
  - Visualization in RViz2 working
  
- **sick_scan_ws**: Fully functional SICK MultiScan LiDAR driver
  - LiDAR data successfully published to ROS 2 topics
  - Point cloud visualization in RViz2 working
  
- **lio_sam**: LIO-SAM repository freshly cloned from GitHub
  - Needs configuration for integration with existing IMU and LiDAR
  - Needs appropriate parameter settings for the hardware

The current focus is on integrating the functional IMU and LiDAR components with the LIO-SAM algorithm to create a complete SLAM system.

## Target Platforms
- **Development**: x86-64 laptop with Ubuntu 22.04 and ROS 2 Humble
- **Deployment**: NVIDIA Jetson Orin 8GB with Ubuntu 22.04 and ROS 2 Humble

## Core Requirements

### Sensors
- **LiDAR**: SICK multiScan 100 and multiScan 136
  - 10 Hz scan rate
  - Dual-return mode
  - Ethernet connection (TCP/IP)
- **IMU**: WitMotion HWT905
  - 200 Hz sampling rate
  - USB-UART connection (230,400 bps)
  - Custom udev rule for persistent device naming (`/dev/ttyIMU`)

### Software Components
1. **SICK Driver**: Integration with `sick_scan_xd` package (IMPLEMENTED)
2. **IMU Driver**: Integration with `witmotion_hwt905_driver` package (IMPLEMENTED)
3. **LIO-SAM**: ROS 2 port with customizations for SICK MultiScan LiDAR (IN PROGRESS)
4. **Visualization**: RViz2 configuration for real-time monitoring (PARTIAL - INDIVIDUAL COMPONENTS WORKING)
5. **Deployment**: Systemd service for automatic startup (NOT STARTED)

### Performance Goals
- Robust real-time SLAM capability
- Accurate mapping with loop closure
- Low-latency sensor data processing
- Resilient to environmental variations

## Project Scope
- Complete end-to-end system integration
- Full documentation for setup, calibration, and operation
- Automated testing and verification scripts
- Production deployment configuration

## Non-Goals
- Custom driver development (using existing packages)
- Support for other LiDAR or IMU models
- Real-time object detection or classification
- Cloud-based processing or remote operation

## Success Criteria
- System boots automatically and starts mapping
- Accurate trajectory tracking with minimal drift
- Clear visualization of point cloud and trajectory
- Comprehensive documentation and test scripts 