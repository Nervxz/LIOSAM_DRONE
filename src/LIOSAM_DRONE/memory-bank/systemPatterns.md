# System Patterns: LIO-SAM with SICK MultiScan & WitMotion IMU

## System Architecture

The system follows a modular architecture with clear separation of concerns:

```
                      +-----------------+
                      |     ROS 2       |
                      |   Environment   |
                      +-----------------+
                              |
           +------------------+------------------+
           |                  |                  |
+----------v----------+ +-----v------+ +---------v---------+
|    SICK LiDAR       | |  WitMotion | |     LIO-SAM       |
|    Driver           | |  IMU Driver| |                   |
| (sick_scan_xd)      | |            | |                   |
+---------------------+ +------------+ +-------------------+
```

## Key Components

### 1. Sensor Drivers
- **SICK MultiScan Driver**: Interfaces with LiDAR over Ethernet, publishes point cloud data
- **WitMotion Driver**: Interfaces with IMU over USB-UART, publishes IMU data

### 2. LIO-SAM Core Nodes
- **imageProjection**: Processes raw point cloud data and applies motion correction
- **featureExtraction**: Extracts edge and planar features from point cloud
- **mapOptimization**: Performs loop closure and optimizes the map
- **imuPreintegration**: Handles IMU data and performs preintegration

### 3. Transformation System
- **tf2**: Manages coordinate frames and transformations
- **static_transform_publisher**: Provides fixed transformations between sensor frames

### 4. Visualization
- **rviz2**: Provides real-time visualization of mapping and localization

## Physical Architecture

Hardware setup with correct connections:

```
+----------------+                 +---------------+                 +------------------+
|                |  Ethernet Cable |               | Ethernet Cable  |                  |
| SICK MultiScan |---------------->| Network Switch|---------------->| Jetson Orin/PC   |
| (192.168.1.100)|                 | (Gigabit)     |                 | (192.168.1.50)   |
+----------------+                 +---------------+                 +------------------+
                                                                            |
                                                                            | USB Cable
                                                                            v
                                                                     +------------------+
                                                                     | WitMotion IMU    |
                                                                     | (USB-UART)       |
                                                                     +------------------+
```

## Data Flow

1. **Point Cloud Data Flow**
   - SICK MultiScan LiDAR → sick_scan_xd driver → `/cloud` topic → LIO-SAM `imageProjection` → Feature extraction → Mapping
   
2. **IMU Data Flow**
   - WitMotion IMU → witmotion_hwt905_driver → `/imu/data` topic → LIO-SAM `imuPreintegration` → Mapping
   
3. **Transformation Flow**
   - Static transforms define relationships between frames
   - Dynamic transforms published by LIO-SAM tracking
   - Full TF tree maintained for visualization and coordination

## Design Patterns

1. **Publisher-Subscriber Pattern**
   - Used throughout for loose coupling between components
   - Enables modular development and testing

2. **Parameter Configuration**
   - Centralized parameter files for each component
   - Runtime configuration through ROS 2 parameter system

3. **Service-oriented Architecture**
   - Services used for on-demand operations (e.g., map saving)
   - Clear API boundaries between components

4. **Launch System**
   - Single entry point via launch file
   - Conditional component startup based on parameters

## Technical Decisions

1. **Frame IDs**
   - `base_lidar`: LiDAR sensor frame
   - `imu_link`: IMU sensor frame
   - `base_link`: Robot body frame
   - `odom`: Odometry frame
   - `map`: Global map frame

2. **Coordinate Systems**
   - ENU (East-North-Up) standard for ROS 2
   - Consistent transformation chains

3. **Calibration Approach**
   - Kalibr-based LiDAR-IMU calibration
   - Manual fallback for basic setups

4. **Persistence**
   - Systemd service for automatic startup
   - Configuration stored in YAML files 