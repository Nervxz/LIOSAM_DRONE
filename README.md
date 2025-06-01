# NERV ROS2 Workspace

This ROS2 workspace contains various packages for robotics development, including LiDAR-Inertial Odometry and SLAM capabilities.

## Contents

- **LIO-SAM**: LiDAR-Inertial Odometry via Smoothing and Mapping
- **GTSAM**: Georgia Tech Smoothing and Mapping library
- **sick_scan_ws**: SICK sensor drivers
- **ros2_imu**: IMU integration packages

## Building the Workspace

```bash
# Source ROS2
source /opt/ros/humble/setup.bash  # or your ROS2 distribution

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

## Usage

[Add specific usage instructions for your packages here]

## Dependencies

- ROS2 (Humble or later recommended)
- GTSAM
- PCL (Point Cloud Library)
- Eigen3

## License

[Add your license information here]
