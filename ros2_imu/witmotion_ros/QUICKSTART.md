# HWT905 IMU Quick Start Guide

## 1. Install Required Packages

```bash
sudo apt-get install -y ros-humble-rviz-imu-plugin ros-humble-imu-tools ros-humble-imu-filter-madgwick ros-humble-robot-state-publisher
```

## 2. Setup

Ensure your IMU is connected to `/dev/ttyUSB0` and you have permissions:

```bash
ls -l /dev/ttyUSB*
id -nG $USER | grep -q "dialout" && echo "User has access" || echo "No access"
```

If you don't have access, run:
```bash
sudo usermod -a -G dialout $USER  # Then log out and back in
```

## 3. Run the IMU with Visualization

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch witmotion_ros final_demo.launch.py
```

## 4. Configure RViz2

When RViz2 starts:

1. Set "Fixed Frame" to "odom" in Global Options
2. Click "Add" button
3. Select "By display type" → "rviz_imu_plugin" → "Imu"
4. Configure the IMU display:
   - Set Topic to "/imu/data"
   - Enable Box and Axes visualization

## 5. Troubleshooting

If the IMU doesn't appear:
- Check if data is being published: `ros2 topic echo /imu --once`
- Verify transforms: `ros2 run tf2_ros tf2_echo odom imu`

## 6. Configuration

Your HWT905 is configured with:
- Baud rate: 230400
- Bandwidth: 256Hz
- Output rate: 200Hz
- 9-axis algorithm enabled 