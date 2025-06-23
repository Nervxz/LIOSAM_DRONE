# Witmotion HWT905 IMU Setup with ROS2

This guide will help you set up and visualize your Witmotion HWT905 IMU with ROS2.

## Hardware Specifications

- Model: HWT905
- Bandwidth: 256Hz
- Baud Rate: 230400
- Output Range: 200Hz
- Device Address: 0x50
- Algorithm: 9-axis

## Installation

### 1. Install Required Packages

```bash
# Install ROS2 IMU plugins
sudo apt-get install -y ros-humble-rviz-imu-plugin

# Install IMU tools for data processing
sudo apt-get install -y ros-humble-imu-tools ros-humble-imu-filter-madgwick

# Install robot state publisher
sudo apt-get install -y ros-humble-robot-state-publisher
```

### 2. Check Device Connection

Make sure your IMU is connected and accessible:

```bash
ls -l /dev/ttyUSB*
```

You should see something like:
```
crw-rw-rw- 1 root dialout 188, 0 May 17 21:15 /dev/ttyUSB0
```

### 3. Verify User Permissions

Ensure your user has permission to access the serial port:

```bash
id -nG $USER | grep -q "dialout" && echo "User has dialout access" || echo "User does not have dialout access"
```

If you don't have dialout access, add your user to the dialout group:

```bash
sudo usermod -a -G dialout $USER
```

Then log out and log back in for the change to take effect.

## Running the IMU

There are several launch files available, each with different configurations:

### Basic IMU Launch

This launches just the IMU node with your custom configuration:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch witmotion_ros hwt905_custom.launch.py
```

### Full Visualization with Madgwick Filter

This is the recommended setup for IMU visualization. It includes:
- IMU node with custom configuration
- TF frames setup
- Madgwick filter for improved orientation
- RViz2 for visualization

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch witmotion_ros final_demo.launch.py
```

When RViz2 starts, follow these steps:
1. Set the Fixed Frame to "odom" in the Global Options at the top
2. Click "Add" in the bottom left corner
3. Navigate to "By display type" > "rviz_imu_plugin" > "Imu"
4. Click "OK" to add the IMU display
5. In the IMU display properties:
   - Set Topic to "/imu/data" for filtered data or "/imu" for raw data
   - Under Box properties, enable the Box visualization
   - Under Axes properties, enable Axes visualization

## Configuration

The IMU is configured in the `hwt905_custom.yml` file. The key settings are:

```yaml
witmotion:
  ros__parameters:
    port: ttyUSB0
    baud_rate: 230400  # Set to 230400 as specified
    polling_interval: 5  # ms
    timeout_ms: 500  # ms
    restart_service_name: /restart_imu
    imu_publisher:
      topic_name: /imu
      frame_id: imu
      use_native_orientation: true  # Using 9-axis algorithm as specified
```

## Understanding Coordinate Frames

The IMU visualization depends on a proper setup of coordinate frames:

```
world → odom → base_link → imu
                       └-→ compass
```

These frames are set up by the launch file using static transform publishers.

## Troubleshooting

### IMU not publishing data

Check if the IMU is publishing data:

```bash
ros2 topic list
ros2 topic echo /imu --once
```

### RViz2 not showing IMU

If RViz2 is running but not showing the IMU:
1. Make sure you've added the IMU display
2. Check that you've selected the correct topic
3. Verify that the transform frames are correctly published:
   ```bash
   ros2 run tf2_ros tf2_echo odom imu
   ```

### Improved Visualization

For better IMU visualization, try using the Madgwick filter topics:
- `/imu/data` for filtered orientation
- `/imu/mag` for magnetometer data

## Advanced Usage

### Custom Launch Files

You can create your own launch files based on the provided examples:
- `hwt905_custom.launch.py`: Basic IMU node
- `hwt905_minimal.launch.py`: IMU with basic visualization
- `final_demo.launch.py`: Full setup with Madgwick filter

### Data Analysis

To record and analyze IMU data:

```bash
ros2 bag record /imu /imu/data /magnetometer
```

## References

- [Witmotion ROS2 Documentation](https://github.com/ElettraSciComp/witmotion_IMU_ros/tree/ros2)
- [RViz IMU Plugin](https://github.com/ros2-gbp/rviz_imu_plugin-release)
- [IMU Madgwick Filter](http://wiki.ros.org/imu_filter_madgwick) 