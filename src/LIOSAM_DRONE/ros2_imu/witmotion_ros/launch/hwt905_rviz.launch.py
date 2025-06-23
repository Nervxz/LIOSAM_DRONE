import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    # Get the IMU config
    config = os.path.join(
        get_package_share_directory('witmotion_ros'),
        'config',
        'hwt905_custom.yml'
        )
    
    # IMU node
    imu_node = Node(
        package = 'witmotion_ros',
        executable = 'witmotion_ros_node',
        name = 'witmotion',
        parameters = [config]
    )
    
    # RViz2 node with configuration for IMU visualization
    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('witmotion_ros'), 'config', 'imu_viz.rviz')],
        output='screen'
    )
    
    # Static transform publisher for visualization
    static_transform = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        name = 'static_transform_publisher',
        arguments = ['0', '0', '0', '0', '0', '0', 'world', 'imu']
    )
    
    ld.add_action(imu_node)
    ld.add_action(static_transform)
    ld.add_action(rviz_node)
    
    return ld 