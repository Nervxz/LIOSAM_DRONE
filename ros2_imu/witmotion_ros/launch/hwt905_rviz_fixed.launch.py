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
    
    # Get the RViz config - using the simpler configuration
    rviz_config = os.path.join(
        get_package_share_directory('witmotion_ros'),
        'config',
        'imu_simple.rviz'
        )
    
    # IMU node
    imu_node = Node(
        package = 'witmotion_ros',
        executable = 'witmotion_ros_node',
        name = 'witmotion',
        parameters = [config]
    )
    
    # Static transform publisher - Fixed frame is map instead of world
    static_transform_map_base = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        name = 'static_transform_publisher_map_base',
        arguments = ['0', '0', '0', '0', '0', '0', 'map', 'base_link']
    )
    
    # Static transform publisher - from base_link to IMU
    static_transform_base_imu = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        name = 'static_transform_publisher_base_imu',
        arguments = ['0', '0', '0', '0', '0', '0', 'base_link', 'imu']
    )
    
    # Static transform publisher - from base_link to compass
    static_transform_base_compass = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        name = 'static_transform_publisher_base_compass',  
        arguments = ['0', '0', '0', '0', '0', '0', 'base_link', 'compass']
    )
    
    # RViz2 node with configuration for IMU visualization
    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )
    
    ld.add_action(imu_node)
    ld.add_action(static_transform_map_base)
    ld.add_action(static_transform_base_imu)
    ld.add_action(static_transform_base_compass)
    ld.add_action(rviz_node)
    
    return ld 