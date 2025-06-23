import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the IMU config
    config = os.path.join(
        get_package_share_directory('witmotion_ros'),
        'config',
        'hwt905_custom.yml'
    )

    # Create the launch description
    ld = LaunchDescription()

    # IMU Node
    imu_node = Node(
        package='witmotion_ros',
        executable='witmotion_ros_node',
        name='witmotion',
        parameters=[config]
    )

    # Static transform publisher for odom -> base_link
    odom_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    # Static transform publisher for base_link -> imu
    base_link_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu']
    )

    # Static transform publisher for base_link -> compass
    base_link_to_compass = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_compass',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'compass']
    )

    # Add a very basic RViz configuration
    rviz_config_path = os.path.join(
        get_package_share_directory('witmotion_ros'),
        'config',
        'imu_minimal.rviz'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Add all nodes to the launch description
    ld.add_action(imu_node)
    ld.add_action(odom_to_base_link)
    ld.add_action(base_link_to_imu)
    ld.add_action(base_link_to_compass)
    ld.add_action(rviz_node)

    return ld 