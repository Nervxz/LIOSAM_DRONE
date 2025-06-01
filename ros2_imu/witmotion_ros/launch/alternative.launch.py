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
    
    # Robot state publisher - publishes fixed transforms
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    # Start RViz with a blank configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('rviz2'), 'default.rviz')],
        name='rviz2',
        output='screen'
    )
    
    # Add necessary transforms
    odom_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )
    
    base_link_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu']
    )
    
    world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
    )
    
    # Add imu_tools package nodes if available
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[{
            'use_mag': False,
            'publish_tf': True,
            'world_frame': 'enu',
            'use_sim_time': False
        }]
    )
    
    # Add nodes to launch description
    ld.add_action(imu_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(world_to_odom)
    ld.add_action(odom_to_base_link)
    ld.add_action(base_link_to_imu)
    ld.add_action(imu_filter_node)
    ld.add_action(rviz_node)
    
    return ld 