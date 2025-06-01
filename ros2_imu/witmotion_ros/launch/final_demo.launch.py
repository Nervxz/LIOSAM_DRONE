import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

"""
This launch file starts:
1. The Witmotion IMU node with your custom configuration
2. TF2 transform publishers for all needed coordinate frames
3. The imu_filter_madgwick node to process IMU data
4. RViz2 with a default configuration

When RViz2 starts, you'll need to:
1. Set the Fixed Frame to "odom" (top of Displays panel)
2. Add an IMU display (Add -> rviz_imu_plugin -> Imu)
3. Set the IMU Topic to "/imu/data" (processed by Madgwick filter)
   or "/imu" for raw IMU data
4. Enable "Box" and "Axes" in the IMU display properties
"""

def generate_launch_description():
    # Get the IMU config
    config_path = os.path.join(
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
        parameters=[config_path],
        output='screen'
    )
    
    # Transform publishers for each link in the TF tree
    world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
    )
    
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
    
    base_link_to_compass = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_compass',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'compass']
    )
    
    # IMU filter for improved orientation estimation
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[{
            'use_mag': True,
            'world_frame': 'enu',
            'publish_tf': False,
            'use_sim_time': False,
            'gain': 0.1,
            'zeta': 0.3,
            'mag_bias_x': 0.0,
            'mag_bias_y': 0.0,
            'mag_bias_z': 0.0,
            'orientation_stddev': 0.01
        }],
        remappings=[
            ('imu/data_raw', '/imu'),
            ('imu/mag', '/magnetometer')
        ],
        output='screen'
    )
    
    # Print instructions to terminal
    instructions = ExecuteProcess(
        cmd=['echo', 
             '\n\n=== INSTRUCTIONS ===\n' +
             'When RViz2 starts, follow these steps:\n' +
             '1. Set the Fixed Frame to "odom" (top of Displays panel)\n' +
             '2. Add an IMU display (Add -> rviz_imu_plugin -> Imu)\n' +
             '3. Set the IMU Topic to "/imu/data"\n' +
             '4. Enable "Box" and "Axes" in the IMU display properties\n' +
             '====================\n'],
        output='screen'
    )
    
    # Define the RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    # Add all nodes to the launch description
    ld.add_action(instructions)
    ld.add_action(imu_node)
    ld.add_action(world_to_odom)
    ld.add_action(odom_to_base_link)
    ld.add_action(base_link_to_imu)
    ld.add_action(base_link_to_compass)
    ld.add_action(imu_filter)
    #ld.add_action(rviz_node)
    
    return ld 
