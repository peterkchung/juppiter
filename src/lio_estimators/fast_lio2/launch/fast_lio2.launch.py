# About: Launch file for FAST-LIO2
# Dev tier LIO estimator

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    pkg_fast_lio2 = get_package_share_directory('fast_lio2')
    
    # Launch arguments
    config_path = LaunchConfiguration('config_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'config_path',
            default_value=os.path.join(pkg_fast_lio2, 'config', 'fast_lio2.yaml'),
            description='Path to FAST-LIO2 configuration file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # FAST-LIO2 Node
        Node(
            package='fast_lio2',
            executable='fast_lio2_node',
            name='fast_lio2',
            output='screen',
            parameters=[
                {'config_path': config_path},
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('/lidar/points', '/lidar/points'),
                ('/imu/data', '/imu/data'),
                ('/lio/odom', '/lio/odom'),
            ],
        ),
    ])
