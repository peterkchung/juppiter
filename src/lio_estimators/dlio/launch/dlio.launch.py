# About: Launch file for DLIO
# Edge tier LIO estimator

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dlio = get_package_share_directory('dlio')
    
    config_path = LaunchConfiguration('config_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=os.path.join(pkg_dlio, 'config', 'dlio.yaml'),
            description='Path to DLIO configuration'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        Node(
            package='dlio',
            executable='dlio_node',
            name='dlio',
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
