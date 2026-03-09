# About: Launch file for OpenVINS
# Edge tier VIO estimator

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_openvins = get_package_share_directory('openvins')
    
    config_path = LaunchConfiguration('config_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=os.path.join(pkg_openvins, 'config', 'openvins.yaml'),
            description='Path to OpenVINS configuration'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        Node(
            package='openvins',
            executable='openvins_node',
            name='openvins',
            output='screen',
            parameters=[
                {'config_path': config_path},
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('/stereo/left/image_raw', '/stereo/left/image_raw'),
                ('/stereo/right/image_raw', '/stereo/right/image_raw'),
                ('/imu/data', '/imu/data'),
                ('/vio/odom', '/vio/odom'),
            ],
        ),
    ])
