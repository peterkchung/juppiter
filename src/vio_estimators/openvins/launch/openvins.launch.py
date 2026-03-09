# About: Launch file for OpenVINS edge tier VIO
# MSCKF-based visual-inertial odometry

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package path
    pkg_openvins = FindPackageShare('openvins')
    
    # Launch arguments
    config_path = LaunchConfiguration('config_path')
    compute_profile = LaunchConfiguration('compute_profile')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=PathJoinSubstitution([pkg_openvins, 'config', 'openvins_simulation.yaml']),
            description='Path to OpenVINS configuration'
        ),
        DeclareLaunchArgument(
            'compute_profile',
            default_value='edge',
            description='Compute tier: edge (10Hz), dev (20Hz), low_power (5Hz)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # OpenVINS VIO Node
        Node(
            package='openvins',
            executable='openvins_node',
            name='openvins',
            output='screen',
            parameters=[
                config_path,
                {
                    'compute_profile': compute_profile,
                    'use_sim_time': use_sim_time,
                }
            ],
            remappings=[
                ('/stereo/left/image_raw', '/stereo/left/image_raw'),
                ('/stereo/right/image_raw', '/stereo/right/image_raw'),
                ('/imu/data', '/imu/data'),
                ('/vio/odom', '/vio/odom'),
                ('/vio/health', '/vio/health'),
            ],
        ),
    ])
