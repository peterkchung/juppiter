# About: Launch file for ORB-SLAM3
# Dev tier VIO estimator

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_orb_slam3 = get_package_share_directory('orb_slam3')
    
    config_path = LaunchConfiguration('config_path')
    vocab_path = LaunchConfiguration('vocabulary_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=os.path.join(pkg_orb_slam3, 'config', 'orb_slam3.yaml'),
            description='Path to ORB-SLAM3 configuration'
        ),
        DeclareLaunchArgument(
            'vocabulary_path',
            default_value='config/orb_vocab.bin',
            description='Path to ORB vocabulary file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        Node(
            package='orb_slam3',
            executable='orb_slam3_node',
            name='orb_slam3',
            output='screen',
            parameters=[
                {'config_path': config_path},
                {'vocabulary_path': vocab_path},
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
