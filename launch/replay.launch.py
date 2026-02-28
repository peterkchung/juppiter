# About: Launch file for EuRoC dataset replay via sensor_bridge.
# Usage: ros2 launch replay.launch.py dataset_path:=/ws/data/MH_01_easy/mav0

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'config', 'euroc_cam.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'dataset_path',
            description='Path to EuRoC mav0 directory'
        ),
        DeclareLaunchArgument(
            'playback_rate',
            default_value='1.0',
            description='Playback speed (1.0 = real-time, 0.0 = fast)'
        ),
        DeclareLaunchArgument(
            'loop',
            default_value='false',
            description='Loop dataset on completion'
        ),
        Node(
            package='sensor_bridge',
            executable='sensor_bridge_node',
            name='sensor_bridge_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'dataset_path': LaunchConfiguration('dataset_path'),
                    'playback_rate': LaunchConfiguration('playback_rate'),
                    'loop': LaunchConfiguration('loop'),
                },
            ],
        ),
    ])
