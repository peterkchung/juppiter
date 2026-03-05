# About: Launch file for EuRoC dataset replay via new modular sensor architecture.
# Usage: ros2 launch replay.launch.py dataset_path:=/ws/data/MH_01_easy/mav0

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config",
        "euroc_cam.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dataset_path", description="Path to EuRoC mav0 directory"
            ),
            DeclareLaunchArgument(
                "playback_rate",
                default_value="1.0",
                description="Playback speed (1.0 = real-time, 0.0 = fast)",
            ),
            DeclareLaunchArgument(
                "loop", default_value="false", description="Loop dataset on completion"
            ),
            # Sensor Bridge Orchestrator Node
            Node(
                package="sensor_core",
                executable="sensor_bridge_node",
                name="sensor_bridge",
                output="screen",
                parameters=[
                    {
                        "providers": ["euroc_provider::EurocDriver"],
                        "calibration_version": "v1.0.0-euroc-default",
                        "monitor_rate_hz": 10.0,
                        "target_skew_ms": 5.0,
                        "max_skew_ms": 10.0,
                        "nominal_health_threshold": 0.75,
                        "degraded_health_threshold": 0.55,
                        # EuRoC provider parameters
                        "dataset_path": LaunchConfiguration("dataset_path"),
                        "playback_rate": LaunchConfiguration("playback_rate"),
                        "loop": LaunchConfiguration("loop"),
                        "publish_depth": True,
                        "frame_id": "camera_link",
                    },
                ],
            ),
        ]
    )
