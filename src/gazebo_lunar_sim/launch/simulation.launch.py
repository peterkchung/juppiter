# About: Launch file for lunar rover simulation
# Integrates Gazebo Harmonic, ROS 2 bridge, and navigation stack

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package paths
    pkg_gazebo_lunar_sim = FindPackageShare('gazebo_lunar_sim').find('gazebo_lunar_sim')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file', default='lunar_flat.sdf')
    compute_profile = LaunchConfiguration('compute_profile', default='dev')
    
    # Robot description from Xacro
    robot_description = Command([
        'xacro ',
        os.path.join(pkg_gazebo_lunar_sim, 'urdf', 'rover.urdf.xacro'),
    ])
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='lunar_flat.sdf',
            description='Gazebo world file'
        ),
        DeclareLaunchArgument(
            'compute_profile',
            default_value='dev',
            description='Compute tier: dev, edge, or low_power'
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }]
        ),
        
        # Gazebo Simulator
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('ros_gz_sim').find('ros_gz_sim'),
                '/launch/gz_sim.launch.py'
            ]),
            launch_arguments={
                'gz_args': [
                    '-r ',
                    os.path.join(pkg_gazebo_lunar_sim, 'worlds', world_file),
                ],
                'on_exit_shutdown': 'true',
            }.items(),
        ),
        
        # ROS-Gazebo Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            parameters=[{
                'config_file': os.path.join(
                    pkg_gazebo_lunar_sim, 'config', 'bridge.yaml'
                ),
            }],
        ),
        
        # Kinetic Estimator (robot_localization)
        Node(
            package='kinetic_estimator',
            executable='kinetic_estimator_node',
            name='kinetic_estimator',
            output='screen',
            parameters=[
                os.path.join(pkg_gazebo_lunar_sim, '..', '..', 'kinetic_estimator', 'config', 'kinetic_estimator.yaml'),
                {'use_sim_time': use_sim_time},
            ],
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_gazebo_lunar_sim, 'config', 'rviz.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz', default='true')),
        ),
    ])
