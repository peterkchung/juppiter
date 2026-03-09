# About: Launch file for FDIIR scenario testing
# Runs a test scenario and validates FDIIR behavior

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package paths
    pkg_integration_tests = get_package_share_directory('juppiter_integration_tests')
    pkg_gazebo_lunar_sim = get_package_share_directory('gazebo_lunar_sim')
    pkg_fusion_core = get_package_share_directory('fusion_core')
    pkg_fast_lio2 = get_package_share_directory('fast_lio2')
    pkg_dlio = get_package_share_directory('dlio')
    pkg_orb_slam3 = get_package_share_directory('orb_slam3')
    pkg_openvins = get_package_share_directory('openvins')
    pkg_kinetic_estimator = get_package_share_directory('kinetic_estimator')
    
    # Launch arguments
    scenario_file = LaunchConfiguration('scenario_file')
    compute_profile = LaunchConfiguration('compute_profile')
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'scenario_file',
            default_value=os.path.join(pkg_integration_tests, 'config', 'lidar_dropout.yaml'),
            description='Path to FDIIR test scenario YAML file'
        ),
        DeclareLaunchArgument(
            'compute_profile',
            default_value='dev',
            description='Compute tier: dev, edge, or low_power'
        ),
        
        # Step 1: Start simulation environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                pkg_gazebo_lunar_sim, '/launch/simulation.launch.py'
            ]),
            launch_arguments={
                'use_sim_time': 'true',
            }.items()
        ),
        
        # Step 2: Start kinetic estimator (after 2s)
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='kinetic_estimator',
                    executable='kinetic_estimator_node',
                    name='kinetic_estimator',
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                ),
            ]
        ),
        
        # Step 3: Start LIO (dev tier = fast_lio2, edge = dlio)
        # For now, always use fast_lio2
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='fast_lio2',
                    executable='fast_lio2_node',
                    name='fast_lio2',
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                ),
            ]
        ),
        
        # Step 4: Start VIO (dev tier = orb_slam3, edge = openvins)
        # For now, always use orb_slam3
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='orb_slam3',
                    executable='orb_slam3_node',
                    name='orb_slam3',
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                ),
            ]
        ),
        
        # Step 5: Start fusion core
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='fusion_core',
                    executable='fusion_node',
                    name='fusion_node',
                    output='screen',
                    parameters=[
                        {'use_sim_time': True},
                        {'compute_profile': compute_profile},
                    ],
                ),
            ]
        ),
        
        # Step 6: Start scenario runner (after everything is up)
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='juppiter_integration_tests',
                    executable='scenario_runner',
                    name='scenario_runner',
                    output='screen',
                    parameters=[
                        {'use_sim_time': True},
                        {'scenario_file': scenario_file},
                        {'auto_start': True},
                    ],
                ),
            ]
        ),
    ])
