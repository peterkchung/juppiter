# About: Master launch file for juppiter complete system
# Brings up all components based on compute profile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package paths
    pkg_gazebo_lunar_sim = get_package_share_directory('gazebo_lunar_sim')
    pkg_fast_lio2 = get_package_share_directory('fast_lio2')
    pkg_dlio = get_package_share_directory('dlio')
    pkg_orb_slam3 = get_package_share_directory('orb_slam3')
    pkg_openvins = get_package_share_directory('openvins')
    pkg_kinetic_estimator = get_package_share_directory('kinetic_estimator')
    pkg_fusion_core = get_package_share_directory('fusion_core')
    
    # Launch arguments
    compute_profile = LaunchConfiguration('compute_profile')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Select estimators based on compute profile
    lio_pkg = 'fast_lio2'  # dev: fast_lio2, edge: dlio
    vio_pkg = 'orb_slam3'  # dev: orb_slam3, edge: openvins
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'compute_profile',
            default_value='dev',
            description='Compute tier: dev, edge, or low_power'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # Step 1: Start Gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                pkg_gazebo_lunar_sim, '/launch/simulation.launch.py'
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items()
        ),
        
        # Step 2: Start kinetic estimator (baseline wheel+IMU)
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='kinetic_estimator',
                    executable='kinetic_estimator_node',
                    name='kinetic_estimator',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                ),
            ]
        ),
        
        # Step 3: Start LIO (LiDAR-Inertial Odometry)
        # For dev tier: FAST-LIO2, edge: DLIO
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='fast_lio2',  # or 'dlio' for edge
                    executable='fast_lio2_node',
                    name='fast_lio2',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                ),
            ]
        ),
        
        # Step 4: Start VIO (Visual-Inertial Odometry)
        # For dev tier: ORB-SLAM3, edge: OpenVINS
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='orb_slam3',  # or 'openvins' for edge
                    executable='orb_slam3_node',
                    name='orb_slam3',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                ),
            ]
        ),
        
        # Step 5: Start fusion core (integrates all estimators)
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='fusion_core',
                    executable='fusion_node',
                    name='fusion_node',
                    output='screen',
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        {'compute_profile': compute_profile},
                    ],
                ),
            ]
        ),
        
        # Step 6: RViz for visualization
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', '/opt/ros/kilted/share/nav2_bringup/rviz/nav2_default_view.rviz'],
                ),
            ]
        ),
    ])
