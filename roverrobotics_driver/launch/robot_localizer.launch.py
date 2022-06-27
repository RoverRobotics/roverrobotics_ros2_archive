#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # IMU and Robot_localization Nodes
    config_path = Path(get_package_share_directory("roverrobotics_driver"), "config",
                          "bno_conf.yaml")

    imu_node = Node(
        package='bno055_driver', 
        executable='bno055_driver', 
        parameters=[config_path]
        )
        
    robot_localization_file_path = Path(get_package_share_directory(
        'roverrobotics_driver'), 'config/ekf.yaml')

     # Start robot localization using an Extended Kalman filter
    localization_node = Node(
    	package='robot_localization',
    	executable='ekf_node',
    	name='ekf_filter_node',
    	output='screen',
    	parameters=[robot_localization_file_path]
    	)

    ld.add_action(imu_node)
    ld.add_action(localization_node)
    
    return ld

