#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from math import pi


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

     # RP Lidar S2 Setup
    serial_port = LaunchConfiguration('serial_port', default='/dev/sl-lidar')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='1000000') #for s2 is 1000000
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='DenseBoost')
    
    
    serial_port_ld = DeclareLaunchArgument(
        'serial_port',
        default_value=serial_port,
        description='Specifying usb port to connected lidar')

    serial_baud_ld = DeclareLaunchArgument(
        'serial_baudrate',
        default_value=serial_baudrate,
        description='Specifying usb port baudrate to connected lidar')
        
    frame_ld = DeclareLaunchArgument(
        'frame_id',
        default_value=frame_id,
        description='Specifying frame_id of lidar')

    inverted_ld = DeclareLaunchArgument(
        'inverted',
        default_value=inverted,
        description='Specifying whether or not to invert scan data')

    angle_ld = DeclareLaunchArgument(
        'angle_compensate',
        default_value=angle_compensate,
        description='Specifying whether or not to enable angle_compensate of scan data')

    scan_ld = DeclareLaunchArgument(
        'scan_mode',
        default_value=scan_mode,
        description='Specifying scan mode of lidar')

    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{'serial_port': serial_port, 
                     'serial_baudrate': serial_baudrate, 
                     'frame_id': frame_id,
                     'inverted': inverted, 
                     'angle_compensate': angle_compensate, 
                     'scan_mode': scan_mode}],
        output='screen')
    
    # Add lidar setup to launch description
    ld.add_action(serial_port_ld)
    ld.add_action(serial_baud_ld)
    ld.add_action(frame_ld)
    ld.add_action(inverted_ld)
    ld.add_action(angle_ld)
    ld.add_action(scan_ld)
    ld.add_action(lidar_node)
    
    # Slam toolbox launch setup
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("roverrobotics_driver"),
                                   'config/slam_configs', 'mapper_params_localization.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
    declare_map_file = DeclareLaunchArgument(
    	'map_file_name',
    	default_value=os.path.join(get_package_share_directory("roverrobotics_driver"),
    				   'config/maps', 'serialized_office_map'))
    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    # Add slam setup to launch description
    #ld.add_action(declare_use_sim_time_argument)
    #ld.add_action(declare_slam_params_file_cmd)
    #ld.add_action(declare_map_file)
    #ld.add_action(start_async_slam_toolbox_node)
    map_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output='screen',
            arguments=['0.0', '0', '0.0', '0', '0', '0', 'map', 'odom'],
        )
    imu_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output='screen',
            arguments=['0', '0', '0.1016', '0', '0',
                       '0', 'base_link', 'base_imu_link'],
        )
    laser_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output='screen',
            arguments=['0.1524', '0', '0.2413', str(pi), '0', '0', 'base_link', 'laser'],
        )
    ld.add_action(map_tf)
    ld.add_action(imu_tf)
    ld.add_action(laser_tf)
    
    return ld

