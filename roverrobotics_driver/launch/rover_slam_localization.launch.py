#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # RP Lidar S2 Setup
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
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
    				   'maps', 'serialized_office_map'))
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
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_map_file)
    ld.add_action(start_async_slam_toolbox_node)
    
    return ld

