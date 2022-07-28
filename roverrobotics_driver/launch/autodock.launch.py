from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # urdf = Path(get_package_share_directory(
    #     'roverrobotics_description'), 'urdf', 'rover.urdf')
    # assert urdf.is_file()
    hardware_config = Path(get_package_share_directory(
        'rover_autodock'), 'config', 'autodocker_config.yaml')
    assert hardware_config.is_file()
    ld = LaunchDescription()

    robot_dock = Node(
        package = 'rover_autodock',
        executable = 'rover_autodock',
        parameters = [hardware_config],
        output='screen')

    ld.add_action(robot_dock)

    return ld
