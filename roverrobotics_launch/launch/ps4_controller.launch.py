from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    controller_config = Path(get_package_share_directory(
        'roverrobotics_launch'), 'config', 'controller_config.yaml')
    assert controller_config.is_file()
    topics_config = Path(get_package_share_directory(
        'roverrobotics_launch'), 'config', 'topics.yaml')
    assert topics_config.is_file()

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        Node(
            package='roverrobotics_input_manager',
            executable='joys_manager.py',
            output='screen',
            parameters=[
                {"controller": str(controller_config),
                "topics": str(topics_config)}]
        ),
        Node(
            package='joy',
            executable='joy_node',
            output='screen',
            parameters=[{'dev': '/dev/input/jsX'}]
        ),
    ])