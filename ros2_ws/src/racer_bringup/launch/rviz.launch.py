from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory("racer_bringup")) / "rviz" / "racer.rviz")
    return LaunchDescription([
        Node(package="rviz2", executable="rviz2", arguments=["-d", config], output="screen")
    ])

