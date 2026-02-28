from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("follow_the_gap")
    params = os.path.join(pkg_share, "config", "straight_drive.yaml")

    return LaunchDescription([
        Node(
            package="follow_the_gap",
            executable="straight_driver",
            name="straight_driver",
            output="screen",
            parameters=[params],
        )
    ])