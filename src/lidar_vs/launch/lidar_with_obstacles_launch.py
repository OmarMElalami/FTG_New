"""Combined launch file: starts the LiDAR connection and the obstacle substitution node.

Data flow:
  Car LiDAR ---[SSH/ROS2 network]---> /scan ---> obstacle_substitution ---> /obstacles

Usage:
  ros2 launch lidar_vs lidar_with_obstacles_launch.py
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    # Start the LiDAR connection to the car via the lidar_vs console script.
    # This SSH-connects to the car and starts the LiDAR, which then publishes
    # /scan data onto the ROS2 network.
    lidar_start = ExecuteProcess(
        cmd=['ros2', 'run', 'lidar_vs', 'lidar_start'],
        output='screen',
        name='lidar_vs',
    )

    # Start the obstacle substitution node after a short delay to give the
    # LiDAR time to come online and begin publishing /scan.
    # Subscribes to /scan, publishes /obstacles (ObstaclesStamped).
    obstacle_substitution = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='obstacle_substitution',
                executable='obstacle_substitution_node',
                name='obstacle_substitution',
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        lidar_start,
        obstacle_substitution,
    ])
