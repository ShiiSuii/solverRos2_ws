from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="solver_balloons",
            executable="balloon_detector",
            output="screen"
        )
    ])
