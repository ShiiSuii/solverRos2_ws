from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='solver_md49',
            executable='md49_node',
            name='md49_node',
            output='screen',
            parameters=['src/solver_md49/config/md49_defaults.yaml']
        ),
        Node(
            package='solver_odometry',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            parameters=['src/solver_odometry/config/hardware.yaml']
        )
    ])
