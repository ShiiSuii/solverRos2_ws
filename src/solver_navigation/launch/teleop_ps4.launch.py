from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_path = '/opt/ros/humble/share/teleop_twist_joy/config/ps4.config.yaml'

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',  # <- este nombre DEBE coincidir
            parameters=[config_path],
            remappings=[
                ('/cmd_vel', '/cmd_vel')
            ],
            output='screen'
        )
    ])

