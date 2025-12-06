from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config_path = '/home/athome/solverRos2_ws/config/ps4.config.yaml'

    return LaunchDescription([

        # Nodo que lee el joystick
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # Nodo que convierte joystick -> cmd_vel
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config_path],
            remappings=[
                ('/cmd_vel', '/cmd_vel')
            ],
            output='screen'
        )
    ])
