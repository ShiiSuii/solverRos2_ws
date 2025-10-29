from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        # Espera unos segundos para asegurar sincronía de TF y tópicos
        TimerAction(
            period=5.0,  # espera 5 segundos antes de lanzar el slam_toolbox
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='sync_slam_toolbox_node',
                    name='slam_toolbox',
                    parameters=['/home/atwork/solverRos2_ws/src/solver_mapping/config/slam_toolbox.yaml'],
                    remappings=[
                        ('/scan', '/scan'),
                        ('/odom', '/odometry/filtered')
                    ]
                )
            ]
        )
    ])
