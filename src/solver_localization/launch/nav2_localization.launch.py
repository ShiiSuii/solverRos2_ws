from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    map_yaml = '/home/atwork/mapa_solver.yaml'
    params_file = '/home/atwork/solverRos2_ws/src/solver_navigation/config/nav2_params.yaml'

    return LaunchDescription([
        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file],
        ),

        # AMCL (localization)
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file],
        ),

        # Lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}],
        ),
    ])
