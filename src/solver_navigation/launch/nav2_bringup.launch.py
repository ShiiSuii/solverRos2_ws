from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    map_yaml = '/home/atwork/mapa_solver.yaml'
    params_file = '/home/atwork/solverRos2_ws/src/solver_navigation/config/nav2_params.yaml'

    return LaunchDescription([
        # === Servidor de mapa ===
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'yaml_filename': map_yaml
            }]
        ),

        # === Localización AMCL ===
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file],
        ),

        # === Planeador global ===
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
        ),

        # === Controlador local (DWB) ===
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
        ),

        # === Behavior Tree Navigator ===
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file],
        ),

        # === Lifecycle Manager ===
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'autostart': True},
                {'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'bt_navigator'
                ]}
            ],
        ),
    ])
