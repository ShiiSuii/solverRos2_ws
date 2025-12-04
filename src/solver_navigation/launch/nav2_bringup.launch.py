from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    config_dir = '/home/atwork/solverRos2_ws/src/solver_navigation/config'
    map_yaml = '/home/atwork/mapa_solver.yaml'
    params_file = os.path.join(config_dir, 'nav2_params.yaml')
    bt_xml = os.path.join(config_dir, 'bt_trees', 'solver_bt.xml')

    return LaunchDescription([
        # === MAP SERVER ===
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml,
                'use_sim_time': False
            }],
        ),

        # === AMCL ===
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file],
            remappings=[('scan', '/scan')],
        ),

        # === PLANNER SERVER ===
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
        ),

        # === CONTROLLER SERVER ===
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
        ),

        # === BEHAVIOR SERVER ===
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file],
        ),

        # === BT NAVIGATOR ===
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file],
        ),

        # === LIFECYCLE MANAGER ===
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'bond_timeout': 5.0,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator'
                ]
            }],
        ),
    ])
