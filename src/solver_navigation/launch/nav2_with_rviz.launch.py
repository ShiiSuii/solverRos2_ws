from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os


def generate_launch_description():

    # Paths
    pkg_dir = '/home/athome/solverRos2_ws/src/solver_navigation'
    config_dir = os.path.join(pkg_dir, 'config')

    params_file = os.path.join(config_dir, 'nav2_params.yaml')
    rviz_config = os.path.join(config_dir, 'solver_nav2_full.rviz')
    map_yaml = '/home/athome/mapa_solver.yaml'

    # ========================
    # RVIZ FIRST
    # ========================
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    # ========================
    # NAV2 STACK
    # ========================
    nav2_nodes = [
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

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file],
            remappings=[('scan', '/scan')],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file],
        ),

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
    ]

    # Launch Nav2 AFTER RViz
    delayed_nav2 = TimerAction(
        period=0.5,
        actions=nav2_nodes
    )

    return LaunchDescription([
        rviz,
        delayed_nav2
    ])
