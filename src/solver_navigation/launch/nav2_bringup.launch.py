from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml = '/home/atwork/mapa_solver.yaml'
    params_file = '/home/atwork/solverRos2_ws/src/solver_navigation/config/nav2_params.yaml'

    return LaunchDescription([
        # MAP SERVER
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml,
                'use_sim_time': False,
            }],
        ),

        # AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file],
            remappings=[
                ('/odom', '/odometry/filtered'),
                ('/scan', '/scan'),
            ],
        ),

        # PLANNER
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
        ),

        # CONTROLLER
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
        ),

        # BEHAVIORS
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file],
        ),

        # BT NAVIGATOR (forzamos el BT BUENO acá)
        # === Behavior Tree Navigator (compatible con Humble) ===
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                params_file,
                # Usar el árbol clásico sin "RemovePassedGoals"
                {'default_bt_xml_filename': '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_w_recovery.xml'}
            ],
        ),


        # LIFECYCLE
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator',
                ]
            }],
        ),
    ])
