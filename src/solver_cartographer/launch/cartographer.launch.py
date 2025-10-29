from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # --- NODO PRINCIPAL DE CARTOGRAPHER ---
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', '/home/atwork/solverRos2_ws/src/solver_cartographer/config',
                '-configuration_basename', 'solver.lua'
            ],
            remappings=[
                ('/scan', '/scan'),
                ('/odom', '/odometry/filtered'),
                ('/imu', '/imu/data_fixed')
            ],
        ),

        # --- NODO QUE PUBLICA EL MAPA ---
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{
                'resolution': 0.05,         # Tamaño de celda en metros
                'publish_period_sec': 1.0   # Frecuencia de publicación del mapa
            }],
        ),
    ])
