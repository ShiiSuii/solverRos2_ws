from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # --- Ruta del YAML de parámetros del robot ---
    params_file = os.path.join(
        get_package_share_directory('solver_localization'),
        'config',
        'robot_params.yaml'
    )

    return LaunchDescription([
        # --- Nodo control MD49 ---
        Node(
            package='solver_md49',
            executable='md49_node',
            name='md49_node',
            output='screen',
            parameters=[params_file],
        ),

        # --- Nodo de velocidad ---
        Node(
            package='solver_velocity',
            executable='solver_velocity_node',
            name='solver_velocity_node',
            output='screen',
            parameters=[params_file],
        ),

        # --- Nodo de odometría ---
        Node(
            package='solver_odometry',
            executable='solver_odometry',
            name='solver_odometry',
            output='screen',
            parameters=[params_file],
        ),

        # --- TF: base_link -> laser ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_laser',
            output='screen',
            arguments=['0.22', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'laser'],
        ),

        # --- TF: base_link -> camera_link ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_camera',
            output='screen',
            arguments=['0.10', '0.0', '1.00', '0.0', '0.0', '0.0', 'base_link', 'camera_link'],
        ),

        # --- Nodo de fusión EKF ---
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('solver_localization'),
                'config',
                'ekf_localization.yaml'
            )],
        ),
    ])
