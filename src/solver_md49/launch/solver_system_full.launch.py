# ~/solverRos2_ws/src/solver_md49/launch/solver_system_full.launch.py
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # === RUTAS A LOS PAQUETES ===
    pkg_md49         = FindPackageShare('solver_md49').find('solver_md49')
    pkg_velocity     = FindPackageShare('solver_velocity').find('solver_velocity')
    pkg_sensors      = FindPackageShare('solver_sensors').find('solver_sensors')
    pkg_localization = FindPackageShare('solver_localization').find('solver_localization')

    # === ARCHIVOS DE CONFIGURACIÓN ===
    md49_config = PathJoinSubstitution([pkg_md49, 'config', 'md49_defaults.yaml'])
    imu_config  = PathJoinSubstitution([pkg_sensors, 'config', 'imu_serial.yaml'])
    ekf_config  = PathJoinSubstitution([pkg_localization, 'config', 'ekf_localization.yaml'])

    # === MD49 (control de motores) ===
    md49_node = Node(
        package='solver_md49',
        executable='md49_node',
        name='md49_node',
        output='screen',
        parameters=[md49_config]
    )

    # === VELOCIDAD (cinemática diferencial) ===
    velocity_node = Node(
        package='solver_velocity',
        executable='solver_velocity_node',
        name='solver_velocity_node',
        output='screen',
        parameters=[{
            'wheel_separation': 0.30,
            'wheel_radius': 0.06
        }]
    )

    # === ODOMETRÍA ===
    odometry_node = Node(
        package='solver_odometry',
        executable='odometry_node',
        name='odometry_node',
        output='screen',
        parameters=[{
            'frame_id': 'odom',
            'child_frame_id': 'base_link',
            'wheel_radius_m': 0.06,
            'track_width_m': 0.30,
            'ticks_per_rev': 360.0,
            'sample_period': 0.1
        }]
    )

    # === IMU SERIAL ===
    imu_node = Node(
        package='solver_sensors',
        executable='imu_serial_node',
        name='imu_serial_node',
        output='screen',
        parameters=[imu_config]
    )

    # === TRANSFORMACIÓN FIJA base_link → imu_link ===
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_imu',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )

    # === EKF LOCALIZATION (con retardo de arranque) ===
    ekf_delayed = TimerAction(
        period=5.0,  # Espera 5 segundos antes de lanzar el EKF
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_localization',
                output='screen',
                parameters=[ekf_config]
            )
        ]
    )

    # === ORDEN DE LANZAMIENTO ===
    return LaunchDescription([
        md49_node,
        velocity_node,
        odometry_node,
        imu_node,
        static_tf_imu,
        ekf_delayed
    ])
