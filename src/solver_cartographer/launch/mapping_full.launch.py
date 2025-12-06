from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # --- Rutas de configuración ---
    params_file = os.path.join(
        get_package_share_directory('solver_localization'),
        'config',
        'robot_params.yaml'
    )

    ekf_params = os.path.join(
        get_package_share_directory('solver_localization'),
        'config',
        'ekf_localization.yaml'
    )

    cartographer_config_dir = '/home/athome/solverRos2_ws/src/solver_cartographer/config'
    cartographer_config_file = 'solver.lua'

    return LaunchDescription([
        # ============================================================
        # 🔹 1. ODOMETRÍA Y LOCALIZACIÓN BASE
        # ============================================================
        Node(
            package='solver_md49',
            executable='md49_node',
            name='md49_node',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='solver_velocity',
            executable='solver_velocity_node',
            name='solver_velocity_node',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='solver_odometry',
            executable='solver_odometry',
            name='solver_odometry',
            output='screen',
            parameters=[params_file],
        ),

        # 🔸 TF base_link → laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_laser',
            output='screen',
            arguments=[
                '--x', '0.22',
                '--y', '0.0',
                '--z', '0.0',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser'
            ],
        ),

        # 🔸 TF base_link → camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_camera',
            output='screen',
            arguments=[
                '--x', '0.10',
                '--y', '0.0',
                '--z', '1.00',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link'
            ],
        ),

        # 🔸 TF odom → base_link (temporal si no lo publica otro nodo)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_odom_base',
            output='screen',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'odom',
                '--child-frame-id', 'base_link'
            ],
        ),

        # 🔸 EKF (fusión IMU + odometría)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[ekf_params],
        ),

        # ============================================================
        # 🔹 2. SLAM 2D - CARTOGRAPHER
        # ============================================================
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', cartographer_config_file
            ],
            remappings=[
                ('/scan', '/scan'),
                ('/odom', '/odometry/filtered'),
                ('/imu', '/imu/data_fixed')
            ],
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{
                'resolution': 0.05,
                'publish_period_sec': 1.0
            }],
        ),

        # ============================================================
        # 🔹 3. SLAM 3D - RTABMAP (RGB-D)
        # ============================================================
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'subscribe_rgb': True,
                'subscribe_depth': True,
                'subscribe_scan': True,
                'approx_sync': True,
                'queue_size': 30,
                'RGBD/NeighborLinkRefining': 'true',
                'Reg/Strategy': '1',        # ICP + Visual
                'Grid/RangeMax': '6.0',
            }],
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('depth/image', '/camera/camera/depth/image_rect_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('scan', '/scan'),
                ('odom', '/odometry/filtered'),
            ],
        ),
    ])
