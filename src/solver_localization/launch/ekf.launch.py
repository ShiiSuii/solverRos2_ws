from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization',
            output='screen',
            parameters=[{
                'two_d_mode': True,
                'frequency': 30.0,
                'sensor_timeout': 0.5,
                'publish_tf': True,
                'publish_acceleration': False,

                # Frames
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'odom',

                # Solo IMU por ahora
                'imu0': '/imu/data',
                'imu0_config': [False, False, False,
                                True, True, True,
                                False, False, False,
                                False, False, True,
                                False, False, False],
                'imu0_differential': False,
                'imu0_relative': False,
                'imu0_queue_size': 10
            }]
        )
    ])
