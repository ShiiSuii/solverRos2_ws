from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_share = FindPackageShare('solver_localization').find('solver_localization')
    ekf_config = PathJoinSubstitution([pkg_share, 'config', 'ekf_localization.yaml'])

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization',
            output='screen',
            parameters=[ekf_config]
        ),
    ])

