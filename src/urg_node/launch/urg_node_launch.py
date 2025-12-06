import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    urg_node_dir = get_package_share_directory('urg_node')
    
    launch_description = LaunchDescription([
        DeclareLaunchArgument(
            'sensor_interface',
            default_value='ethernet',
            description='sensor_interface: supported: serial, ethernet'
        )
    ])

    # Expande el archivo YAML correcto según el modo de conexión
    def expand_param_file_name(context):
        param_file = os.path.join(
            urg_node_dir,
            'launch',
            'urg_node_' + context.launch_configurations['sensor_interface'] + '.yaml'
        )
        if os.path.exists(param_file):
            return [SetLaunchConfiguration('param', param_file)]
        else:
            raise FileNotFoundError(f"URG parameter file not found: {param_file}")

    launch_description.add_action(
        OpaqueFunction(function=expand_param_file_name)
    )

    # Nodo corregido para ROS2 Jazzy
    hokuyo_node = Node(
        package='urg_node',
        executable='urg_node_driver',   # ← CORREGIDO PARA JAZZY
        name='urg_node',
        output='screen',
        parameters=[LaunchConfiguration('param')]
    )

    launch_description.add_action(hokuyo_node)

    return launch_description
