from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    nav2_bringup_dir = '/opt/ros/humble/share/nav2_bringup/launch'
    map_file = '/home/atwork/mapa_solver.yaml'
    params_file = '/home/atwork/solverRos2_ws/src/solver_navigation/config/nav2_params.yaml'

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'false',
                'params_file': params_file
            }.items(),
        )
    ])
