from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
                'subscribe_scan': False,
                'approx_sync': True,
                'queue_size': 30,
                'RGBD/NeighborLinkRefining': 'true',
                'Reg/Strategy': '1',  # Visual + ICP
                'Grid/RangeMax': '6.0',
                'Vis/MinInliers': '10',
                'Vis/EstimationType': '1',
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
            ],
        ),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'map_frame_id': 'map'
            }],
        )
    ])
