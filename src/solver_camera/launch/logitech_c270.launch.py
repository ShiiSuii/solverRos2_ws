from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='logitech_c270',
            namespace='camera',
            parameters=[{
                'video_device': '/dev/video0',
                'pixel_format': 'YUYV',
                'image_size': [640, 480],
                'camera_frame_id': 'logitech_frame'
            }],
            output='screen'
        ),
    ])
