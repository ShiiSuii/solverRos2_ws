from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'solver_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    include_package_data=True,

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'bt_trees'),
            glob('config/bt_trees/*.xml')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atwork',
    maintainer_email='atwork@example.com',
    description='Navigation package',
    license='MIT',

    entry_points={
        'console_scripts': [
            'initial_pose_publisher = solver_navigation.initial_pose_publisher:main',
            'map_save_on_button = solver_navigation.map_save_on_button:main',
        ],
    },
)
