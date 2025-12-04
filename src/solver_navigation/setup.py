from setuptools import setup
from glob import glob
import os

package_name = 'solver_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'bt_trees'), glob('config/bt_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atwork',
    maintainer_email='atwork@solver.local',
    description='Navigation package with Nav2 bringup',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
