from setuptools import setup
import os
from glob import glob

package_name = 'solver_md49'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 👇 agrega esta línea para incluir los launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atwork',
    maintainer_email='example@example.com',
    description='MD49 motor controller node for Solver robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'md49_node = solver_md49.md49_node:main',
        ],
    },
)

