import os
from glob import glob
from setuptools import setup

package_name = 'solver_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Archivos básicos de ROS2
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Archivos de configuración (.yaml)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # Archivos de lanzamiento (.launch.py)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atwork',
    maintainer_email='atwork@example.com',
    description='Paquete de localización con EKF para robot tipo skid-steer.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Ejecución del broadcaster TF de IMU
            'imu_tf_broadcaster = solver_localization.imu_tf_broadcaster:main',
            'debug_ekf_input_node = solver_localization.debug_ekf_input_node:main',
        ],
    },
)
