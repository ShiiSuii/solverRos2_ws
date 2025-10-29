from setuptools import setup

package_name = 'solver_odometry'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/hardware.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atwork',
    maintainer_email='atwork@example.com',
    description='Compute odometry (x,y,theta) from MD49 encoders.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'solver_odometry = solver_odometry.odometry_node:main',
        ],
    },
)
    