from setuptools import setup

package_name = 'solver_velocity'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atwork',
    maintainer_email='atwork@todo.todo',
    description='Nodo que calcula velocidades lineales y angulares a partir de los encoders del MD49',
    license='MIT',
    entry_points={
        'console_scripts': [
            'solver_velocity_node = solver_velocity.solver_velocity_node:main',
            'rotate_360.py = solver_localization.scripts.rotate_360:main',
        ],
    },
)

