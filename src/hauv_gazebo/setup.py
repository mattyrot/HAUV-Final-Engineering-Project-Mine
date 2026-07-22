import os
from glob import glob

from setuptools import setup

package_name = 'hauv_gazebo'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HAUV Team',
    maintainer_email='bonnyrot@gmail.com',
    description='Gazebo simulation of the HAUV.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'thruster_bridge = hauv_gazebo.thruster_bridge:main',
            'sensor_bridge = hauv_gazebo.sensor_bridge:main',
            'sim_pilot = hauv_gazebo.sim_pilot:main',
            'teleop_key = hauv_gazebo.teleop_key:main',
        ],
    },
)
