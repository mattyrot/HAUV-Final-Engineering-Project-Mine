import os
from glob import glob

from setuptools import setup

package_name = 'hauv_description'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        # package://hauv_description/meshes/... resolves to share/<pkg>/meshes/,
        # so the URDF's mesh paths only work if these land here.
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HAUV Team',
    maintainer_email='bonnyrot@gmail.com',
    description='URDF model and RViz2 visualisation for the HAUV.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motor_to_joint_states = hauv_description.motor_to_joint_states:main',
            'attitude_to_tf = hauv_description.attitude_to_tf:main',
        ],
    },
)
