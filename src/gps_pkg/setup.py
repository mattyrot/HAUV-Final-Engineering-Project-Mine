from setuptools import setup

package_name = 'gps_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='up',
    maintainer_email='bonnyrot@gmail.com',
    description='u-blox NEO-M8N GPS node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_node = gps_pkg.gps_node:main',
        ],
    },
)
