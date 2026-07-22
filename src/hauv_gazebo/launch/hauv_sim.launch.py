"""Gazebo simulation of the HAUV.

    # full sim with GUI (needs an X server - see the package README)
    ros2 launch hauv_gazebo hauv_sim.launch.py

    # headless (no GUI) - much lighter, good for testing control loops
    ros2 launch hauv_gazebo hauv_sim.launch.py gui:=false

    # sim only, without the guidance bridges (drive the thrusters by hand)
    ros2 launch hauv_gazebo hauv_sim.launch.py bridges:=false

With the bridges running, `guidance_node` flies the simulation unmodified:
it reads /esp32/* and /gps/fix (synthesised from Gazebo) and writes /motor_data
(turned into thruster forces). Start guidance separately, exactly as on the
vehicle.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('hauv_gazebo')
    urdf = os.path.join(share, 'urdf', 'hauv_gazebo.urdf')
    world = os.path.join(share, 'worlds', 'underwater.world')

    with open(urdf, 'r') as f:
        robot_description = f.read()

    gui = LaunchConfiguration('gui')
    bridges = LaunchConfiguration('bridges')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Run the Gazebo client GUI.'),
        DeclareLaunchArgument('bridges', default_value='true',
                              description='Run the thruster/sensor bridges so '
                                          'guidance_node can fly the sim.'),
        DeclareLaunchArgument('spawn_z', default_value='-0.15',
                              description='Spawn depth (world Z, negative = under water).'),
        # sensor_bridge synthesises a dry leak reading at 20 Hz. To test the
        # auto-surface failsafe you must turn that off and own the topic:
        # guidance_node needs LEAK_TRIGGER_N *consecutive* wet samples, and an
        # interleaved dry stream keeps resetting the count.
        DeclareLaunchArgument('publish_leak', default_value='true',
                              description='Let sensor_bridge synthesise /esp32/leak=0.0. '
                                          'Set false to inject a leak by hand.'),

        # gzserver always; gzclient only when gui:=true. Kept as separate
        # processes so headless costs nothing.
        ExecuteProcess(
            cmd=['gzserver', '--verbose', world,
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen',
            condition=IfCondition(gui),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_hauv',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'hauv',
                '-x', '0', '-y', '0',
                '-z', LaunchConfiguration('spawn_z'),
            ],
        ),

        # /motor_data -> thruster forces
        Node(
            package='hauv_gazebo',
            executable='thruster_bridge',
            name='thruster_bridge',
            output='screen',
            condition=IfCondition(bridges),
        ),

        # Gazebo IMU/pose -> /esp32/* and /gps/fix
        Node(
            package='hauv_gazebo',
            executable='sensor_bridge',
            name='sensor_bridge',
            output='screen',
            # value_type=bool is required: sensor_bridge declares publish_leak as
            # a bool, but a LaunchConfiguration substitutes a *string*, and Foxy
            # rejects the type mismatch at startup.
            parameters=[{'publish_leak': ParameterValue(
                LaunchConfiguration('publish_leak'), value_type=bool)}],
            condition=IfCondition(bridges),
        ),
    ])
