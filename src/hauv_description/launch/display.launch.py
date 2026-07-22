"""Show the HAUV model in RViz2.

    # live - mirrors the real vehicle: thrusters follow /motor_data, hull pose
    #        follows the real BNO055 attitude and BAR100 depth
    DISPLAY=:0 ros2 launch hauv_description display.launch.py

    # offline - no vehicle needed, model just sits there
    DISPLAY=:0 ros2 launch hauv_description display.launch.py live:=false

Over SSH there is no DISPLAY, so prefix with the UP Board's local screen
(DISPLAY=:0) or RViz will abort with "cannot connect to X server".

Notes
-----
motor_to_joint_states runs in BOTH modes. It publishes /joint_states at 30 Hz
whether or not /motor_data is arriving, so the six continuous thruster joints
always have a transform. That is deliberate: joint_state_publisher is not
installed on this box, and this node covers the same need.

The RViz fixed frame is 'world'. Live, attitude_to_tf moves base_link inside
it; offline, a static identity transform stands in so the fixed frame always
exists and RViz never shows a TF error.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    share = get_package_share_directory('hauv_description')
    # 'lite' is the default: the detailed model is ~2.03M triangles, which is
    # hopeless on the UP Board's Atom - and worse over X forwarding, where all
    # that geometry is tunnelled to the PC's X server. The lite model is ~3.6k
    # triangles of URDF primitives with no mesh files at all, and is
    # dimensionally identical.
    model = LaunchConfiguration('model').perform(context)
    name = 'hauv_lite.urdf' if model == 'lite' else 'hauv.urdf'
    urdf = os.path.join(share, 'urdf', name)

    # Foxy's robot_state_publisher wants the URDF as a string parameter.
    with open(urdf, 'r') as f:
        robot_description = f.read()

    return _nodes(share, robot_description)


def generate_launch_description():
    live = LaunchConfiguration('live')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value='lite',
            description="'lite' (URDF primitives, ~3.6k tris - use on the UP "
                        "Board) or 'full' (vendor STLs, ~2.03M tris)."),
        DeclareLaunchArgument(
            'live', default_value='true',
            description='Drive the hull pose from the real IMU + depth sensor.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Also start RViz2.'),
        DeclareLaunchArgument(
            'zero_on_start', default_value='true',
            description='Treat the first pitch/roll reading as level.'),
        OpaqueFunction(function=_launch_setup),
    ])


def _nodes(share, robot_description):
    rviz_cfg = os.path.join(share, 'rviz', 'hauv.rviz')
    live = LaunchConfiguration('live')
    rviz = LaunchConfiguration('rviz')

    return [

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # Always: gives every thruster joint a state, live data or not.
        Node(
            package='hauv_description',
            executable='motor_to_joint_states',
            name='motor_to_joint_states',
            output='screen',
        ),

        # Live: hull pose follows the real IMU attitude and depth.
        Node(
            package='hauv_description',
            executable='attitude_to_tf',
            name='attitude_to_tf',
            output='screen',
            parameters=[{
                'zero_on_start': LaunchConfiguration('zero_on_start'),
            }],
            condition=IfCondition(live),
        ),

        # Offline: stand-in so 'world' exists and RViz has a fixed frame.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
            condition=UnlessCondition(live),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg],
            condition=IfCondition(rviz),
        ),
    ]
