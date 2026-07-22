#!/usr/bin/env python3
"""Drive the RViz model from the live vehicle: world -> base_link TF.

Takes the real BNO055 attitude and BAR100 depth and publishes them as a
transform, so the model in RViz rolls, pitches, turns and sinks exactly as the
vehicle does. Combined with motor_to_joint_states (which spins the thrusters
from /motor_data) this gives a live mirror of the vehicle.

VISUALISATION ONLY. Nothing here simulates physics - it displays measured state.

Inputs
    /esp32/bno055_data   geometry_msgs/Twist    linear.x/y/z = yaw/pitch/roll, degrees
    /esp32/bar100_data   geometry_msgs/Vector3  x = depth in metres, positive down

Output
    TF world -> base_link

Parameters
    zero_on_start (bool, default True)
        Capture the first pitch/roll reading as level. The vehicle is rarely
        sitting perfectly flat on the bench, and without this the model looks
        permanently tilted. Yaw is never zeroed - it is a real heading.
    invert_depth (bool, default False)
        Flip the sign if the model climbs when the vehicle dives. Depth is
        positive-down, world Z is up, so z = -depth is the expected mapping.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist, Vector3
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def quat_from_rpy(roll, pitch, yaw):
    """RPY (radians) -> (x, y, z, w). Standard Z-Y-X intrinsic convention."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,   # x
        cr * sp * cy + sr * cp * sy,   # y
        cr * cp * sy - sr * sp * cy,   # z
        cr * cp * cy + sr * sp * sy,   # w
    )


class AttitudeToTF(Node):

    def __init__(self):
        super().__init__('attitude_to_tf')

        self.declare_parameter('zero_on_start', True)
        self.declare_parameter('invert_depth', False)
        self.zero_on_start = self.get_parameter('zero_on_start').value
        self.invert_depth = self.get_parameter('invert_depth').value

        self.yaw = 0.0      # degrees
        self.pitch = 0.0
        self.roll = 0.0
        self.depth = 0.0    # metres, positive down

        self.pitch_offset = None   # set on the first reading if zeroing
        self.roll_offset = None

        self.br = TransformBroadcaster(self)
        self.create_subscription(Twist, '/esp32/bno055_data', self.on_imu, 10)
        self.create_subscription(Vector3, '/esp32/bar100_data', self.on_depth, 10)
        self.create_timer(1.0 / 30.0, self.tick)

        self.get_logger().info(
            'publishing world -> base_link from live IMU + depth '
            f'(zero_on_start={self.zero_on_start})')

    def on_imu(self, msg):
        self.yaw = msg.linear.x
        self.pitch = msg.linear.y
        self.roll = msg.linear.z
        if self.zero_on_start and self.pitch_offset is None:
            self.pitch_offset = self.pitch
            self.roll_offset = self.roll
            self.get_logger().info(
                f'levelled: pitch offset {self.pitch_offset:.1f} deg, '
                f'roll offset {self.roll_offset:.1f} deg')

    def on_depth(self, msg):
        self.depth = msg.x

    def tick(self):
        pitch = self.pitch - (self.pitch_offset or 0.0)
        roll = self.roll - (self.roll_offset or 0.0)

        z = self.depth if self.invert_depth else -self.depth

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = float(z)

        qx, qy, qz, qw = quat_from_rpy(
            math.radians(roll), math.radians(pitch), math.radians(self.yaw))
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = AttitudeToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
