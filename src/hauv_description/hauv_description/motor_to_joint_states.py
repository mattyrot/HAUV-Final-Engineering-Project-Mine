#!/usr/bin/env python3
"""
Bridge /motor_data (PWM) -> /joint_states so RViz shows the thrusters turning.

This is VISUALISATION ONLY. RViz has no physics: this spins the propeller
joints to match what guidance_node is commanding, it does not simulate thrust,
buoyancy or vehicle motion. For that you need Gazebo.

    ros2 run robot_state_publisher robot_state_publisher hauv.urdf &
    python3 motor_to_joint_states.py &
    rviz2

Motor -> link mapping follows the convention in guidance_node.py. VERIFY IT:
CLAUDE.md documents angular.z as both motor 6 and the camera servo, so the
sixth assignment below is an inference, not a checked fact.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

PWM_NEUTRAL = 1500.0
PWM_SPAN = 400.0          # 1500 -> 1900 is full forward
MAX_RAD_S = 40.0          # visual only; ~380 rpm reads about right on screen
DEADBAND = 8.0            # ignore PWM jitter around neutral

# (Twist field, joint name). Order matches motors 1..6.
MAP = [
    ('linear.x',  't1_front_left_joint'),
    ('linear.y',  't5_vert_front_joint'),
    ('linear.z',  't2_front_right_joint'),
    ('angular.x', 't3_rear_left_joint'),
    ('angular.y', 't6_vert_rear_joint'),
    ('angular.z', 't4_rear_right_joint'),
]

# CCW thrusters spin the other way for the same commanded thrust.
SPIN_SIGN = {
    't1_front_left_joint': +1.0,
    't2_front_right_joint': -1.0,
    't3_rear_left_joint': -1.0,
    't4_rear_right_joint': +1.0,
    't5_vert_front_joint': +1.0,
    't6_vert_rear_joint': -1.0,
}


def field(msg, path):
    part, axis = path.split('.')
    return getattr(getattr(msg, part), axis)


class MotorToJointStates(Node):

    def __init__(self):
        super().__init__('motor_to_joint_states')
        self.names = [j for _, j in MAP]
        self.angle = {j: 0.0 for j in self.names}
        self.vel = {j: 0.0 for j in self.names}
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_subscription(Twist, '/motor_data', self.on_motor, 10)
        self.dt = 1.0 / 30.0
        self.create_timer(self.dt, self.tick)
        self.get_logger().info('bridging /motor_data -> /joint_states (visual only)')

    def on_motor(self, msg):
        for path, joint in MAP:
            pwm = float(field(msg, path))
            if pwm <= 0.0:                     # topic not populated yet
                continue
            d = pwm - PWM_NEUTRAL
            if abs(d) < DEADBAND:
                d = 0.0
            frac = max(-1.0, min(1.0, d / PWM_SPAN))
            self.vel[joint] = frac * MAX_RAD_S * SPIN_SIGN[joint]

    def tick(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.names
        for j in self.names:
            self.angle[j] = (self.angle[j] + self.vel[j] * self.dt) % (2.0 * math.pi)
        js.position = [self.angle[j] for j in self.names]
        js.velocity = [self.vel[j] for j in self.names]
        self.pub.publish(js)


def main():
    rclpy.init()
    node = MotorToJointStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
