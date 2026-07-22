#!/usr/bin/env python3
"""Drive the simulated thrusters from the real /motor_data topic.

This is what lets `guidance_node` fly the simulation completely unmodified:
it publishes the same PWM values it would send to the ESP32, and this node
turns them into forces on the six thruster links in Gazebo.

    /motor_data (Twist, PWM 1100-1900)  ->  /hauv/thruster/tN (Wrench, newtons)

Motor -> link mapping follows the convention already used by
motor_to_joint_states.py in hauv_description, so the same six thrusters move in
Gazebo and in RViz.

Thrust model
------------
A T200 at 12 V produces roughly 35 N forward at full throttle and about 80 % of
that in reverse - propellers are less efficient backwards. That asymmetry is
modelled because it visibly affects a real ROV: reversing is weaker than
driving forward. The curve is otherwise linear in PWM, which is a simplification
(real thrust is closer to quadratic in RPM) but good enough to fly the control
loops against.
"""

import rclpy
from geometry_msgs.msg import Twist, Wrench
from rclpy.node import Node

PWM_NEUTRAL = 1500.0
PWM_SPAN = 400.0        # 1500 -> 1900 is full forward
DEADBAND = 8.0          # ignore PWM jitter around neutral
MAX_THRUST_FWD = 35.0   # newtons, T200 @ 12 V
MAX_THRUST_REV = 28.0   # newtons, ~80 % of forward
PUBLISH_HZ = 200.0      # see the note in __init__ - the force plugin is rate sensitive

# (Twist field, thruster topic suffix), motors 1..6.
#
# DERIVED FROM PHYSICS, not copied. hauv_description/motor_to_joint_states.py
# has motors 4 and 6 the other way round, and its README says outright that the
# mapping was never verified. It is wrong, and it matters here because Gazebo
# actually integrates the forces.
#
# Each thruster's contribution per unit thrust, from its position and its +Z
# direction (fwd, strafe, yaw-moment):
#     t1 (+0.707, +0.707, +0.110)      t3 (-0.707, +0.707, -0.110)
#     t2 (+0.707, -0.707, -0.110)      t4 (-0.707, -0.707, +0.110)
# guidance_node's mixing coefficients (fwd, strafe, yaw):
#     m1 (+,+,+)   m3 (+,-,-)   m6 (-,+,-)   m4 (-,-,+)
# Matching sign patterns gives exactly one assignment: m1->t1, m3->t2,
# m6->t3, m4->t4.
#
# With motors 4 and 6 swapped, forward still worked but yaw commands came out
# as pure strafe - the four yaw moments cancelled to zero.
MAP = [
    ('linear.x',  't1'),   # motor 1 -> t1_front_left
    ('linear.y',  't5'),   # motor 2 -> t5_vert_front   (vertical pair)
    ('linear.z',  't2'),   # motor 3 -> t2_front_right
    ('angular.x', 't4'),   # motor 4 -> t4_rear_right
    ('angular.y', 't6'),   # motor 5 -> t6_vert_rear    (vertical pair)
    ('angular.z', 't3'),   # motor 6 -> t3_rear_left
]


def field(msg, path):
    part, axis = path.split('.')
    return getattr(getattr(msg, part), axis)


class ThrusterBridge(Node):

    def __init__(self):
        super().__init__('thruster_bridge')
        self.pubs = {
            name: self.create_publisher(Wrench, f'/hauv/thruster/{name}', 10)
            for _, name in MAP
        }
        self.force = {name: 0.0 for _, name in MAP}
        self.create_subscription(Twist, '/motor_data', self.on_motor, 10)
        # Republish fast. gazebo_ros_force turned out to be RATE SENSITIVE: the
        # same 35 N moved the vehicle 5x further when published at 500 Hz than
        # at 20 Hz, because physics runs at 1000 Hz and steps between messages
        # get little or no force. 200 Hz is a compromise - most of the benefit
        # without spending a core on publishing. If thrust still feels weak,
        # raise PUBLISH_HZ before touching the thrust constants.
        self.create_timer(1.0 / PUBLISH_HZ, self.tick)
        self.get_logger().info(
            'bridging /motor_data -> /hauv/thruster/t1..t6 '
            f'(max {MAX_THRUST_FWD:.0f} N fwd / {MAX_THRUST_REV:.0f} N rev)')

    def on_motor(self, msg):
        for path, name in MAP:
            pwm = float(field(msg, path))
            if pwm <= 0.0:            # topic not populated yet
                continue
            d = pwm - PWM_NEUTRAL
            if abs(d) < DEADBAND:
                d = 0.0
            frac = max(-1.0, min(1.0, d / PWM_SPAN))
            self.force[name] = frac * (MAX_THRUST_FWD if frac >= 0
                                       else MAX_THRUST_REV)

    def tick(self):
        for _, name in MAP:
            w = Wrench()
            # Thrust acts along the link's local +Z, NOT +X.
            #
            # Every thruster joint declares axis="0 0 1": the propeller spins
            # about local Z, so that is the thrust axis. Working the joint rpy
            # through, local +Z gives (+-0.70, +-0.71, 0) for t1-t4 - the four
            # 45-degree vectored horizontals - and (0, 0, 1) for t5/t6, the
            # vertical pair. Local +X points somewhere useless by comparison.
            #
            # Using force.x looked plausible (the RViz README calls +X the
            # thrust axis, but that refers to the spin axis of the RViz model)
            # and was quietly wrong: it shoved t5/t6 forwards at z = -0.006,
            # below the centre of mass at +0.012, so "full vertical thrust"
            # produced a pitching couple - -16 deg one way, +19 deg the other -
            # and no vertical motion at all.
            w.force.z = self.force[name]
            self.pubs[name].publish(w)


def main():
    rclpy.init()
    node = ThrusterBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
