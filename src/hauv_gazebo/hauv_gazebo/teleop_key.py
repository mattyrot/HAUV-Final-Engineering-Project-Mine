#!/usr/bin/env python3
"""Fly the simulated HAUV from the keyboard.

WSL has no /dev/input - no USB passthrough, no loadable modules, so `joy_node`
can never see a real controller without rebuilding the WSL kernel. The keyboard
sidesteps that completely: it arrives on stdin, which WSL has always had. This
node reads keys and publishes `/joy` in exactly the shape joy_node would, so
`guidance_node` cannot tell the difference and flies the sim unmodified.

    ros2 run hauv_gazebo teleop_key

    arrows      up/down = forward/reverse, left/right = turn
    w / s       ascend / descend
    a / d       strafe left / right
    space       all stop
    + / -       speed scale
    q           quit (motors released to neutral)

Controls are STICKY, like a throttle rather than a gas pedal: a key sets the
axis and it holds until you change it or press space. A terminal reports key
presses but never key releases, so "hold to drive" would depend on the
auto-repeat delay and feel like a stutter. Sticky suits an ROV anyway - it takes
several seconds to accelerate, so you set a thrust and watch it develop.

Axis assignments match guidance_node's joystick map, and the two trigger axes
rest at +1.0 exactly as a real controller does - guidance reads axes[2]/axes[5]
!= 1.0 as a camera-tilt command, so publishing 0.0 there would drive the servo
continuously.

Publishing NEVER STOPS while this node runs, even with no keys pressed. That is
deliberate: guidance_node's JOY_FAILSAFE_TIMEOUT zeroes thrust 1.5 s after the
last /joy message, so a teleop that only published on keypress would have the
vehicle cut out between presses.

If stdin is not a terminal this falls back to reading piped bytes, which makes
the node scriptable for tests as well as usable by hand.
"""

import math
import os
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from sensor_msgs.msg import Joy

PUBLISH_HZ = 20.0

# guidance_node's joystick axis map.
AX_STRAFE, AX_FWD, AX_YAW, AX_VERT = 0, 1, 3, 4
# Triggers rest RELEASED at +1.0 on a real controller - see the module docstring.
NEUTRAL_AXES = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]

# Escape sequences the arrow keys actually send.
ARROWS = {'A': 'up', 'B': 'down', 'C': 'right', 'D': 'left'}

HELP = """
  HAUV keyboard teleop
  --------------------------------------------------
   arrow up/down     forward / reverse
   arrow left/right  turn left / right
   w / s             ascend / descend
   a / d             strafe left / right
   space             ALL STOP
   + / -             speed scale
   q or Ctrl-C       quit
  --------------------------------------------------
"""


class TeleopKey(Node):

    def __init__(self):
        super().__init__('teleop_key')
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        self.create_subscription(Vector3, '/esp32/bar100_data', self.on_bar, 10)
        self.create_subscription(Twist, '/esp32/bno055_data', self.on_imu, 10)

        self.axes = list(NEUTRAL_AXES)
        self.scale = 0.6          # start gentle; full stick is a lot of thrust
        self.depth = float('nan')
        self.heading = float('nan')
        self.done = False
        self._pending = ''        # partial escape sequence

        self.create_timer(1.0 / PUBLISH_HZ, self.tick)

    def on_bar(self, msg):
        self.depth = msg.x

    def on_imu(self, msg):
        self.heading = msg.linear.x

    # ── keys ─────────────────────────────────────────────────────────────────

    def apply(self, key):
        """Map one keystroke onto the sticky axis state. Returns False to quit."""
        s = self.scale
        if key == 'up':
            self.axes[AX_FWD] = +s
        elif key == 'down':
            self.axes[AX_FWD] = -s
        elif key == 'left':
            self.axes[AX_YAW] = +s
        elif key == 'right':
            self.axes[AX_YAW] = -s
        elif key in ('w', 'W'):
            self.axes[AX_VERT] = +s
        elif key in ('s', 'S'):
            self.axes[AX_VERT] = -s
        elif key in ('a', 'A'):
            self.axes[AX_STRAFE] = +s
        elif key in ('d', 'D'):
            self.axes[AX_STRAFE] = -s
        elif key == ' ':
            self.axes = list(NEUTRAL_AXES)
        elif key in ('+', '='):
            self.scale = min(1.0, round(self.scale + 0.1, 2))
            self._rescale()
        elif key in ('-', '_'):
            self.scale = max(0.1, round(self.scale - 0.1, 2))
            self._rescale()
        elif key in ('q', 'Q', '\x03'):
            return False
        return True

    def _rescale(self):
        """Keep whatever is currently commanded, at the new scale."""
        for i in (AX_STRAFE, AX_FWD, AX_YAW, AX_VERT):
            if self.axes[i]:
                self.axes[i] = math.copysign(self.scale, self.axes[i])

    def feed(self, data):
        """Decode a chunk of stdin, handling multi-byte arrow escapes."""
        self._pending += data
        while self._pending:
            if self._pending[0] == '\x1b':
                # Need the full "\x1b[X"; wait for more bytes if it is split.
                if len(self._pending) < 3:
                    if len(self._pending) == 1 and len(data) == 1:
                        return          # lone ESC, wait
                    return
                seq, self._pending = self._pending[:3], self._pending[3:]
                key = ARROWS.get(seq[2])
                if key and not self.apply(key):
                    self.done = True
                    return
            else:
                ch, self._pending = self._pending[0], self._pending[1:]
                if ch in '\r\n':
                    continue
                if not self.apply(ch):
                    self.done = True
                    return

    # ── output ───────────────────────────────────────────────────────────────

    def tick(self):
        j = Joy()
        j.header.stamp = self.get_clock().now().to_msg()
        j.axes = [float(v) for v in self.axes]
        j.buttons = [0] * 16      # never toggle guidance's mode/lights by accident
        self.joy_pub.publish(j)

    def hud(self):
        def arrow(v, pos, neg):
            return pos if v > 0.01 else (neg if v < -0.01 else '·')
        return (f'\r  fwd {arrow(self.axes[AX_FWD], "^", "v")} '
                f'turn {arrow(self.axes[AX_YAW], "<", ">")} '
                f'vert {arrow(self.axes[AX_VERT], "^", "v")} '
                f'strafe {arrow(self.axes[AX_STRAFE], "<", ">")} '
                f'| scale {self.scale:.1f} '
                f'| depth {self.depth:6.2f} m  hdg {self.heading:5.1f}   ')


def main():
    rclpy.init()
    node = TeleopKey()

    interactive = sys.stdin.isatty()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd) if interactive else None
    if interactive:
        print(HELP)
        tty.setraw(fd)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
            r, _, _ = select.select([sys.stdin], [], [], 0)
            if r:
                data = os.read(fd, 64).decode('utf-8', 'ignore')
                if not data:            # EOF on a pipe
                    break
                node.feed(data)
            if interactive:
                sys.stdout.write(node.hud())
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        if interactive:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            print('\n  stopping - motors to neutral')
        # Leave the vehicle stopped rather than coasting on the last command.
        node.axes = list(NEUTRAL_AXES)
        for _ in range(10):
            node.tick()
            rclpy.spin_once(node, timeout_sec=0.01)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
