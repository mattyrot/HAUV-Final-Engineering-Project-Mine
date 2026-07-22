#!/usr/bin/env python3
"""Feed Gazebo's simulated sensors back to guidance as if they were the ESP32.

Together with thruster_bridge this closes the loop: guidance_node subscribes to
the same /esp32/* topics it uses on the real vehicle, so every control feature -
depth hold, auto-GOTO, the leak failsafe, the safety envelope - can be exercised
dry, without water and without risking hardware.

    /hauv/imu          (sensor_msgs/Imu)   -> /esp32/bno055_data   (Twist, degrees)
    /gazebo/model_states                   -> /esp32/bar100_data   (Vector3, depth m)
                                           -> /gps/fix             (NavSatFix)
    (synthesised)                          -> /esp32/leak          (Float64, 0.0)

Conventions match the real firmware exactly, because guidance parses them
positionally:
    bno055_data : linear.x = yaw/heading, y = pitch, z = roll   [degrees]
    bar100_data : x = depth (m, positive DOWN), y = pressure, z = temperature
    leak        : 0.0 dry, 1.0 leak

Depth comes from the model's Z in Gazebo, negated: the world has Z up, the
vehicle spawns at the surface, so diving gives a positive depth. That matches
the BAR100 and keeps the depth-hold sign conventions honest.

Set `publish_leak:=false` to stop synthesising a dry leak reading - useful if
you want to inject a leak by hand and watch the auto-surface failsafe fire.
"""

import math

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix

MODEL_NAME = 'hauv'

# Where the simulated vehicle "is" in the world, so /gps/fix is plausible and
# auto-GOTO has something to navigate against. Roughly the test site.
ORIGIN_LAT = 31.2651596
ORIGIN_LON = 34.8037453
M_PER_DEG_LAT = 111320.0


def quat_to_euler(x, y, z, w):
    """Quaternion -> (roll, pitch, yaw) in radians, Z-Y-X convention."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class SensorBridge(Node):

    def __init__(self):
        super().__init__('sensor_bridge')

        self.declare_parameter('publish_leak', True)
        self.publish_leak = bool(self.get_parameter('publish_leak').value)

        self.imu_pub = self.create_publisher(Twist, '/esp32/bno055_data', 10)
        self.bar_pub = self.create_publisher(Vector3, '/esp32/bar100_data', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        if self.publish_leak:
            from std_msgs.msg import Float64
            self._Float64 = Float64
            self.leak_pub = self.create_publisher(Float64, '/esp32/leak', 10)

        self.create_subscription(Imu, '/hauv/imu', self.on_imu, 10)
        self.create_subscription(ModelStates, '/gazebo/model_states',
                                 self.on_states, 10)

        self.depth = 0.0
        self.x = 0.0
        self.y = 0.0
        self.have_pose = False
        self.create_timer(0.05, self.tick)     # 20 Hz, like the real ESP32

        self.get_logger().info(
            'bridging Gazebo -> /esp32/bno055_data, /esp32/bar100_data, /gps/fix'
            + (', /esp32/leak' if self.publish_leak else ' (leak NOT synthesised)'))

    def on_imu(self, msg):
        q = msg.orientation
        roll, pitch, yaw = quat_to_euler(q.x, q.y, q.z, q.w)
        out = Twist()
        # Firmware reports degrees, and yaw as a 0-360 heading.
        out.linear.x = math.degrees(yaw) % 360.0
        out.linear.y = math.degrees(pitch)
        out.linear.z = math.degrees(roll)
        self.imu_pub.publish(out)

    def on_states(self, msg):
        try:
            i = msg.name.index(MODEL_NAME)
        except ValueError:
            return
        p = msg.pose[i].position
        self.x, self.y = p.x, p.y
        self.depth = -p.z        # world Z is up; depth is positive down
        self.have_pose = True

    def tick(self):
        bar = Vector3()
        bar.x = float(self.depth)
        # Plausible stand-ins so anything reading these fields sees sane numbers.
        bar.y = 1013.25 + max(0.0, self.depth) * 100.65   # mbar
        bar.z = 18.0                                       # degC
        self.bar_pub.publish(bar)

        if self.publish_leak:
            m = self._Float64()
            m.data = 0.0
            self.leak_pub.publish(m)

        if self.have_pose:
            fix = NavSatFix()
            fix.header.stamp = self.get_clock().now().to_msg()
            fix.header.frame_id = 'gps'
            fix.status.status = 0        # STATUS_FIX
            fix.status.service = 1       # SERVICE_GPS
            fix.latitude = ORIGIN_LAT + (self.x / M_PER_DEG_LAT)
            lon_scale = M_PER_DEG_LAT * math.cos(math.radians(ORIGIN_LAT))
            fix.longitude = ORIGIN_LON + (self.y / lon_scale)
            fix.altitude = -self.depth
            self.gps_pub.publish(fix)


def main():
    rclpy.init()
    node = SensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
