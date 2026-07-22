#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, String, UInt16
import collections
import glob
import subprocess
import time
import math

# ---------------------------------------------------------------------------
# Sensor descriptor
# ---------------------------------------------------------------------------

FREQ_WINDOW     = 5.0    # seconds of history used to estimate actual Hz
FREQ_THRESHOLD  = 0.50   # alert if actual Hz < 50% of expected Hz

# Bit order for the /health/status bitmask (bit 0 = first). A bit is 1 when that
# sensor is healthy (responding, in-range, at rate). Must match HEALTH_ORDER in
# acoustic_bridge_node.py.
HEALTH_ORDER = ['bno055', 'bar100', 'bme280', 'leak',
                'dvl', 'subsonus_vel', 'subsonus_ori', 'gps']


class SensorWatch:
    def __init__(self, name, timeout, expected_hz=0.0):
        self.name = name
        self.timeout = timeout
        self.expected_hz = expected_hz   # 0 = no frequency check
        self.last_time = None
        self.timeout_alerted = False
        self.limit_alerted = {}          # field -> bool
        self.bad_count = {}              # field -> consecutive bad reading count
        self.freq_alerted = False
        self.msg_times = collections.deque()  # rolling timestamps


def _check_finite(value):
    return math.isfinite(value)


# ---------------------------------------------------------------------------
# Sensor limits  {field: (min, max, unit)}
# ---------------------------------------------------------------------------

LIMITS = {
    'bno055': {
        'heading': (0.0,   360.0,  'deg'),
        'pitch':   (-90.0,  90.0,  'deg'),
        'roll':    (-180.0, 180.0, 'deg'),
    },
    'bar100': {
        'depth':       (-1.0,   300.0,   'm'),
        'pressure':    (900.0,  40000.0, 'mbar'),
        'temperature': (-20.0,  85.0,    'C'),
    },
    'bme280': {
        'temperature': (-40.0, 85.0,   'C'),
        'pressure':    (300.0, 1100.0, 'hPa'),
        'humidity':    (0.0,   100.0,  '%'),
    },
    'dvl': {
        'vx': (-5.0, 5.0, 'm/s'),
        'vy': (-5.0, 5.0, 'm/s'),
        'vz': (-5.0, 5.0, 'm/s'),
    },
    'subsonus_vel': {
        'vx': (-5.0, 5.0, 'm/s'),
        'vy': (-5.0, 5.0, 'm/s'),
        'vz': (-5.0, 5.0, 'm/s'),
    },
    'subsonus_ori': {
        'heading': (0.0,   360.0,  'deg'),
        'pitch':   (-90.0,  90.0,  'deg'),
        'roll':    (-180.0, 180.0, 'deg'),
    },
}

# How long to wait at startup before alerting "no data ever received"
STARTUP_GRACE = 15.0

# Motor cross-check: if effort exceeds this fraction and no motion seen → alert
MOTOR_NEUTRAL     = 1500.0
MOTOR_MAX_DEV     = 400.0   # 1100–1900 µs range
MOTOR_EFFORT_THR  = 0.20    # 20% effort (~80 µs from neutral)
MOTION_SPEED_THR  = 0.04    # m/s  — below this counts as "not moving"
MOTION_HDG_RATE   = 2.0     # deg/s — below this counts as "not rotating"
MOTOR_STUCK_TIME  = 6.0     # seconds effort+no-motion must persist before alert

# Both ESP32 I2C sensors dead for this long → auto-reset ESP32
ESP32_RESTART_TIMEOUT = 20.0
# Minimum gap between auto-resets (avoid reset storms)
ESP32_RESET_COOLDOWN = 60.0


class HealthMonitorNode(Node):

    def __init__(self):
        super().__init__('health_monitor_node')

        self._start_time = time.time()
        self._both_dead_since = None
        self._last_esp32_reset = 0.0

        # Motor cross-check state
        self._motor_effort   = 0.0   # 0–1, max effort across all motors
        self._linear_speed   = 0.0   # m/s from best available velocity source
        self._heading        = None  # degrees, last known
        self._heading_time   = 0.0
        self._heading_rate   = 0.0   # deg/s
        self._stuck_since    = None
        self._stuck_alerted  = False

        self.sensors = {
            'bno055':       SensorWatch('BNO055 IMU',           timeout=2.0,  expected_hz=20.0),
            'bar100':       SensorWatch('BAR100 depth',         timeout=2.0,  expected_hz=20.0),
            'bme280':       SensorWatch('BME280 env',           timeout=2.0,  expected_hz=20.0),
            'leak':         SensorWatch('Leak sensor',          timeout=2.0,  expected_hz=20.0),
            'dvl':          SensorWatch('DVL',                  timeout=5.0,  expected_hz=10.0),
            # No frequency check on the USBL — it trickles at a variable ~0.25 Hz
            # (marginal packet-20 decode). The 60 s timeout covers total data loss.
            'subsonus_vel': SensorWatch('Subsonus velocity',    timeout=60.0, expected_hz=0.0),
            'subsonus_ori': SensorWatch('Subsonus orientation', timeout=60.0, expected_hz=0.0),
            'gps':          SensorWatch('GPS',                  timeout=10.0, expected_hz=1.0),
        }

        self.alert_pub = self.create_publisher(String, '/health/alert', 10)
        self.status_pub = self.create_publisher(UInt16, '/health/status', 10)

        self.create_subscription(Twist,     '/esp32/bno055_data',    self._bno_cb,    10)
        self.create_subscription(Vector3,   '/esp32/bar100_data',    self._bar_cb,    10)
        self.create_subscription(Vector3,   '/esp32/bme280_data',    self._bme_cb,    10)
        self.create_subscription(Float64,   '/esp32/leak',           self._leak_cb,   10)
        self.create_subscription(Twist,     '/dvl/velocity_data',    self._dvl_cb,    10)
        self.create_subscription(Twist,     '/subsonus/velocity',    self._sv_cb,     10)
        self.create_subscription(Twist,     '/subsonus/orientation', self._so_cb,     10)
        self.create_subscription(NavSatFix, '/gps/fix',              self._gps_cb,    10)
        self.create_subscription(Twist,     '/motor_data',           self._motor_cb,  10)

        self.create_timer(1.0, self._check_timeouts)
        self.get_logger().info('Health monitor active')

    # -----------------------------------------------------------------------
    # Alert helpers
    # -----------------------------------------------------------------------

    def _alert(self, text, severity='warn'):
        if severity == 'error':
            self.get_logger().error(text)
        else:
            self.get_logger().warn(text)
        msg = String()
        msg.data = text
        self.alert_pub.publish(msg)

    def _clear(self, text):
        self.get_logger().info(text)
        msg = String()
        msg.data = text
        self.alert_pub.publish(msg)

    CONSECUTIVE_BAD_THRESHOLD = 3  # ignore transient single bad readings

    def _check_limit(self, key, field, value):
        sw = self.sensors[key]
        lo, hi, unit = LIMITS[key][field]
        bad = not _check_finite(value) or value < lo or value > hi
        if bad:
            sw.bad_count[field] = sw.bad_count.get(field, 0) + 1
            if sw.bad_count[field] >= self.CONSECUTIVE_BAD_THRESHOLD:
                if not sw.limit_alerted.get(field):
                    self._alert(
                        f'{sw.name} {field} out of range: '
                        f'{"nan" if not math.isfinite(value) else f"{value:.2f}"} {unit} '
                        f'(limit {lo}–{hi})',
                        'error'
                    )
                    sw.limit_alerted[field] = True
        else:
            sw.bad_count[field] = 0
            if sw.limit_alerted.get(field):
                self._clear(f'{sw.name} {field} recovered: {value:.2f} {unit}')
                sw.limit_alerted[field] = False

    def _touch(self, key):
        now = time.time()
        sw = self.sensors[key]
        sw.last_time = now
        if sw.expected_hz > 0:
            sw.msg_times.append(now)

    # -----------------------------------------------------------------------
    # Subscribers
    # -----------------------------------------------------------------------

    def _bno_cb(self, msg):
        self._touch('bno055')
        self._check_limit('bno055', 'heading', msg.linear.x)
        self._check_limit('bno055', 'pitch',   msg.linear.y)
        self._check_limit('bno055', 'roll',    msg.linear.z)
        # Heading rate for motor cross-check
        now = time.time()
        hdg = msg.linear.x
        if self._heading is not None and (now - self._heading_time) > 0:
            dt = now - self._heading_time
            delta = abs((hdg - self._heading + 180) % 360 - 180)
            self._heading_rate = delta / dt
        self._heading = hdg
        self._heading_time = now

    def _bar_cb(self, msg):
        self._touch('bar100')
        self._check_limit('bar100', 'depth',       msg.x)
        self._check_limit('bar100', 'pressure',    msg.y)
        self._check_limit('bar100', 'temperature', msg.z)

    def _bme_cb(self, msg):
        self._touch('bme280')
        self._check_limit('bme280', 'temperature', msg.x)
        self._check_limit('bme280', 'pressure',    msg.y)
        self._check_limit('bme280', 'humidity',    msg.z)

    def _leak_cb(self, msg):
        self._touch('leak')
        sw = self.sensors['leak']
        if msg.data == 1.0:
            if not sw.limit_alerted.get('leak'):
                self._alert('LEAK DETECTED!', 'error')
                sw.limit_alerted['leak'] = True
        else:
            if sw.limit_alerted.get('leak'):
                self._clear('Leak sensor clear')
                sw.limit_alerted['leak'] = False

    def _dvl_cb(self, msg):
        self._touch('dvl')
        self._check_limit('dvl', 'vx', msg.linear.x)
        self._check_limit('dvl', 'vy', msg.linear.y)
        self._check_limit('dvl', 'vz', msg.linear.z)
        # Feed linear speed (DVL overridden by Subsonus if fresh)
        if self._is_dead('subsonus_vel', time.time()):
            self._linear_speed = math.sqrt(
                msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)

    def _sv_cb(self, msg):
        self._touch('subsonus_vel')
        self._check_limit('subsonus_vel', 'vx', msg.linear.x)
        self._check_limit('subsonus_vel', 'vy', msg.linear.y)
        self._check_limit('subsonus_vel', 'vz', msg.linear.z)
        # Subsonus is primary speed source
        self._linear_speed = math.sqrt(
            msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)

    def _so_cb(self, msg):
        self._touch('subsonus_ori')
        self._check_limit('subsonus_ori', 'heading', msg.linear.x)
        self._check_limit('subsonus_ori', 'pitch',   msg.linear.y)
        self._check_limit('subsonus_ori', 'roll',    msg.linear.z)

    def _motor_cb(self, msg):
        def effort(pwm):
            return abs(pwm - MOTOR_NEUTRAL) / MOTOR_MAX_DEV
        self._motor_effort = max(
            effort(msg.linear.x),  effort(msg.linear.y),  effort(msg.linear.z),
            effort(msg.angular.x), effort(msg.angular.y), effort(msg.angular.z),
        )

    def _gps_cb(self, msg):
        self._touch('gps')
        sw = self.sensors['gps']
        no_fix = msg.status.status < 0
        if no_fix:
            if not sw.limit_alerted.get('fix'):
                self._alert('GPS: no fix', 'warn')
                sw.limit_alerted['fix'] = True
        else:
            if sw.limit_alerted.get('fix'):
                self._clear('GPS: fix acquired')
                sw.limit_alerted['fix'] = False

    # -----------------------------------------------------------------------
    # ESP32 auto-reset via DTR
    # -----------------------------------------------------------------------

    def _find_esp32_port(self):
        for dev in sorted(glob.glob('/dev/ttyUSB*')):
            try:
                out = subprocess.run(
                    ['udevadm', 'info', dev],
                    capture_output=True, text=True, timeout=2
                ).stdout
                if 'ID_VENDOR=Silicon_Labs' in out:
                    return dev
            except Exception:
                pass
        return None

    def _reset_esp32(self):
        port = self._find_esp32_port()
        if not port:
            self._alert('ESP32 auto-reset failed: port not found', 'error')
            return
        try:
            import serial
            s = serial.Serial(port, 115200)
            s.setDTR(False)
            time.sleep(0.2)
            s.setDTR(True)
            s.close()
            self._alert(f'ESP32 auto-reset triggered ({port})', 'warn')
        except Exception as e:
            self._alert(f'ESP32 auto-reset failed: {e}', 'error')

    # -----------------------------------------------------------------------
    # 1 Hz timeout watchdog
    # -----------------------------------------------------------------------

    def _is_dead(self, key, now):
        sw = self.sensors[key]
        if sw.last_time is None:
            return (now - self._start_time) > STARTUP_GRACE
        return (now - sw.last_time) > sw.timeout

    def _check_timeouts(self):
        now = time.time()
        past_grace = (now - self._start_time) > STARTUP_GRACE
        for key, sw in self.sensors.items():
            if sw.last_time is None:
                if past_grace and not sw.timeout_alerted:
                    self._alert(f'{sw.name}: no data received since startup', 'error')
                    sw.timeout_alerted = True
                continue
            age = now - sw.last_time
            if age > sw.timeout:
                if not sw.timeout_alerted:
                    self._alert(
                        f'{sw.name} not responding ({age:.0f}s since last message)',
                        'error'
                    )
                    sw.timeout_alerted = True
            else:
                if sw.timeout_alerted:
                    self._clear(f'{sw.name} restored')
                    sw.timeout_alerted = False

        # Frequency check: prune old timestamps and verify actual Hz vs expected
        for key, sw in self.sensors.items():
            if sw.expected_hz <= 0 or sw.last_time is None:
                continue
            # Drop timestamps outside the rolling window
            cutoff = now - FREQ_WINDOW
            while sw.msg_times and sw.msg_times[0] < cutoff:
                sw.msg_times.popleft()
            # Need at least a full window of history before checking
            if (now - self._start_time) < (STARTUP_GRACE + FREQ_WINDOW):
                continue
            if sw.timeout_alerted:
                # Already alerting on timeout, skip freq alert
                continue
            count = len(sw.msg_times)
            # actual Hz = messages in the window / window length
            actual_hz = count / FREQ_WINDOW
            min_hz = sw.expected_hz * FREQ_THRESHOLD
            if actual_hz < min_hz:
                if not sw.freq_alerted:
                    self._alert(
                        f'{sw.name} low frequency: {actual_hz:.1f} Hz '
                        f'(expected {sw.expected_hz:.0f} Hz)',
                        'warn'
                    )
                    sw.freq_alerted = True
            else:
                if sw.freq_alerted:
                    self._clear(
                        f'{sw.name} frequency restored: {actual_hz:.1f} Hz'
                    )
                    sw.freq_alerted = False

        # Motor cross-check: high effort + no motion → possible stuck thruster
        has_velocity_source = (
            not self._is_dead('subsonus_vel', now) or
            not self._is_dead('dvl', now)
        )
        has_heading_source = not self._is_dead('bno055', now)

        if has_velocity_source or has_heading_source:
            moving = (
                (has_velocity_source and self._linear_speed >= MOTION_SPEED_THR) or
                (has_heading_source  and self._heading_rate  >= MOTION_HDG_RATE)
            )
            high_effort = self._motor_effort >= MOTOR_EFFORT_THR

            if high_effort and not moving:
                if self._stuck_since is None:
                    self._stuck_since = now
                elif (now - self._stuck_since) >= MOTOR_STUCK_TIME and not self._stuck_alerted:
                    self._alert(
                        f'Motors commanded ({self._motor_effort*100:.0f}% effort) '
                        f'but no motion detected', 'warn'
                    )
                    self._stuck_alerted = True
            else:
                if self._stuck_alerted:
                    self._clear('Motor response normal')
                    self._stuck_alerted = False
                self._stuck_since = None

        # Publish per-sensor health bitmask (1 = healthy) for the acoustic link
        bitmask = 0
        for i, key in enumerate(HEALTH_ORDER):
            sw = self.sensors[key]
            healthy = (
                not self._is_dead(key, now)
                and not any(sw.limit_alerted.values())
                and not sw.freq_alerted
            )
            if healthy:
                bitmask |= (1 << i)
        status = UInt16()
        status.data = bitmask
        self.status_pub.publish(status)

        # Auto-reset ESP32 when both I2C sensors (BNO055 + BAR100) are dead
        if past_grace and self._is_dead('bno055', now) and self._is_dead('bar100', now):
            if self._both_dead_since is None:
                self._both_dead_since = now
            elif (now - self._both_dead_since) >= ESP32_RESTART_TIMEOUT:
                if (now - self._last_esp32_reset) >= ESP32_RESET_COOLDOWN:
                    self._last_esp32_reset = now
                    self._both_dead_since = now  # reset timer for next cycle
                    self._reset_esp32()
        else:
            self._both_dead_since = None


def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
