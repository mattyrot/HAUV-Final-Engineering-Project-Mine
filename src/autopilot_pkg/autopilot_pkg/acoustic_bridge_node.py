"""
acoustic_bridge_node.py — runs on UP Board
Bridges ROS2 topics <-> Subsonus acoustic modem (TCP port 16740).

Receives:  command packets from surface -> publishes /joy
Sends:     telemetry packets (IMU, depth, DVL, temp, GPS) -> surface

Deploy to install tree same as guidance_node.py, then run:
  python3 /home/up/rov_ws/install/autopilot_pkg/lib/python3.8/site-packages/autopilot_pkg/acoustic_bridge_node.py
"""

import struct
import socket
import threading
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, NavSatFix
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64, UInt16

# Sensor order for the /health/status bitmask (bit 0 = first). Must match
# HEALTH_ORDER in health_monitor_node.py.
HEALTH_ORDER = ['bno055', 'bar100', 'bme280', 'leak',
                'dvl', 'subsonus_vel', 'subsonus_ori', 'gps']

# ── link mode ────────────────────────────────────────────────────────────────
# DIRECT_TCP_TEST = True: bench test over plain Ethernet. The bridge runs a TCP
#   SERVER on the UP Board and the surface PC connects straight to 192.168.168.101.
#   No Subsonus, no water — exercises the whole packet path except the acoustic hop.
# DIRECT_TCP_TEST = False: real link. The bridge connects out to the Subsonus
#   transparent modem port, which transmits acoustically to the remote unit.
DIRECT_TCP_TEST = True
TEST_PORT     = 17000               # server port the surface PC connects to

SUBSONUS_IP   = '192.168.168.103'   # ← change to actual Subsonus IP
SUBSONUS_PORT = 16740
RECONNECT_S   = 3.0
TEL_RATE_HZ   = 2.0

CMD_MAGIC = 0xAC
TEL_MAGIC = 0xAB

# Command (surface → ROV): 8 bytes
# magic(B) fwd(b) strafe(b) yaw(b) vert(b) cam(b) buttons(H)
CMD_FMT  = '!BbbbbbH'
CMD_SIZE = struct.calcsize(CMD_FMT)   # 8

# Telemetry (ROV → surface): 32 bytes
# magic(B) pitch(h) roll(h) yaw(h) depth(H) dvl_e(h) dvl_n(h) dvl_u(h)
#           dvl_range(H) temp(h) lat(i) lon(i) pressure(H) leak(B) health(H)
# health is a per-sensor OK bitmask from /health/status (see HEALTH_ORDER).
TEL_FMT  = '!BhhhHhhhHhiiHBH'
TEL_SIZE = struct.calcsize(TEL_FMT)   # 32

# Auto-GOTO target (surface → ROV): 9 bytes — magic(B) lat(i) lon(i), each deg*1e7
GOTO_MAGIC = 0xAD
GOTO_FMT   = '!Bii'
GOTO_SIZE  = struct.calcsize(GOTO_FMT)   # 9


import math


def _f(v):
    """Sanitize a float: NaN/inf -> 0.0 so struct.pack never sees a bad value."""
    return v if isinstance(v, (int, float)) and math.isfinite(v) else 0.0

def _clamp_i16(v):
    return max(-32768, min(32767, int(_f(v))))

def _clamp_u16(v):
    return max(0, min(65535, int(_f(v))))

def _clamp_i32(v):
    return max(-2147483648, min(2147483647, int(_f(v))))

def _clamp_i8(v):
    return max(-128, min(127, int(_f(v))))

def _clamp_u8(v):
    return max(0, min(255, int(_f(v))))


class AcousticBridgeNode(Node):
    def __init__(self):
        super().__init__('acoustic_bridge_node')

        # Telemetry state
        self.pitch       = 0.0
        self.roll        = 0.0
        self.yaw         = 0.0
        self.depth       = 0.0
        self.temperature = 0.0
        self.dvl_e       = 0.0
        self.dvl_n       = 0.0
        self.dvl_u       = 0.0
        self.dvl_range   = 0.0
        self.gps_lat     = 0.0
        self.gps_lon     = 0.0
        self.pressure    = 0.0
        self.leak        = 0.0
        self.health      = 0

        # ROS subscriptions
        self.create_subscription(Twist,     '/esp32/bno055_data',    self._imu_cb,       10)
        self.create_subscription(Vector3,   '/esp32/bar100_data',    self._bar_cb,       10)
        self.create_subscription(Twist,     '/dvl/velocity_data',    self._dvl_vel_cb,   10)
        self.create_subscription(Float64,   '/dvl/range_to_bottom',  self._dvl_range_cb, 10)
        # GPS from the u-blox NEO-M8N (dedicated module, always available)
        self.create_subscription(NavSatFix, '/gps/fix',              self._gps_cb,       10)
        self.create_subscription(Float64,   '/esp32/leak',           self._leak_cb,      10)
        self.create_subscription(UInt16,    '/health/status',        self._health_cb,    10)

        # Joy publisher — decoded commands are re-published here for guidance_node
        self.joy_pub = self.create_publisher(Joy, '/joy_acoustic', 10)
        # Auto-GOTO target from the surface (acoustic 'go to location')
        self.goto_pub = self.create_publisher(NavSatFix, '/guidance/goto_target', 10)

        # TCP socket (managed by background thread)
        self._sock      = None
        self._sock_lock = threading.Lock()
        threading.Thread(target=self._tcp_loop, daemon=True).start()

        self.create_timer(1.0 / TEL_RATE_HZ, self._send_telemetry)
        mode = f'DIRECT TCP TEST (server :{TEST_PORT})' if DIRECT_TCP_TEST \
            else f'acoustic link ({SUBSONUS_IP}:{SUBSONUS_PORT})'
        self.get_logger().info(f'Acoustic bridge started — mode: {mode}')

    # ── ROS callbacks ──────────────────────────────────────────────────────────

    def _imu_cb(self, msg):
        self.yaw   = msg.linear.x
        self.pitch = msg.linear.y
        self.roll  = msg.linear.z

    def _bar_cb(self, msg):
        self.depth       = msg.x   # x = depth (m)
        self.pressure    = msg.y   # y = pressure (mbar)
        self.temperature = msg.z   # z = temperature (°C)

    def _leak_cb(self, msg):
        self.leak = msg.data       # 0 = dry, 1 = leak

    def _health_cb(self, msg):
        self.health = msg.data     # per-sensor OK bitmask (see HEALTH_ORDER)

    def _dvl_vel_cb(self, msg):
        self.dvl_e = msg.linear.x
        self.dvl_n = msg.linear.y
        self.dvl_u = msg.linear.z

    def _dvl_range_cb(self, msg):
        self.dvl_range = msg.data

    def _gps_cb(self, msg):
        self.gps_lat = msg.latitude
        self.gps_lon = msg.longitude

    # ── telemetry sender ───────────────────────────────────────────────────────

    def _send_telemetry(self):
        with self._sock_lock:
            sock = self._sock
        if sock is None:
            return
        try:
            payload = struct.pack(
                TEL_FMT,
                TEL_MAGIC,
                _clamp_i16(self.pitch * 100),
                _clamp_i16(self.roll  * 100),
                _clamp_i16(self.yaw   * 10),
                _clamp_u16(self.depth * 100),
                _clamp_i16(self.dvl_e * 1000),
                _clamp_i16(self.dvl_n * 1000),
                _clamp_i16(self.dvl_u * 1000),
                _clamp_u16(self.dvl_range * 100),
                _clamp_i16(self.temperature * 100),
                _clamp_i32(self.gps_lat * 1e6),
                _clamp_i32(self.gps_lon * 1e6),
                _clamp_u16(self.pressure),
                _clamp_u8(1 if self.leak else 0),
                _clamp_u16(self.health),
            )
            sock.sendall(payload)
        except Exception as e:
            self.get_logger().warn(f'Telemetry send error: {e}')
            with self._sock_lock:
                if self._sock is sock:
                    self._sock = None
            try:
                sock.close()
            except Exception:
                pass

    # ── TCP receive loop (background thread) ───────────────────────────────────

    def _tcp_loop(self):
        if DIRECT_TCP_TEST:
            self._server_loop()
        else:
            self._client_loop()

    def _client_loop(self):
        """Real link: connect out to the Subsonus transparent modem port."""
        while rclpy.ok():
            sock = None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect((SUBSONUS_IP, SUBSONUS_PORT))
                sock.settimeout(1.0)
                with self._sock_lock:
                    self._sock = sock
                self.get_logger().info(f'Subsonus TCP connected ({SUBSONUS_IP}:{SUBSONUS_PORT})')
                self._serve(sock)
            except Exception as e:
                self.get_logger().warn(f'Subsonus TCP error: {e} — retry in {RECONNECT_S}s')
            with self._sock_lock:
                if self._sock is sock:
                    self._sock = None
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass
            time.sleep(RECONNECT_S)

    def _server_loop(self):
        """Bench test: listen on TEST_PORT for the surface PC (plain Ethernet)."""
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('0.0.0.0', TEST_PORT))
        srv.listen(1)
        srv.settimeout(1.0)
        self.get_logger().info(f'DIRECT_TCP_TEST — waiting for surface PC on port {TEST_PORT}')
        while rclpy.ok():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().warn(f'Accept error: {e}')
                time.sleep(RECONNECT_S)
                continue
            conn.settimeout(1.0)
            with self._sock_lock:
                self._sock = conn
            self.get_logger().info(f'Surface PC connected from {addr[0]}:{addr[1]}')
            try:
                self._serve(conn)
            except Exception as e:
                self.get_logger().warn(f'Client session error: {e}')
            with self._sock_lock:
                if self._sock is conn:
                    self._sock = None
            try:
                conn.close()
            except Exception:
                pass
            self.get_logger().info('Surface PC disconnected — waiting for reconnect')

    def _serve(self, sock):
        """Receive command packets from the given socket until it closes."""
        buf = b''
        while rclpy.ok():
            with self._sock_lock:
                if self._sock is not sock:
                    break
            try:
                chunk = sock.recv(256)
                if not chunk:
                    break
                buf += chunk
                buf = self._consume(buf)
            except socket.timeout:
                pass

    def _consume(self, buf):
        """Parse a mixed stream of command (0xAC) and goto (0xAD) packets."""
        while buf:
            m = buf[0]
            if m == CMD_MAGIC:
                if len(buf) < CMD_SIZE:
                    break
                self._handle_command(buf[:CMD_SIZE])
                buf = buf[CMD_SIZE:]
            elif m == GOTO_MAGIC:
                if len(buf) < GOTO_SIZE:
                    break
                self._handle_goto(buf[:GOTO_SIZE])
                buf = buf[GOTO_SIZE:]
            else:
                # unknown byte — resync to the next known magic
                c = buf.find(bytes([CMD_MAGIC]), 1)
                g = buf.find(bytes([GOTO_MAGIC]), 1)
                cands = [i for i in (c, g) if i >= 0]
                if not cands:
                    return b''
                buf = buf[min(cands):]
        return buf

    def _handle_goto(self, raw):
        try:
            _, lat_e7, lon_e7 = struct.unpack(GOTO_FMT, raw)
        except struct.error:
            return
        lat, lon = lat_e7 / 1e7, lon_e7 / 1e7
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.status.status = 0  # STATUS_FIX = valid target
        fix.latitude = lat
        fix.longitude = lon
        self.goto_pub.publish(fix)
        self.get_logger().info(f'Acoustic GOTO target -> {lat:.7f}, {lon:.7f}')

    def _handle_command(self, raw):
        try:
            _, fwd, strafe, yaw, vert, cam, buttons = struct.unpack(CMD_FMT, raw)
        except struct.error:
            return

        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        axes = [0.0] * 6
        axes[1] = fwd    / 100.0   # forward/back
        axes[0] = strafe / 100.0   # left/right
        axes[3] = yaw    / 100.0   # yaw
        axes[4] = vert   / 100.0   # vertical
        if cam > 0:
            axes[5] = 1.0 - cam / 100.0   # cam up (axis 5 resting = 1.0)
        elif cam < 0:
            axes[2] = 1.0 + cam / 100.0   # cam down (axis 2 resting = 1.0)
        else:
            axes[5] = 1.0
            axes[2] = 1.0
        msg.axes = axes
        msg.buttons = [int(bool(buttons & (1 << i))) for i in range(16)]
        self.joy_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AcousticBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
