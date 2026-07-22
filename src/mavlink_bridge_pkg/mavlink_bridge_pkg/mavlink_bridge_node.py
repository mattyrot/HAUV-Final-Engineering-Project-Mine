#!/usr/bin/env python3
"""
MAVLink Bridge Node - Uses pymavlink for reliable MAVLink v2 communication.
Bridges between QGroundControl (MAVLink) and ROS2 (HAUV).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64, Bool, String
from sensor_msgs.msg import NavSatFix, NavSatStatus, Joy
import socket
import math
import time
import io
import subprocess
from threading import Thread
from pymavlink.dialects.v20 import ardupilotmega as mavlink2


class MAVLinkBridgeNode(Node):
    """
    Bridge between MAVLink (QGroundControl) and ROS2 (HAUV) using pymavlink.
    """

    def __init__(self):
        super().__init__('mavlink_bridge_node')
        self.get_logger().info("MAVLink Bridge Node starting...")

        # Network configuration
        self.mavlink_port = 14550
        self.qgc_ip = "192.168.168.1"
        self.qgc_port = 14550  # updated to actual source port once QGC sends a packet
        
        # Socket setup
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_socket.bind(('0.0.0.0', self.mavlink_port))
        self.udp_socket.settimeout(0.1)
        
        # MAVLink instance (pymavlink) - uses BytesIO buffer
        self.mav_out = io.BytesIO()
        self.mav = mavlink2.MAVLink(self.mav_out)
        self.mav.srcSystem = 1
        self.mav.srcComponent = 1

        # Publisher — feeds guidance_node as if a physical joystick were connected
        self.joy_publisher = self.create_publisher(Joy, '/joy', 10)
        # Watchdog: publish QGC connection status for ESP32 indicator LED
        self.qgc_status_pub = self.create_publisher(Bool, '/qgc_status', 10)
        # Mode publisher for guidance_node
        self.qgc_mode_pub = self.create_publisher(String, '/qgc_mode', 10)
        # Auto-GOTO target (from QGC 'Go to location') -> guidance_node
        self.goto_pub = self.create_publisher(NavSatFix, '/guidance/goto_target', 10)
        self._upload_mission_type = 0   # mission_type of an in-progress QGC upload

        # Subscribers (for telemetry to send to QGC)
        self.orientation_subscription = self.create_subscription(
            Twist, '/esp32/bno055_data', self.orientation_callback, 10)
        self.subsonus_vel_subscription = self.create_subscription(
            Twist, '/subsonus/velocity', self.subsonus_velocity_callback, 10)
        self.dvl_subscription = self.create_subscription(
            Twist, '/dvl/velocity_data', self.dvl_callback, 10)
        self.dvl_range_subscription = self.create_subscription(
            Float64, '/dvl/range_to_bottom', self.dvl_range_callback, 10)
        self.depth_subscription = self.create_subscription(
            Vector3, '/esp32/bar100_data', self.depth_callback, 10)
        self.leak_subscription = self.create_subscription(
            Float64, '/esp32/leak', self.leak_callback, 10)
        self.gps_subscription = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.bme280_subscription = self.create_subscription(
            Vector3, '/esp32/bme280_data', self.bme280_callback, 10)
        self.health_subscription = self.create_subscription(
            String, '/health/alert', self.health_alert_callback, 10)

        # State variables
        self.orientation = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self.velocity = {"vx": 0.0, "vy": 0.0, "vz": 0.0}
        self.position = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.depth = 0.0
        self.pressure = 0.0
        self.temperature = 0.0
        self.leak = False
        self._leak_alert_t = 0.0   # last leak STATUSTEXT time (for repeat while active)
        self.battery_voltage = 12.0
        self.bme_temp = 0.0
        self.bme_pressure = 0.0
        self.bme_humidity = 0.0
        self.armed = True  # boot armed so QGC shows "Armed"/ready, not "Not Ready"
                           # (cosmetic: motors respond to the joystick regardless of arm state)
        self.current_custom_mode = 19  # ArduSub MANUAL by default
        self.cpu_percent = 0.0
        self.cpu_temp = 0.0
        self.gps_lat = 0.0
        self.gps_lon = 0.0
        self.gps_alt = 0.0
        self.gps_fix = 0
        self.gps_hdop = 9999
        self.home_lat = None
        self.home_lon = None
        self.home_alt = None
        self.home_set = False

        # Watchdog: track last message times from sensors and QGC
        self.last_subsonus_vel_time = 0.0
        self.last_dvl_vel_time = 0.0
        self.dvl_range = 0.0
        self.last_bno_time = 0.0
        self.last_bar_time = 0.0
        self.last_qgc_msg_time = 0.0
        self.bno_alert_sent = False
        self.bar_alert_sent = False
        self.restart_alert_sent = False

        # Timers
        self.heartbeat_timer = self.create_timer(1.0, self.send_heartbeat)
        self.telemetry_timer = self.create_timer(0.05, self.send_telemetry)  # 20 Hz
        self.home_timer = self.create_timer(5.0, self.send_home_position)
        self.sysinfo_timer = self.create_timer(2.0, self.update_system_info)
        self.watchdog_timer = self.create_timer(1.0, self.run_watchdog)

        # UDP receive thread
        self.udp_thread = Thread(target=self.receive_mavlink_messages, daemon=True)
        self.udp_thread.start()

        self.get_logger().info(f"Listening for MAVLink on UDP port {self.mavlink_port}")

    def orientation_callback(self, msg):
        """Update orientation from BNO055."""
        self.orientation["roll"] = msg.linear.z
        self.orientation["pitch"] = msg.linear.y
        self.orientation["yaw"] = msg.linear.x
        self.last_bno_time = time.time()
        self.bno_alert_sent = False
        self.restart_alert_sent = False

    def dvl_callback(self, msg):
        """Update velocity from DVL (primary source). linear.x=E, y=N, z=U."""
        self.velocity["vx"] = msg.linear.x
        self.velocity["vy"] = msg.linear.y
        self.velocity["vz"] = msg.linear.z
        self.last_dvl_vel_time = time.time()

    def dvl_range_callback(self, msg):
        """Receive range-to-bottom from DVL and forward to QGC as DISTANCE_SENSOR."""
        self.dvl_range = msg.data
        self.send_distance_sensor(msg.data)

    def send_distance_sensor(self, range_m):
        if not self.qgc_ip:
            return
        distance_cm = max(1, min(int(range_m * 100), 65535))
        self.mav_out.seek(0)
        self.mav_out.truncate(0)
        self.mav.distance_sensor_send(
            time_boot_ms=int((time.time() % 1000) * 1000),
            min_distance=1,        # cm
            max_distance=5000,     # 50 m max range
            current_distance=distance_cm,
            type=0,                # MAV_DISTANCE_SENSOR_LASER (generic)
            id=0,
            orientation=25,        # MAV_SENSOR_ROTATION_PITCH_270 = downward
            covariance=255,        # unknown
        )
        self.send_mavlink_message(self.mav_out.getvalue())

    def subsonus_velocity_callback(self, msg):
        """Update velocity from Subsonus — fallback only when DVL data is stale (>2 s)."""
        self.last_subsonus_vel_time = time.time()
        if time.time() - self.last_dvl_vel_time > 2.0:
            self.velocity["vx"] = msg.linear.x
            self.velocity["vy"] = msg.linear.y
            self.velocity["vz"] = msg.linear.z

    def depth_callback(self, msg):
        """Update depth, pressure, and temperature from BAR100."""
        self.depth = msg.x
        self.pressure = msg.y
        self.temperature = msg.z
        self.last_bar_time = time.time()
        self.bar_alert_sent = False

    def health_alert_callback(self, msg):
        """Forward health monitor alerts to QGC as STATUSTEXT."""
        text = msg.data
        severity = 2  # MAV_SEVERITY_CRITICAL for errors, 4 for warnings
        lower = text.lower()
        if 'leak' in lower or 'not responding' in lower or 'no data' in lower or 'out of range' in lower:
            severity = 2  # CRITICAL
        elif 'no fix' in lower:
            severity = 4  # WARNING
        elif 'restored' in lower or 'recovered' in lower or 'clear' in lower or 'acquired' in lower:
            severity = 6  # INFO
        self.send_statustext(text[:50], severity=severity)

    def bme280_callback(self, msg):
        self.bme_temp = msg.x
        self.bme_pressure = msg.y
        self.bme_humidity = msg.z

    def gps_callback(self, msg):
        self.gps_lat = msg.latitude
        self.gps_lon = msg.longitude
        self.gps_alt = msg.altitude
        self.gps_fix = msg.status.status
        self.gps_hdop = int(msg.position_covariance[0] ** 0.5 * 100) if msg.position_covariance[0] > 0 else 9999
        if not self.home_set and msg.status.status >= 0 and msg.latitude != 0.0:
            self.home_lat = msg.latitude
            self.home_lon = msg.longitude
            self.home_alt = msg.altitude
            self.home_set = True
            self.get_logger().info(f"Home auto-set to first GPS fix: {self.home_lat:.6f}, {self.home_lon:.6f}")

    def leak_callback(self, msg):
        leaked = msg.data >= 0.5
        now = time.time()
        # Fire on the rising edge, then repeat every 3 s while it persists so a
        # dropped STATUSTEXT can't hide an active leak. Guidance auto-surfaces on
        # the same signal (board-side), so the message says so.
        if leaked and (not self.leak or now - self._leak_alert_t >= 3.0):
            self.send_statustext("LEAK! Auto-surfacing", severity=0)  # 0 = EMERGENCY
            self.get_logger().error("LEAK DETECTED — auto-surfacing")
            self._leak_alert_t = now
        self.leak = leaked

    def update_system_info(self):
        try:
            # CPU % from /proc/stat (two samples 0.1s apart)
            def read_cpu_times():
                with open('/proc/stat') as f:
                    fields = f.readline().split()
                idle = int(fields[4])
                total = sum(int(x) for x in fields[1:])
                return idle, total
            i1, t1 = read_cpu_times()
            time.sleep(0.1)
            i2, t2 = read_cpu_times()
            self.cpu_percent = 100.0 * (1.0 - (i2 - i1) / max(t2 - t1, 1))
        except Exception:
            self.cpu_percent = 0.0
        try:
            import glob
            paths = glob.glob('/sys/devices/platform/coretemp.*/hwmon/hwmon*/temp*_input')
            if paths:
                with open(paths[0]) as f:
                    self.cpu_temp = int(f.read().strip()) / 1000.0
        except Exception:
            self.cpu_temp = 0.0

        if self.qgc_ip is None:
            return
        try:
            time_boot_ms = int(self.get_clock().now().nanoseconds / 1_000_000) & 0xFFFFFFFF
            # NAMED_VALUE_FLOAT for CPU temp and BME pressure (no dedicated message)
            for name, value in [
                (b'CpuTemp\0\0\0', self.cpu_temp),
                (b'BMEPress\0\0',  self.bme_pressure),
            ]:
                self.mav_out.seek(0)
                self.mav_out.truncate(0)
                self.mav.named_value_float_send(
                    time_boot_ms=time_boot_ms,
                    name=name,
                    value=float(value)
                )
                self.send_mavlink_message(self.mav_out.getvalue())
            # HYGROMETER_SENSOR — native MAVLink message for humidity + temperature
            self.mav_out.seek(0)
            self.mav_out.truncate(0)
            self.mav.hygrometer_sensor_send(
                id=0,
                temperature=int(self.bme_temp * 100),   # centi-degrees C
                humidity=int(self.bme_humidity * 100)    # centi-percent
            )
            self.send_mavlink_message(self.mav_out.getvalue())
        except Exception as e:
            self.get_logger().error(f"Error sending system info: {e}")

    def send_home_position(self):
        if self.qgc_ip is None or not self.home_set:
            return
        try:
            self.mav_out.seek(0)
            self.mav_out.truncate(0)
            self.mav.home_position_send(
                latitude=int(self.home_lat * 1e7),
                longitude=int(self.home_lon * 1e7),
                altitude=int(self.home_alt * 1000),
                x=0.0, y=0.0, z=0.0,
                q=[1.0, 0.0, 0.0, 0.0],
                approach_x=0.0, approach_y=0.0, approach_z=0.0
            )
            self.send_mavlink_message(self.mav_out.getvalue())
        except Exception as e:
            self.get_logger().error(f"Error sending home position: {e}")

    def run_watchdog(self):
        """1 Hz: publish QGC connection status and alert on dead sensors."""
        now = time.time()
        SENSOR_TIMEOUT = 5.0  # seconds without data = sensor dead

        # Publish QGC connected status for ESP32 LED
        qgc_connected = (now - self.last_qgc_msg_time) < SENSOR_TIMEOUT
        msg = Bool()
        msg.data = qgc_connected
        self.qgc_status_pub.publish(msg)

        # Only alert once QGC is known and sensors have been seen at least once
        if self.qgc_ip is None:
            return

        bno_dead = self.last_bno_time > 0 and (now - self.last_bno_time) > SENSOR_TIMEOUT
        bar_dead = self.last_bar_time > 0 and (now - self.last_bar_time) > SENSOR_TIMEOUT

        if bno_dead and not self.bno_alert_sent:
            self.send_statustext("BNO055 IMU not responding!", severity=2)
            self.get_logger().error("Watchdog: BNO055 IMU not responding!")
            self.bno_alert_sent = True

        if bar_dead and not self.bar_alert_sent:
            self.send_statustext("BAR100 depth sensor not responding!", severity=2)
            self.get_logger().error("Watchdog: BAR100 not responding!")
            self.bar_alert_sent = True

        # Both sensors dead for 20 s → ESP32 is restarting
        RESTART_TIMEOUT = 20.0
        both_dead_long = (
            self.last_bno_time > 0 and (now - self.last_bno_time) > RESTART_TIMEOUT and
            self.last_bar_time > 0 and (now - self.last_bar_time) > RESTART_TIMEOUT
        )
        if both_dead_long and not self.restart_alert_sent:
            self.send_statustext("Both sensors lost - restarting ESP32", severity=1)
            self.get_logger().error("Watchdog: both sensors lost >20s, ESP32 restarting")
            self.restart_alert_sent = True
        elif not both_dead_long:
            self.restart_alert_sent = False

    def send_statustext(self, text, severity=6):
        try:
            self.mav_out.seek(0)
            self.mav_out.truncate(0)
            self.mav.statustext_send(severity=severity, text=text.encode())
            self.send_mavlink_message(self.mav_out.getvalue())
        except Exception as e:
            self.get_logger().error(f"Error sending statustext: {e}")

    def send_heartbeat(self):
        """Send MAVLink HEARTBEAT message."""
        if self.qgc_ip is None:
            return

        try:
            # Clear buffer and send heartbeat
            self.mav_out.seek(0)
            self.mav_out.truncate(0)
            
            base_mode = 0x40 | 0x01  # MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            if self.armed:
                base_mode |= 0x80  # MAV_MODE_FLAG_SAFETY_ARMED
            self.mav.heartbeat_send(
                type=12,  # MAV_TYPE_SUBMARINE
                autopilot=3,  # MAV_AUTOPILOT_ARDUPILOTMEGA (ArduSub)
                base_mode=base_mode,
                custom_mode=self.current_custom_mode,
                system_status=4  # MAV_STATE_ACTIVE
            )
            
            packet = self.mav_out.getvalue()
            if packet:
                self.send_mavlink_message(packet)
                self.get_logger().info(f"[HEARTBEAT] Sent {len(packet)} bytes to {self.qgc_ip}")
            else:
                self.get_logger().warn("[HEARTBEAT] Empty packet generated!")
        except Exception as e:
            self.get_logger().error(f"Error sending heartbeat: {e}")

    def send_telemetry(self):
        """Send telemetry messages to QGC."""
        if self.qgc_ip is None:
            return

        try:
            time_boot_ms = int(self.get_clock().now().nanoseconds / 1_000_000) & 0xFFFFFFFF

            # ATTITUDE message
            self.mav_out.seek(0)
            self.mav_out.truncate(0)
            self.mav.attitude_send(
                time_boot_ms=time_boot_ms,
                roll=math.radians(self.orientation["roll"]),
                pitch=math.radians(self.orientation["pitch"]),
                yaw=math.radians(self.orientation["yaw"]),
                rollspeed=0.0,
                pitchspeed=0.0,
                yawspeed=0.0
            )
            self.send_mavlink_message(self.mav_out.getvalue())

            # GLOBAL_POSITION_INT — use real GPS lat/lon so map shows correct location
            self.mav_out.seek(0)
            self.mav_out.truncate(0)
            self.mav.global_position_int_send(
                time_boot_ms=time_boot_ms,
                lat=int(self.gps_lat * 1e7),
                lon=int(self.gps_lon * 1e7),
                alt=int(self.gps_alt * 1000),
                relative_alt=int(-self.depth * 1000),
                vx=int(self.velocity["vx"] * 100),
                vy=int(self.velocity["vy"] * 100),
                vz=int(self.velocity["vz"] * 100),
                hdg=int(self.orientation["yaw"] * 100) % 36000
            )
            self.send_mavlink_message(self.mav_out.getvalue())

            # SYS_STATUS message
            self.mav_out.seek(0)
            self.mav_out.truncate(0)
            # Only report sensors we actually have so QGC pre-arm checks pass
            # MAV_SYS_STATUS_SENSOR: 0x01=gyro, 0x02=accel, 0x04=mag, 0x20=GPS, 0x1000=motor outputs
            sensor_mask = 0x1023  # gyro + accel + mag + motor outputs
            self.mav.sys_status_send(
                onboard_control_sensors_present=sensor_mask,
                onboard_control_sensors_enabled=sensor_mask,
                onboard_control_sensors_health=sensor_mask,
                load=int(self.cpu_percent * 10),  # units: 0.01%, so 5000 = 50%
                voltage_battery=int(self.battery_voltage * 1000),
                current_battery=-1,
                battery_remaining=100,
                drop_rate_comm=0,
                errors_comm=0,
                errors_count1=0,
                errors_count2=0,
                errors_count3=0,
                errors_count4=0
            )
            self.send_mavlink_message(self.mav_out.getvalue())

            # VFR_HUD — speed and heading overlay in QGC
            self.mav_out.seek(0)
            self.mav_out.truncate(0)
            groundspeed = math.sqrt(self.velocity["vx"]**2 + self.velocity["vy"]**2)
            self.mav.vfr_hud_send(
                airspeed=groundspeed,
                groundspeed=groundspeed,
                heading=int(self.orientation["yaw"]) % 360,
                throttle=0,
                alt=0.0,
                climb=-self.velocity["vz"]
            )
            self.send_mavlink_message(self.mav_out.getvalue())

            # SCALED_PRESSURE — BAR100 pressure (mbar) and temperature (cdegC)
            self.mav_out.seek(0)
            self.mav_out.truncate(0)
            self.mav.scaled_pressure_send(
                time_boot_ms=time_boot_ms,
                press_abs=float(self.pressure) if self.pressure else 1013.25,
                press_diff=0.0,
                temperature=int(self.temperature * 100) if not math.isnan(self.temperature) else 0
            )
            self.send_mavlink_message(self.mav_out.getvalue())

            # GPS_RAW_INT — real GPS from NEO-M8N
            fix_map = {-1: 0, 0: 3, 1: 4, 2: 4}  # NavSatStatus.STATUS_FIX=0 -> MAV 3D fix
            self.mav_out.seek(0)
            self.mav_out.truncate(0)
            self.mav.gps_raw_int_send(
                time_usec=int(self.get_clock().now().nanoseconds / 1000),
                fix_type=fix_map.get(self.gps_fix, 0),
                lat=int(self.gps_lat * 1e7),
                lon=int(self.gps_lon * 1e7),
                alt=int(self.gps_alt * 1000),
                eph=self.gps_hdop,
                epv=9999,
                vel=0,
                cog=0,
                satellites_visible=255
            )
            self.send_mavlink_message(self.mav_out.getvalue())

        except Exception as e:
            self.get_logger().error(f"Error sending telemetry: {e}")

    def send_mavlink_message(self, packet):
        """Send MAVLink packet to QGC."""
        try:
            if self.qgc_ip and self.qgc_port:
                self.udp_socket.sendto(packet, (self.qgc_ip, self.qgc_port))
                self.get_logger().debug(f"Sent MAVLink packet ({len(packet)} bytes) to {self.qgc_ip}:{self.qgc_port}")
        except Exception as e:
            self.get_logger().warn(f"Failed to send MAVLink message: {e}")

    def receive_mavlink_messages(self):
        """Receive incoming MAVLink messages (runs in background thread)."""
        while True:
            try:
                data, addr = self.udp_socket.recvfrom(1024)
                
                # Always update QGC address from incoming packets so replies go to the right port
                if addr[0] != self.qgc_ip or addr[1] != self.qgc_port:
                    self.qgc_ip = addr[0]
                    self.qgc_port = addr[1]
                    self.get_logger().info(f"QGC detected at {self.qgc_ip}:{self.qgc_port}")
                self.last_qgc_msg_time = time.time()

                self.process_mavlink_message(data)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"Error receiving MAVLink: {e}")

    def process_mavlink_message(self, data):
        """Process incoming MAVLink message."""
        try:
            # parse_buffer returns a list of decoded messages
            messages = self.mav.parse_buffer(data)
            if not messages:
                return

            for msg in messages:
                name = msg.get_type()
                if name == 'MANUAL_CONTROL':
                    self.handle_manual_control(msg)
                elif name == 'HEARTBEAT':
                    self.get_logger().debug("Received HEARTBEAT from QGC")
                elif name == 'COMMAND_LONG':
                    self.handle_command_long(msg)
                elif name == 'SET_GPS_GLOBAL_ORIGIN':
                    self.handle_set_gps_global_origin(msg)
                elif name == 'PARAM_REQUEST_LIST':
                    self.handle_param_request_list()
                elif name == 'PARAM_REQUEST_READ':
                    self.handle_param_request_read(msg)
                elif name == 'SET_MODE':
                    self._apply_mode(int(msg.custom_mode))
                elif name == 'MISSION_REQUEST_LIST':
                    self.handle_mission_request_list(msg)
                elif name == 'MISSION_COUNT':
                    self.handle_mission_count(msg)
                elif name == 'MISSION_CLEAR_ALL':
                    self.handle_mission_clear_all(msg)
                elif name == 'COMMAND_INT':
                    self.handle_command_int(msg)
                elif name == 'SET_POSITION_TARGET_GLOBAL_INT':
                    self._publish_goto(msg.lat_int / 1e7, msg.lon_int / 1e7, 'SET_POSITION_TARGET')
                elif name == 'MISSION_ITEM_INT':
                    self.handle_mission_item_int(msg)
                elif name == 'MISSION_ITEM':
                    self.handle_mission_item(msg)
                else:
                    # Any message type we don't explicitly handle (debug-level so QGC's
                    # periodic camera/message probes don't clutter the log)
                    if name not in ('MANUAL_CONTROL', 'HEARTBEAT', 'PARAM_REQUEST_READ'):
                        self.get_logger().debug(f"RX unhandled MAVLink: {name}")

        except Exception as e:
            self.get_logger().warn(f"Error processing MAVLink message: {e}")

    def send_param(self, name, value, index, count):
        try:
            self.mav_out.seek(0)
            self.mav_out.truncate(0)
            self.mav.param_value_send(
                param_id=name.encode().ljust(16, b'\0'),
                param_value=float(value),
                param_type=9,  # MAV_PARAM_TYPE_REAL32
                param_count=count,
                param_index=index
            )
            self.send_mavlink_message(self.mav_out.getvalue())
        except Exception as e:
            self.get_logger().error(f"Error sending param: {e}")

    def handle_param_request_list(self):
        params = [
            ('SYSID_THISMAV', 1),
            ('ARMING_CHECK',  0),
        ]
        for i, (name, val) in enumerate(params):
            self.send_param(name, val, i, len(params))

    def handle_param_request_read(self, msg):
        self.send_param('SYSID_THISMAV', 1, 0, 1)

    def handle_set_gps_global_origin(self, msg):
        try:
            lat = msg.latitude / 1e7
            lon = msg.longitude / 1e7
            alt = msg.altitude / 1000.0
            self.get_logger().info(f"GPS global origin set to {lat:.6f}, {lon:.6f}, {alt:.1f}m")
            # Confirm with GPS_GLOBAL_ORIGIN
            self.mav_out.seek(0)
            self.mav_out.truncate(0)
            self.mav.gps_global_origin_send(
                latitude=msg.latitude,
                longitude=msg.longitude,
                altitude=msg.altitude
            )
            self.send_mavlink_message(self.mav_out.getvalue())
        except Exception as e:
            self.get_logger().error(f"Error handling SET_GPS_GLOBAL_ORIGIN: {e}")

    def handle_mission_request_list(self, msg):
        try:
            mtype = getattr(msg, 'mission_type', 0)
            self.mav_out.seek(0)
            self.mav_out.truncate(0)
            self.mav.mission_count_send(
                msg.get_srcSystem(),
                msg.get_srcComponent(),
                0,  # count of items is 0
                mission_type=mtype
            )
            self.send_mavlink_message(self.mav_out.getvalue())
            self.get_logger().info(f"Sent MISSION_COUNT 0 for type {mtype} to QGC")
        except Exception as e:
            self.get_logger().error(f"Error handling MISSION_REQUEST_LIST: {e}")

    def handle_mission_clear_all(self, msg):
        try:
            mtype = getattr(msg, 'mission_type', 0)
            self.mav_out.seek(0)
            self.mav_out.truncate(0)
            self.mav.mission_ack_send(
                msg.get_srcSystem(),
                msg.get_srcComponent(),
                0,  # MAV_MISSION_ACCEPTED = 0
                mission_type=mtype
            )
            self.send_mavlink_message(self.mav_out.getvalue())
            self.get_logger().info(f"Sent MISSION_ACK for CLEAR_ALL type {mtype} to QGC")
        except Exception as e:
            self.get_logger().error(f"Error handling MISSION_CLEAR_ALL: {e}")

    def handle_mission_count(self, msg):
        """QGC uploads a (guided) waypoint via the mission-write handshake. Reply
        MISSION_REQUEST_INT(seq=0) so QGC sends us the item."""
        try:
            self._upload_mission_type = getattr(msg, 'mission_type', 0)
            self.get_logger().info(
                f"MISSION_COUNT={getattr(msg, 'count', '?')} type={self._upload_mission_type}"
                f" — requesting item 0")
            self.mav_out.seek(0)
            self.mav_out.truncate(0)
            self.mav.mission_request_int_send(
                msg.get_srcSystem(),
                msg.get_srcComponent(),
                0,  # seq
                mission_type=self._upload_mission_type
            )
            self.send_mavlink_message(self.mav_out.getvalue())
        except Exception as e:
            self.get_logger().error(f"Error handling MISSION_COUNT: {e}")

    def handle_mission_item(self, msg):
        """Deprecated float MISSION_ITEM — what QGC's 'Go to location' actually sends to
        this vehicle. x=lat, y=lon in DEGREES (not scaled). Forward as GOTO and ACK it."""
        try:
            mtype = getattr(msg, 'mission_type', self._upload_mission_type)
            self.get_logger().info(
                f"MISSION_ITEM seq={getattr(msg, 'seq', 0)} cmd={getattr(msg, 'command', 0)}"
                f" current={getattr(msg, 'current', 0)} x={msg.x} y={msg.y}")
            self._publish_goto(msg.x, msg.y, 'MISSION_ITEM')
            self.mav_out.seek(0)
            self.mav_out.truncate(0)
            self.mav.mission_ack_send(
                msg.get_srcSystem(),
                msg.get_srcComponent(),
                0,  # MAV_MISSION_ACCEPTED
                mission_type=mtype
            )
            self.send_mavlink_message(self.mav_out.getvalue())
        except Exception as e:
            self.get_logger().error(f"Error handling MISSION_ITEM: {e}")

    def handle_mission_item_int(self, msg):
        """Receive the guided/waypoint item, forward it as a GOTO target, and ACK it.
        The MISSION_ACK completes QGC's mission-write handshake (clears the error popup)."""
        try:
            mtype = getattr(msg, 'mission_type', self._upload_mission_type)
            self.get_logger().info(
                f"MISSION_ITEM_INT seq={getattr(msg, 'seq', 0)} cmd={getattr(msg, 'command', 0)}"
                f" current={getattr(msg, 'current', 0)} x={msg.x} y={msg.y}")
            self._publish_goto(msg.x / 1e7, msg.y / 1e7, 'MISSION_ITEM_INT')
            self.mav_out.seek(0)
            self.mav_out.truncate(0)
            self.mav.mission_ack_send(
                msg.get_srcSystem(),
                msg.get_srcComponent(),
                0,  # MAV_MISSION_ACCEPTED
                mission_type=mtype
            )
            self.send_mavlink_message(self.mav_out.getvalue())
        except Exception as e:
            self.get_logger().error(f"Error handling MISSION_ITEM_INT: {e}")

    def _apply_mode(self, custom_mode):
        """Apply an ArduSub custom_mode and notify guidance_node."""
        ARDUSUB_MODES = {0: "STABILIZE", 4: "GUIDED", 19: "MANUAL"}
        if custom_mode not in ARDUSUB_MODES:
            return False
        self.current_custom_mode = custom_mode
        mode_name = ARDUSUB_MODES[custom_mode]
        msg = String()
        msg.data = mode_name
        self.qgc_mode_pub.publish(msg)
        self.get_logger().info(f"Mode changed to {mode_name}")
        return True

    def handle_command_long(self, msg):
        MAV_CMD_DO_SET_HOME = 179
        MAV_CMD_COMPONENT_ARM_DISARM = 400
        MAV_CMD_DO_SET_MODE = 176
        MAV_CMD_DO_REPOSITION = 192
        MAV_RESULT_ACCEPTED = 0
        MAV_RESULT_UNSUPPORTED = 3
        try:
            if msg.command == MAV_CMD_COMPONENT_ARM_DISARM:
                self.armed = (msg.param1 == 1.0)
                self.get_logger().info(f"{'Armed' if self.armed else 'Disarmed'}")
                result = MAV_RESULT_ACCEPTED
            elif msg.command == MAV_CMD_DO_SET_MODE:
                accepted = self._apply_mode(int(msg.param2))
                result = MAV_RESULT_ACCEPTED if accepted else MAV_RESULT_UNSUPPORTED
            elif msg.command == MAV_CMD_DO_SET_HOME:
                if msg.param1 == 1:
                    # use current GPS position
                    self.home_lat = self.gps_lat
                    self.home_lon = self.gps_lon
                    self.home_alt = self.gps_alt
                else:
                    self.home_lat = msg.param5
                    self.home_lon = msg.param6
                    self.home_alt = msg.param7
                self.get_logger().info(f"Home set to {self.home_lat:.6f}, {self.home_lon:.6f}")
                result = MAV_RESULT_ACCEPTED
            elif msg.command == MAV_CMD_DO_REPOSITION:
                # 'Go to location' as COMMAND_LONG: param5=lat, param6=lon (deg)
                self._publish_goto(msg.param5, msg.param6, 'DO_REPOSITION(long)')
                result = MAV_RESULT_ACCEPTED
            else:
                self.get_logger().debug(f"Unhandled COMMAND_LONG cmd={msg.command}")
                result = MAV_RESULT_UNSUPPORTED

            # Send COMMAND_ACK
            self.mav_out.seek(0)
            self.mav_out.truncate(0)
            self.mav.command_ack_send(command=msg.command, result=result)
            self.send_mavlink_message(self.mav_out.getvalue())
        except Exception as e:
            self.get_logger().error(f"Error handling COMMAND_LONG: {e}")

    def handle_command_int(self, msg):
        """COMMAND_INT carries lat/lon as scaled ints (deg * 1e7) in x/y."""
        MAV_CMD_DO_REPOSITION = 192
        try:
            if msg.command == MAV_CMD_DO_REPOSITION:
                self._publish_goto(msg.x / 1e7, msg.y / 1e7, 'DO_REPOSITION(int)')
                result = 0  # ACCEPTED
            else:
                self.get_logger().debug(f"Unhandled COMMAND_INT cmd={msg.command}")
                result = 3  # UNSUPPORTED
            self.mav_out.seek(0)
            self.mav_out.truncate(0)
            self.mav.command_ack_send(command=msg.command, result=result)
            self.send_mavlink_message(self.mav_out.getvalue())
        except Exception as e:
            self.get_logger().error(f"Error handling COMMAND_INT: {e}")

    def _publish_goto(self, lat, lon, source):
        """Forward a 'go to' target (lat/lon in degrees) to guidance_node."""
        try:
            if abs(lat) > 90.0 or abs(lon) > 180.0 or (lat == 0.0 and lon == 0.0):
                self.get_logger().warn(f"Ignoring invalid GOTO from {source}: {lat}, {lon}")
                return
            fix = NavSatFix()
            fix.header.stamp = self.get_clock().now().to_msg()
            fix.status.status = NavSatStatus.STATUS_FIX  # 0 = valid target
            fix.status.service = NavSatStatus.SERVICE_GPS
            fix.latitude = float(lat)
            fix.longitude = float(lon)
            self.goto_pub.publish(fix)
            self.get_logger().info(f"GOTO target from {source}: {lat:.7f}, {lon:.7f}")
        except Exception as e:
            self.get_logger().error(f"Error publishing GOTO: {e}")

    def handle_manual_control(self, msg):
        """Translate QGC MANUAL_CONTROL into a Joy message for guidance_node."""
        try:
            joy = Joy()
            joy.header.stamp = self.get_clock().now().to_msg()

            # axes: match guidance_node's expected layout
            # axes[0]=strafe, axes[1]=forward, axes[2]=cam_down(trigger), axes[3]=yaw, axes[4]=vertical, axes[5]=cam_up(trigger)
            # QGC z is 0-1000 with 500=centre; convert to [-1, 1]
            vertical = (msg.z - 500) / 500.0
            joy.axes = [
                msg.y / 1000.0,   # axes[0] strafe left/right
                msg.x / 1000.0,   # axes[1] forward/back
                1.0,              # axes[2] cam down trigger (neutral=1)
                msg.r / 1000.0,   # axes[3] yaw
                vertical,         # axes[4] ascend/descend
                1.0,              # axes[5] cam up trigger (neutral=1)
            ]

            # buttons: map bitmask bits to array (guidance_node uses [0],[1],[5])
            buttons = msg.buttons if msg.buttons else 0
            joy.buttons = [(buttons >> i) & 1 for i in range(16)]

            self.joy_publisher.publish(joy)

        except Exception as e:
            self.get_logger().error(f"Error handling manual control: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MAVLinkBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
