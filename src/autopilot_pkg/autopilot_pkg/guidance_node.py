import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, NavSatFix
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64
import math
import time

# If no joystick/acoustic command arrives within this window, zero the thruster
# inputs (failsafe) so a dropped control link can't leave motors driving.
JOY_FAILSAFE_TIMEOUT = 1.5  # seconds

# ── Auto-GOTO (surface GPS navigation) ───────────────────────────────────────
GOTO_ARRIVAL_M     = 4.0     # m — within this of target = arrived (keep > GPS jitter)
GOTO_SLOWDOWN_M    = 12.0    # m — start ramping forward thrust down inside this range
GOTO_MAX_FWD       = 0.6     # 0..1 forward-thrust cap (raise toward 1.0 for "full speed")
GOTO_YAW_FULL      = 45.0    # deg heading error that commands full yaw
GOTO_ALIGN_DEG     = 45.0    # deg — only drive forward when heading error is under this
GOTO_MAX_DIST      = 1000.0  # m — ignore/cancel targets farther than this (sanity)
GOTO_ABORT_AXIS    = 0.2     # joystick deflection that cancels auto and returns to manual
GOTO_COMMS_TIMEOUT = 5.0     # s without /joy while navigating -> stop (lost operator link)
GOTO_HOLD_DEADZONE_M = 1.5   # within this of target: hold (no horizontal thrust), just depth
GOTO_LOITER_FWD      = 0.25  # gentle forward-thrust cap while loitering (in the hold zone)
GOTO_DEPTH_KP        = 0.35  # vertical thrust (0..1) per metre of depth error
GOTO_DEPTH_DEADBAND  = 0.3   # m — within this of the target depth, no vertical thrust
GOTO_VERT_MAX        = 0.5   # cap on the depth-hold vertical thrust

# ── Leak → auto-surface failsafe ─────────────────────────────────────────────
LEAK_TRIGGER_N = 2      # consecutive leak readings before the failsafe latches (debounce)
LEAK_ASCEND    = 0.8    # vertical thrust (0..1) for the emergency ascent
LEAK_SURFACE_M = 0.4    # m — with a fresh depth at/above this, stop ascending (hold at surface)
LEAK_CLEAR_SEC = 5.0    # s the sensor must read dry before auto-clearing back to manual

class GuidanceNode(Node):
    def __init__(self):
        super().__init__('guidance_node')
        self.get_logger().info(f"Guidance Node has been started!")

        # Publishers
        self.motors_publisher = self.create_publisher(Twist, '/motor_data', 10)
        self.lights_publisher = self.create_publisher(Vector3, '/lights_servo_data', 10)
        self.odometry_publisher = self.create_publisher(Odometry, '/odometry', 10)

        # Subscribers
        self.orientation_subscription = self.create_subscription(Twist, '/esp32/bno055_data', self.orientation_callback, 10)
        self.dvl_subscription = self.create_subscription(Twist, '/dvl/velocity_data', self.dvl_callback, 10)
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        # Acoustic link commands (from acoustic_bridge_node) drive guidance the
        # same way as a physical joystick — used for untethered QGC control.
        self.joy_acoustic_subscription = self.create_subscription(Joy, '/joy_acoustic', self.joy_callback, 10)
        self.bar100_subscription = self.create_subscription(Vector3, '/esp32/bar100_data', self.bar100_callback, 10)
        self.qgc_mode_subscription = self.create_subscription(String, '/qgc_mode', self.qgc_mode_callback, 10)
        # Auto-GOTO: current fix + target destination (target comes from mavlink_bridge)
        self.gps_subscription = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.goto_subscription = self.create_subscription(NavSatFix, '/guidance/goto_target', self.goto_callback, 10)
        # Leak sensor (0.0 dry / 1.0 leak) — drives the auto-surface failsafe
        self.leak_subscription = self.create_subscription(Float64, '/esp32/leak', self.leak_callback, 10)

        # Timers
        self.motor_light_pub_timer = self.create_timer(0.05, self.motor_light_publisher)  # 20 Hz

        # Variables
        self.start_time = self.get_clock().now()
        self.orientation = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.dvl_velocities = {"vx": 0.0, "vy": 0.0, "vz": 0.0}
        self.motor_velocity = {"motor1": 1500, "motor2": 1500, "motor3": 1500, "motor4": 1500, "motor5": 1500, "cam_servo": 90, "motor6": 1500}
        self.light_intensity = {"light_single": 1100, "light_couple": 1100}
        self.joystick_axes = [0.0] * 6
        self.joystick_buttons = [0] * 16
        self.joy_cam_up = 1.0
        self.joy_cam_down = 1.0
        self.joystick_mode = True
        self.previous_button_states = [False] * 16
        self._last_joy_time = 0.0   # for control-link failsafe

        # Auto-GOTO state
        self.gps = {"lat": 0.0, "lon": 0.0, "fix": False}
        self.goto_target = None        # {"lat","lon"} while navigating
        self.auto_goto_active = False
        self.depth = 0.0               # current depth (m) from BAR100
        self.goto_depth = 0.0          # depth to hold during GOTO (captured at engage)
        self._loitering = False

        # Leak → auto-surface failsafe (latched; overrides every other control)
        self.leak_value = 0.0
        self.leak_failsafe = False
        self._leak_count = 0
        self._leak_dry_since = None
        self._last_bar_time = 0.0      # freshness of the depth reading

        # Target orientation in autonomous mode
        self.target_orientation = {"x": 0.0, "y": 0.0, "z": 0.0}

        # PID constants
        self.kp_yaw = 15.0
        self.kd_yaw = 2.0
        self.kp_pitch = 15.0
        self.kd_pitch = 2.0
        self.kp_roll = 15.0
        self.kd_roll = 2.0
        self.state_estimate = [0.0, 0.0, 0.0]

        # Safety envelope — attitude limits
        self.ENVELOPE_ENABLED = False  # disabled until IMU offset is calibrated
        self.ENVELOPE_WARN  = 30.0   # deg — begin scaling inputs
        self.ENVELOPE_HARD  = 50.0   # deg — zero inputs, max correction
        self.ENVELOPE_GAIN  = 6.0    # PWM correction per degree beyond warn
        self._envelope_active = False
        # IMU mount offsets — auto-calibrated from first readings on startup
        self.PITCH_OFFSET   = 0.0
        self.ROLL_OFFSET    = 0.0
        self._offset_samples_pitch = []
        self._offset_samples_roll  = []
        self._OFFSET_N = 20   # number of readings to average

    def orientation_callback(self, msg):
        self.orientation["x"] = msg.linear.x
        self.orientation["y"] = msg.linear.y
        self.orientation["z"] = msg.linear.z

        if len(self._offset_samples_pitch) < self._OFFSET_N:
            self._offset_samples_pitch.append(msg.linear.y)
            self._offset_samples_roll.append(msg.linear.z)
            if len(self._offset_samples_pitch) == self._OFFSET_N:
                self.PITCH_OFFSET = sum(self._offset_samples_pitch) / self._OFFSET_N
                self.ROLL_OFFSET  = sum(self._offset_samples_roll)  / self._OFFSET_N
                self.ENVELOPE_ENABLED = True
                self.get_logger().info(
                    f'IMU offset calibrated — pitch={self.PITCH_OFFSET:.1f}° roll={self.ROLL_OFFSET:.1f}°'
                )

    def dvl_callback(self, msg):
        self.dvl_velocities["vx"] = msg.linear.x
        self.dvl_velocities["vy"] = msg.linear.y
        self.dvl_velocities["vz"] = msg.linear.z

    def joy_callback(self, msg):
        self._last_joy_time = time.time()
        self.joystick_axes[1] = msg.axes[1]
        self.joystick_axes[0] = msg.axes[0]
        self.joystick_axes[4] = msg.axes[4]
        self.joystick_axes[3] = msg.axes[3]
        self.joy_cam_up = msg.axes[5]
        self.joy_cam_down = msg.axes[2]

        # Auto-GOTO: any real manual stick input cancels autonomous navigation
        if self.auto_goto_active and (
                abs(msg.axes[0]) > GOTO_ABORT_AXIS or abs(msg.axes[1]) > GOTO_ABORT_AXIS or
                abs(msg.axes[3]) > GOTO_ABORT_AXIS or abs(msg.axes[4]) > GOTO_ABORT_AXIS):
            self.auto_goto_active = False
            self.goto_target = None
            self.joystick_mode = True
            self.get_logger().info('GOTO cancelled — manual joystick override')

        n = len(msg.buttons)
        for i in range(min(n, 16)):
            self.joystick_buttons[i] = msg.buttons[i]

        # Mode toggle — button[5]
        if self.joystick_buttons[5] and not self.previous_button_states[5]:
            self.joystick_mode = not self.joystick_mode
            self.get_logger().info(f'Switched to {"Joystick" if self.joystick_mode else "Autonomous"} mode')
            if not self.joystick_mode:
                self.target_orientation = self.orientation.copy()

        # Light toggles — buttons[0] and [1]
        if self.joystick_buttons[0] and not self.previous_button_states[0]:
            self.light_intensity["light_single"] = 1900 if self.light_intensity["light_single"] == 1100 else 1100
        if self.joystick_buttons[1] and not self.previous_button_states[1]:
            self.light_intensity["light_couple"] = 1900 if self.light_intensity["light_couple"] == 1100 else 1100

        # Camera servo (BUTTONS only — over QGC the camera triggers/axes can't reach
        # it; MANUAL_CONTROL's 4 axes are all used for thrusters).
        #   UP:   LB (button 4)  or left stick-click  (button 9)
        #   DOWN: RB (button 5)  or right stick-click (button 10)
        # Bumpers only pass through if not assigned to a QGC function; the
        # stick-clicks (9/10) always pass through as a fallback.
        # Range limited to 60-120 (the range the working test sketch used); past
        # that the servo jams against its mechanical stop and buzzes/stalls.
        if self.joystick_buttons[4] or self.joystick_buttons[9]:
            self.motor_velocity["cam_servo"] = min(120, self.motor_velocity["cam_servo"] + 3)
        if self.joystick_buttons[5] or self.joystick_buttons[10]:
            self.motor_velocity["cam_servo"] = max(60, self.motor_velocity["cam_servo"] - 3)

        for i in range(min(n, 16)):
            self.previous_button_states[i] = self.joystick_buttons[i]

    def bar100_callback(self, msg):
        self.depth = msg.x   # x = depth in metres (BAR100)
        self._last_bar_time = time.time()

    # ── Leak → auto-surface failsafe ─────────────────────────────────────────
    def leak_callback(self, msg):
        self.leak_value = msg.data
        if msg.data >= 0.5:                          # water bridging the sensor
            self._leak_dry_since = None
            if not self.leak_failsafe:
                self._leak_count += 1
                if self._leak_count >= LEAK_TRIGGER_N:
                    self._trigger_leak_failsafe()
        else:                                        # dry
            self._leak_count = 0
            if self.leak_failsafe:                   # already surfacing — time the dry period
                if self._leak_dry_since is None:
                    self._leak_dry_since = time.time()
                elif time.time() - self._leak_dry_since >= LEAK_CLEAR_SEC:
                    self._clear_leak_failsafe()

    def _trigger_leak_failsafe(self):
        self.leak_failsafe = True
        self.auto_goto_active = False   # abandon any autonomous nav
        self.goto_target = None
        self._loitering = False
        self._leak_dry_since = None
        self.get_logger().error('LEAK DETECTED — auto-surfacing; all other control overridden')

    def _clear_leak_failsafe(self):
        self.leak_failsafe = False
        self._leak_count = 0
        self._leak_dry_since = None
        self.joystick_mode = True       # recover to manual, motors neutral
        self._stop_motors()
        self.get_logger().warn(f'Leak cleared (dry {LEAK_CLEAR_SEC:.0f}s) — manual control restored')

    def _auto_surface(self):
        """Emergency ascent: kill horizontal motion and drive the vertical
        thrusters up until at the surface (then hold). Board-side, link-independent.
        If depth is stale/unavailable it ascends anyway (fail toward the surface)."""
        for m in ('motor1', 'motor3', 'motor4', 'motor6'):
            self.motor_velocity[m] = 1500
        depth_fresh = (time.time() - self._last_bar_time) < 2.0
        at_surface = depth_fresh and self.depth <= LEAK_SURFACE_M
        vert = 0.0 if at_surface else LEAK_ASCEND   # +vert = ascend (same sign as depth hold)
        self.motor_velocity['motor2'] = max(1100, min(1900, 1500 + vert * 400))
        self.motor_velocity['motor5'] = max(1100, min(1900, 1500 + vert * 400))

    def qgc_mode_callback(self, msg):
        if msg.data == "STABILIZE" and self.joystick_mode:
            self.joystick_mode = False
            self.target_orientation = self.orientation.copy()
            self.get_logger().info("QGC: Switched to Stabilize mode")
        elif msg.data == "GUIDED" and self.joystick_mode:
            self.joystick_mode = False
            self.target_orientation = self.orientation.copy()
            self.get_logger().info("QGC: Guided mode (awaiting GOTO target)")
        elif msg.data == "MANUAL" and (not self.joystick_mode or self.auto_goto_active):
            self.joystick_mode = True
            self.auto_goto_active = False
            self.goto_target = None
            self.get_logger().info("QGC: Switched to Manual mode")

    def motor_light_publisher(self):
        if self.leak_failsafe:
            self._auto_surface()          # top priority — leak overrides every mode
        elif self.auto_goto_active:
            self.calculate_goto_motor_speeds()
        elif self.joystick_mode:
            if self.joy_cam_up != 1.0:
                self.motor_velocity["cam_servo"] = max(0, min(180, int((1 - self.joy_cam_up) * 45 + 90)))
            elif self.joy_cam_down != 1.0:
                self.motor_velocity["cam_servo"] = max(0, min(180, int((1 + self.joy_cam_down) * 45)))
            cam_servo_pwm = self.motor_velocity["cam_servo"]

            forward_backward = self.joystick_axes[1]
            left_right = self.joystick_axes[0]
            ascend_descend = self.joystick_axes[4]
            yaw = self.joystick_axes[3]

            # Failsafe: if the control link has gone silent (joystick unplugged,
            # QGC/acoustic link dropped), zero thruster inputs so the ROV stops
            # instead of coasting on the last command.
            if time.time() - self._last_joy_time > JOY_FAILSAFE_TIMEOUT:
                forward_backward = 0.0
                left_right = 0.0
                ascend_descend = 0.0
                yaw = 0.0

            scale_factor = 400

            motor1_speed = 1500 + (forward_backward * scale_factor * math.cos(math.radians(45)) +
                                   left_right * scale_factor * math.sin(math.radians(45)) +
                                   yaw * scale_factor)
            motor3_speed = 1500 + (forward_backward * scale_factor * math.cos(math.radians(45)) -
                                   left_right * scale_factor * math.sin(math.radians(45)) -
                                   yaw * scale_factor)
            motor6_speed = 1500 + (-forward_backward * scale_factor * math.cos(math.radians(45)) +
                                   left_right * scale_factor * math.sin(math.radians(45)) -
                                   yaw * scale_factor)
            motor4_speed = 1500 + (-forward_backward * scale_factor * math.cos(math.radians(45)) -
                                   left_right * scale_factor * math.sin(math.radians(45)) +
                                   yaw * scale_factor)
            motor2_speed = 1500 + (ascend_descend * scale_factor)
            motor5_speed = 1500 + (ascend_descend * scale_factor)

            self.motor_velocity["motor1"] = motor1_speed
            self.motor_velocity["motor2"] = motor2_speed
            self.motor_velocity["motor3"] = motor3_speed
            self.motor_velocity["motor4"] = motor4_speed
            self.motor_velocity["motor5"] = motor5_speed
            self.motor_velocity["motor6"] = motor6_speed
        else:
            self.calculate_autonomous_motor_speeds()

        if not self.leak_failsafe:
            self._apply_safety_envelope()   # leak auto-surface bypasses the envelope

        msg = Twist()
        msg.linear.x = float(self.motor_velocity["motor1"])
        msg.linear.y = float(self.motor_velocity["motor2"])
        msg.linear.z = float(self.motor_velocity["motor3"])
        msg.angular.x = float(self.motor_velocity["motor4"])
        msg.angular.y = float(self.motor_velocity["motor5"])
        msg.angular.z = float(self.motor_velocity["motor6"])
        self.motors_publisher.publish(msg)

        light_msg = Vector3()
        light_msg.x = float(self.light_intensity["light_single"])
        light_msg.y = float(self.light_intensity["light_couple"])
        light_msg.z = float(self.motor_velocity["cam_servo"])
        self.lights_publisher.publish(light_msg)

    def calculate_autonomous_motor_speeds(self):
        target_yaw_rad = math.radians(self.target_orientation["x"])
        target_pitch_rad = math.radians(self.target_orientation["y"])
        current_yaw_rad = math.radians(self.orientation["x"])
        current_pitch_rad = math.radians(self.orientation["y"])

        yaw_error = self.angle_wrap(target_yaw_rad - current_yaw_rad)
        pitch_error = self.angle_wrap(target_pitch_rad - current_pitch_rad)

        yaw_adjustment = int((400/(15*math.pi)) * self.kp_yaw * yaw_error)
        pitch_adjustment = int((400/(15*math.pi)) * self.kp_pitch * pitch_error)

        vx = self.dvl_velocities["vx"]*1000
        vy = self.dvl_velocities["vy"]*1000
        vz = self.dvl_velocities["vz"]*1000

        motor1_speed = 1500 + yaw_adjustment + (vx + vy) / math.sqrt(2)
        motor3_speed = 1500 + yaw_adjustment + (vx - vy) / math.sqrt(2)
        motor6_speed = 1500 - yaw_adjustment - (vx + vy) / math.sqrt(2)
        motor4_speed = 1500 - yaw_adjustment - (vx - vy) / math.sqrt(2)
        motor2_speed = 1500 + pitch_adjustment + vz
        motor5_speed = 1500 - pitch_adjustment + vz

        self.motor_velocity["motor1"] = max(1100, min(1900, motor1_speed))
        self.motor_velocity["motor2"] = max(1100, min(1900, motor2_speed))
        self.motor_velocity["motor3"] = max(1100, min(1900, motor3_speed))
        self.motor_velocity["motor4"] = max(1100, min(1900, motor4_speed))
        self.motor_velocity["motor5"] = max(1100, min(1900, motor5_speed))
        self.motor_velocity["motor6"] = max(1100, min(1900, motor6_speed))

    # ── Auto-GOTO (surface GPS navigation) ───────────────────────────────────

    def gps_callback(self, msg):
        self.gps["lat"] = msg.latitude
        self.gps["lon"] = msg.longitude
        # NavSatStatus.STATUS_NO_FIX = -1; >= 0 means we have a usable fix
        self.gps["fix"] = (msg.status.status >= 0) and (msg.latitude != 0.0 or msg.longitude != 0.0)

    def goto_callback(self, msg):
        # A valid target engages auto-GOTO; an invalid/no-fix target cancels it.
        if msg.status.status < 0 or (msg.latitude == 0.0 and msg.longitude == 0.0):
            if self.auto_goto_active:
                self.get_logger().info('GOTO cancelled')
            self.auto_goto_active = False
            self.goto_target = None
            self.joystick_mode = True   # return to manual (stopped), not orientation-hold
            return
        self.goto_target = {"lat": msg.latitude, "lon": msg.longitude}
        self.goto_depth = self.depth   # hold the depth we're at while navigating
        self.auto_goto_active = True
        self._loitering = False
        self.joystick_mode = False
        self.get_logger().info(
            f'GOTO engaged -> {msg.latitude:.7f}, {msg.longitude:.7f} (hold depth {self.goto_depth:.1f} m)')

    @staticmethod
    def _haversine_m(lat1, lon1, lat2, lon2):
        R = 6371000.0
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
        a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
        return 2.0 * R * math.asin(min(1.0, math.sqrt(a)))

    @staticmethod
    def _bearing_deg(lat1, lon1, lat2, lon2):
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dl = math.radians(lon2 - lon1)
        y = math.sin(dl) * math.cos(p2)
        x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
        return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

    def _stop_motors(self):
        for k in ("motor1", "motor2", "motor3", "motor4", "motor5", "motor6"):
            self.motor_velocity[k] = 1500

    def _end_goto(self, resume_manual=True):
        self.auto_goto_active = False
        self.goto_target = None
        self._loitering = False
        if resume_manual:
            self.joystick_mode = True
        self._stop_motors()

    def calculate_goto_motor_speeds(self):
        # Safety: lost the operator link while navigating -> stop
        if time.time() - self._last_joy_time > GOTO_COMMS_TIMEOUT:
            self.get_logger().warn('GOTO stop — operator link lost (no /joy)')
            self._end_goto()
            return
        tgt = self.goto_target
        if tgt is None or not self.gps["fix"]:
            self._stop_motors()  # no target / no GPS fix -> hold, don't drive
            return

        dist = self._haversine_m(self.gps["lat"], self.gps["lon"], tgt["lat"], tgt["lon"])
        if dist > GOTO_MAX_DIST:
            self.get_logger().warn(f'GOTO cancel — target {dist:.0f} m away exceeds {GOTO_MAX_DIST:.0f} m')
            self._end_goto()
            return

        # Loiter on arrival: hold the position instead of stopping/returning to
        # manual. Abort still available via joystick / QGC Manual / comms-loss.
        arrived = dist <= GOTO_ARRIVAL_M
        if arrived and not self._loitering:
            self._loitering = True
            self.get_logger().info(f'GOTO arrived ({dist:.1f} m) — loitering (holding position + depth)')
        elif not arrived and self._loitering:
            self._loitering = False
            self.get_logger().info('GOTO drifted out of the hold radius — resuming transit')

        bearing = self._bearing_deg(self.gps["lat"], self.gps["lon"], tgt["lat"], tgt["lon"])
        err = ((bearing - self.orientation["x"] + 180.0) % 360.0) - 180.0  # heading error [-180, 180]

        if dist <= GOTO_HOLD_DEADZONE_M:
            # essentially on the point — don't chase the noisy near-target bearing
            yaw = 0.0
            fwd = 0.0
        else:
            yaw = max(-1.0, min(1.0, err / GOTO_YAW_FULL))
            cap = GOTO_LOITER_FWD if self._loitering else GOTO_MAX_FWD
            if abs(err) < GOTO_ALIGN_DEG:
                # forward: capped, ramped down near target, reduced while off-heading
                fwd = cap * min(1.0, dist / GOTO_SLOWDOWN_M) * math.cos(math.radians(err))
                fwd = max(0.0, fwd)
            else:
                fwd = 0.0  # rotate in place until pointed at the target

        vert = self._goto_depth_cmd()   # depth hold on the vertical thrusters

        # Horizontal thruster mixing as in joystick mode (forward + yaw), plus
        # the vertical pair (motor2/5) driven by depth hold.
        s = 400.0
        c = math.cos(math.radians(45))
        self.motor_velocity["motor1"] = max(1100, min(1900, 1500 + (fwd * s * c + yaw * s)))
        self.motor_velocity["motor3"] = max(1100, min(1900, 1500 + (fwd * s * c - yaw * s)))
        self.motor_velocity["motor6"] = max(1100, min(1900, 1500 + (-fwd * s * c - yaw * s)))
        self.motor_velocity["motor4"] = max(1100, min(1900, 1500 + (-fwd * s * c + yaw * s)))
        self.motor_velocity["motor2"] = max(1100, min(1900, 1500 + vert * s))
        self.motor_velocity["motor5"] = max(1100, min(1900, 1500 + vert * s))

    def _goto_depth_cmd(self):
        """Vertical thrust (−1..1) to hold self.goto_depth. Depth is +down (BAR100).
        NOTE: verify the direction in water — if it drives the wrong way, flip the
        sign of GOTO_DEPTH_KP."""
        err_m = self.depth - self.goto_depth      # + = deeper than the target
        if abs(err_m) < GOTO_DEPTH_DEADBAND:
            return 0.0
        return max(-GOTO_VERT_MAX, min(GOTO_VERT_MAX, GOTO_DEPTH_KP * err_m))

    def _apply_safety_envelope(self):
        if not self.ENVELOPE_ENABLED:
            return
        pitch = self.orientation["y"] - self.PITCH_OFFSET
        roll  = self.orientation["z"] - self.ROLL_OFFSET

        pitch_abs = abs(pitch)
        roll_abs  = abs(roll)

        # Determine worst-axis violation
        max_abs = max(pitch_abs, roll_abs)
        active = max_abs > self.ENVELOPE_WARN

        if active and not self._envelope_active:
            self.get_logger().warn(
                f'Safety envelope active — pitch={pitch:.1f}° roll={roll:.1f}°'
            )
        elif not active and self._envelope_active:
            self.get_logger().info('Safety envelope cleared')
        self._envelope_active = active

        if not active:
            return

        zone = self.ENVELOPE_HARD - self.ENVELOPE_WARN
        # Scale factor 1.0 at warn threshold → 0.0 at hard limit
        scale = 1.0 - min(1.0, (max_abs - self.ENVELOPE_WARN) / zone)

        # Scale all horizontal motor deviations from neutral
        for key in ('motor1', 'motor3', 'motor4', 'motor6'):
            self.motor_velocity[key] = 1500 + (self.motor_velocity[key] - 1500) * scale

        # Pitch correction via vertical thruster differential (motor2 vs motor5)
        if pitch_abs > self.ENVELOPE_WARN:
            correction = (
                -math.copysign(1.0, pitch)
                * min(pitch_abs - self.ENVELOPE_WARN, zone)
                * self.ENVELOPE_GAIN
            )
            self.motor_velocity["motor2"] = (
                1500 + (self.motor_velocity["motor2"] - 1500) * scale + correction
            )
            self.motor_velocity["motor5"] = (
                1500 + (self.motor_velocity["motor5"] - 1500) * scale - correction
            )
        else:
            for key in ('motor2', 'motor5'):
                self.motor_velocity[key] = 1500 + (self.motor_velocity[key] - 1500) * scale

        # Clamp all outputs to valid PWM range
        for key in ('motor1', 'motor2', 'motor3', 'motor4', 'motor5', 'motor6'):
            self.motor_velocity[key] = max(1100, min(1900, int(self.motor_velocity[key])))

    def angle_wrap(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    guidance_node = GuidanceNode()
    rclpy.spin(guidance_node)
    guidance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
