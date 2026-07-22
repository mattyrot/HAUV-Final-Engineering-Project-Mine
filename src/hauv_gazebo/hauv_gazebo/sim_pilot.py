#!/usr/bin/env python3
"""Fly guidance_node in the simulation from a script, and measure what happens.

There is no joystick in WSL and no way to click QGroundControl from a test, so
this node stands in for both: it publishes /joy exactly as joy_node would,
injects /guidance/goto_target exactly as mavlink_bridge_node does when you click
"Go to location", and injects /esp32/leak. At the same time it records the
vehicle's true state from Gazebo, so every claim about a run is a number rather
than an impression.

    ros2 run hauv_gazebo sim_pilot --ros-args -p scenario:=goto

THE /joy STREAM IS NOT OPTIONAL - it is the whole reason this node exists.
guidance_node starts with `_last_joy_time = 0.0`, so both of its control-link
failsafes are already tripped before the first message arrives:
JOY_FAILSAFE_TIMEOUT (1.5 s) zeroes manual thrust, and GOTO_COMMS_TIMEOUT (5 s)
*aborts auto-GOTO outright*. A GOTO test run without a joystick publisher
engages, drives for five seconds and quits - which looks exactly like a broken
navigator and is not. So this node publishes /joy at 20 Hz through every
scenario, neutral unless the current phase says otherwise.

Scenarios (`scenario` parameter):

    manual      stick -> /joy -> guidance -> /motor_data -> the sim moves.
                Forward, yaw, strafe and descend in turn, checking each acts on
                the axis it should and in the direction it should.
    depth_hold  Engage GOTO on the spot (inside GOTO_HOLD_DEADZONE_M, so
                horizontal thrust is zero by construction) and hold depth
                against the +0.1 kg buoyant trim. Must be run deep - see below.
    goto        Turn to a target ~29 m astern, transit, ramp down, arrive,
                loiter, then abort with the stick.
    leak        Inject /esp32/leak=1.0 and watch the auto-surface, then clear it.

Depth-dependent scenarios must spawn deep. At the default spawn (z=-0.15) the
vehicle floats up against the surface slab and parks at depth ~0.066 m; with
GOTO_DEPTH_DEADBAND at 0.3 m the depth error never leaves the deadband and depth
hold is a no-op that trivially "passes". run_guidance_test.sh spawns depth_hold
at -6 m and leak at -8 m for this reason.

Each run prints a per-phase table, then PASS/FAIL lines, and writes the whole
sample series to CSV so a surprising result can be re-read rather than re-run.
"""

import csv
import math
import os
import time
from collections import namedtuple

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from sensor_msgs.msg import Joy, NavSatFix
from std_msgs.msg import Float64

try:
    from rosgraph_msgs.msg import Clock
except ImportError:      # /clock is a nicety - real-time factor only
    Clock = None

MODEL_NAME = 'hauv'
JOY_HZ = 20.0
SAMPLE_HZ = 10.0

# Mirrored from sensor_bridge. GOTO targets below are built as metre offsets
# from the live fix using exactly this scale, so if it changes there it must
# change here or the target lands somewhere other than where the test thinks.
M_PER_DEG_LAT = 111320.0

# Mirrored from guidance_node, for the verdicts only.
GOTO_ARRIVAL_M = 4.0
GOTO_HOLD_DEADZONE_M = 1.5
GOTO_DEPTH_DEADBAND = 0.3
LEAK_SURFACE_M = 0.4
LEAK_ASCEND_PWM = 1820.0       # 1500 + LEAK_ASCEND (0.8) * 400
NEUTRAL = 1500.0

# Xbox-style rest position: sticks centred, BOTH TRIGGERS RELEASED AT +1.0.
# guidance_node reads axes[2]/axes[5] as the camera tilt and treats any value
# != 1.0 as a command, so publishing 0.0 there would drive the camera servo on
# every single message. A real joystick rests at 1.0; so does this.
NEUTRAL_AXES = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]

# guidance_node's joystick axis map, for readability below.
AX_STRAFE, AX_FWD, AX_YAW, AX_VERT = 0, 1, 3, 4

Phase = namedtuple('Phase', 'name secs axes act until')


def P(name, secs, axes=None, act=None, until=None):
    """A scripted phase: hold these axes for this long, maybe do something once."""
    return Phase(name, secs, axes or {}, act, until)


def _arrived(s):
    d = s.dist_to_target()
    return d is not None and d < GOTO_ARRIVAL_M


# Every scenario opens with a settle phase: the vehicle is dropped in and needs
# to stop moving before any measurement means anything. It also lets
# guidance_node's IMU offset calibration (first 20 readings) average a vehicle
# that is level and still rather than one still settling from the spawn.
SETTLE = P('settle', 8.0)

SCENARIOS = {
    'manual': [
        SETTLE,
        P('forward', 10.0, {AX_FWD: +1.0}),
        P('coast-1', 6.0),
        P('yaw-left', 10.0, {AX_YAW: +1.0}),
        P('coast-2', 6.0),
        P('strafe-left', 10.0, {AX_STRAFE: +1.0}),
        P('coast-3', 6.0),
        P('descend', 10.0, {AX_VERT: -1.0}),
        P('coast-4', 6.0),
        # Two axes at once. guidance_node's joystick branch assigns
        # motor_velocity WITHOUT clamping to 1100-1900 (unlike its autonomous
        # and GOTO branches, which both clamp), so forward+yaw together should
        # emit 1500 + 400*cos45 + 400 = 2183 on motor1. No single-axis phase can
        # show this: alone, each command tops out at exactly 1900.
        P('fwd+yaw', 8.0, {AX_FWD: +1.0, AX_YAW: +1.0}),
    ],
    'depth_hold': [
        SETTLE,
        P('engage', 0.2, act='goto_here'),
        P('hold', 90.0),
    ],
    'goto': [
        SETTLE,
        P('engage', 0.2, act='goto_far'),
        # Ends as soon as it arrives, so the phase length measures the transit
        # instead of capping it. The 240 s is only a backstop.
        P('transit', 240.0, until=_arrived),
        P('loiter', 60.0),
        P('abort-stick', 3.0, {AX_FWD: +1.0}),   # > GOTO_ABORT_AXIS (0.2)
        P('after-abort', 6.0),
    ],
    'leak': [
        SETTLE,
        P('leak-on', 30.0, act='leak_on'),
        P('leak-off', 12.0, act='leak_off'),     # > LEAK_CLEAR_SEC (5 s)
    ],
}

# Metre offsets (north, east) for the goto scenario's target. Deliberately
# BEHIND the vehicle - it spawns heading ~0 (north), so a target at bearing
# ~211 deg forces a real turn-in-place first (heading error > GOTO_ALIGN_DEG),
# which makes "turn, then drive" visible as two distinct behaviours rather than
# one blurred one.
GOTO_OFFSET_N = -25.0
GOTO_OFFSET_E = -15.0


def quat_to_euler(x, y, z, w):
    """Quaternion -> (roll, pitch, yaw) radians. Same convention as sensor_bridge."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def haversine_m(lat1, lon1, lat2, lon2):
    """Identical to guidance_node._haversine_m, so distances agree with its logic."""
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2.0 * r * math.asin(min(1.0, math.sqrt(a)))


class SimPilot(Node):

    def __init__(self):
        super().__init__('sim_pilot')

        self.declare_parameter('scenario', 'manual')
        self.declare_parameter('out_dir', '/tmp/hauv_sim_results')
        self.scenario = str(self.get_parameter('scenario').value)
        self.out_dir = str(self.get_parameter('out_dir').value)

        if self.scenario not in SCENARIOS:
            raise SystemExit(f'unknown scenario {self.scenario!r}; '
                             f'choose from {sorted(SCENARIOS)}')
        self.phases = SCENARIOS[self.scenario]

        # --- what we drive -------------------------------------------------
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        self.goto_pub = self.create_publisher(NavSatFix, '/guidance/goto_target', 10)
        self.leak_pub = self.create_publisher(Float64, '/esp32/leak', 10)

        # --- what we watch -------------------------------------------------
        self.create_subscription(ModelStates, '/gazebo/model_states', self.on_states, 10)
        self.create_subscription(Twist, '/motor_data', self.on_motor, 10)
        self.create_subscription(Vector3, '/esp32/bar100_data', self.on_bar, 10)
        self.create_subscription(Twist, '/esp32/bno055_data', self.on_imu, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.on_fix, 10)
        if Clock is not None:
            self.create_subscription(Clock, '/clock', self.on_clock, 10)

        # Ground truth from Gazebo
        self.pose = None            # x, y, z
        self.att = (0.0, 0.0, 0.0)  # roll, pitch, yaw (rad)
        self.vel = (0.0, 0.0, 0.0)  # world-frame linear
        self.yaw_rate = 0.0
        # What guidance sees
        self.depth = 0.0
        self.heading = 0.0
        self.fix = None
        self.motors = [0.0] * 6
        self.have_motor = False     # don't log the initialiser as if it were data

        self.target = None          # (lat, lon) once a GOTO is issued
        self.leak_cmd = 0.0
        self.axes = list(NEUTRAL_AXES)

        self.sim_t0 = None
        self.sim_t = None
        self.rows = []
        self.done = False

        self.t_start = time.time()
        self.phase_i = 0
        self.phase_t0 = self.t_start

        self.create_timer(1.0 / JOY_HZ, self.tick_joy)
        self.create_timer(1.0 / SAMPLE_HZ, self.tick_sample)

        self.get_logger().info(f'scenario={self.scenario}  '
                               f'phases={[p.name for p in self.phases]}')
        self._banner(self.phases[0])

    # ── inputs ───────────────────────────────────────────────────────────────

    def on_states(self, msg):
        try:
            i = msg.name.index(MODEL_NAME)
        except ValueError:
            return
        p = msg.pose[i].position
        q = msg.pose[i].orientation
        t = msg.twist[i]
        self.pose = (p.x, p.y, p.z)
        self.att = quat_to_euler(q.x, q.y, q.z, q.w)
        self.vel = (t.linear.x, t.linear.y, t.linear.z)
        self.yaw_rate = t.angular.z

    def on_motor(self, msg):
        self.motors = [msg.linear.x, msg.linear.y, msg.linear.z,
                       msg.angular.x, msg.angular.y, msg.angular.z]
        self.have_motor = True

    def on_bar(self, msg):
        self.depth = msg.x

    def on_imu(self, msg):
        self.heading = msg.linear.x

    def on_fix(self, msg):
        self.fix = (msg.latitude, msg.longitude)

    def on_clock(self, msg):
        self.sim_t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.sim_t0 is None:
            self.sim_t0 = self.sim_t

    # ── outputs ──────────────────────────────────────────────────────────────

    def tick_joy(self):
        """The heartbeat guidance_node's two failsafes are watching for."""
        j = Joy()
        j.header.stamp = self.get_clock().now().to_msg()
        j.axes = [float(v) for v in self.axes]
        j.buttons = [0] * 16          # no mode toggles, no lights, no camera
        self.joy_pub.publish(j)

        if self.scenario == 'leak':
            # sensor_bridge is launched with publish_leak:=false for this run,
            # so the pilot owns the topic outright. Publishing alongside it
            # would race: the failsafe needs LEAK_TRIGGER_N consecutive wet
            # samples, and an interleaved dry stream resets the count.
            m = Float64()
            m.data = self.leak_cmd
            self.leak_pub.publish(m)

    def do_goto_here(self):
        """Target the vehicle's own position: inside GOTO_HOLD_DEADZONE_M, so
        guidance commands zero yaw and zero forward and only depth hold runs."""
        if self.fix is None:
            self.get_logger().error('no /gps/fix yet - cannot engage GOTO')
            return
        self._send_goto(self.fix[0], self.fix[1])

    def do_goto_far(self):
        if self.fix is None:
            self.get_logger().error('no /gps/fix yet - cannot engage GOTO')
            return
        lat, lon = self.fix
        lon_scale = M_PER_DEG_LAT * math.cos(math.radians(lat))
        self._send_goto(lat + GOTO_OFFSET_N / M_PER_DEG_LAT,
                        lon + GOTO_OFFSET_E / lon_scale)

    def _send_goto(self, lat, lon):
        m = NavSatFix()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'gps'
        m.status.status = 0           # STATUS_FIX - anything < 0 cancels
        m.status.service = 1
        m.latitude, m.longitude = lat, lon
        self.goto_pub.publish(m)
        self.target = (lat, lon)
        d = self.dist_to_target()
        self.get_logger().info(f'GOTO issued -> {lat:.7f}, {lon:.7f} '
                               f'({d:.1f} m away, depth now {self.depth:.2f} m)')

    def do_leak_on(self):
        self.leak_cmd = 1.0
        self.get_logger().warn('injecting /esp32/leak = 1.0')

    def do_leak_off(self):
        self.leak_cmd = 0.0
        self.get_logger().info('/esp32/leak back to 0.0 - expecting auto-clear')

    # ── the script ───────────────────────────────────────────────────────────

    def dist_to_target(self):
        if self.target is None or self.fix is None:
            return None
        return haversine_m(self.fix[0], self.fix[1], self.target[0], self.target[1])

    def _banner(self, ph):
        extra = f'  axes={ph.axes}' if ph.axes else ''
        self.get_logger().info(f'--- phase {ph.name} ({ph.secs:.0f}s){extra}')

    def tick_sample(self):
        now = time.time()
        ph = self.phases[self.phase_i]

        expired = (now - self.phase_t0) >= ph.secs
        early = ph.until is not None and ph.until(self)
        if expired or early:
            if early:
                self.get_logger().info(
                    f'    phase {ph.name} ended early at '
                    f'{now - self.phase_t0:.1f}s (condition met)')
            self.phase_i += 1
            if self.phase_i >= len(self.phases):
                self.done = True
                return
            ph = self.phases[self.phase_i]
            self.phase_t0 = now
            self._banner(ph)
            if ph.act:
                getattr(self, 'do_' + ph.act)()

        self.axes = list(NEUTRAL_AXES)
        for i, v in ph.axes.items():
            self.axes[i] = v

        if self.pose is None or not self.have_motor:
            return
        roll, pitch, yaw = self.att
        vx, vy, vz = self.vel
        c, s = math.cos(yaw), math.sin(yaw)
        self.rows.append({
            't': round(now - self.t_start, 2),
            'phase': ph.name,
            'x': self.pose[0], 'y': self.pose[1], 'z': self.pose[2],
            'depth': self.depth,
            'roll': math.degrees(roll), 'pitch': math.degrees(pitch),
            'yaw': math.degrees(yaw), 'heading': self.heading,
            'vx': vx, 'vy': vy, 'vz': vz,
            # body frame: surge is along the nose, sway to port
            'surge': vx * c + vy * s,
            'sway': -vx * s + vy * c,
            'yawrate': math.degrees(self.yaw_rate),
            'm1': self.motors[0], 'm2': self.motors[1], 'm3': self.motors[2],
            'm4': self.motors[3], 'm5': self.motors[4], 'm6': self.motors[5],
            'dist': self.dist_to_target() or 0.0,
        })

    # ── reporting ────────────────────────────────────────────────────────────

    def rows_of(self, name):
        return [r for r in self.rows if r['phase'] == name]

    @staticmethod
    def mean(rows, key, tail=0.5):
        """Mean over the last `tail` fraction of a phase - skips the transient
        while the vehicle accelerates into the commanded state."""
        if not rows:
            return 0.0
        sub = rows[int(len(rows) * (1.0 - tail)):] or rows
        return sum(r[key] for r in sub) / len(sub)

    def write_csv(self):
        os.makedirs(self.out_dir, exist_ok=True)
        path = os.path.join(self.out_dir, f'{self.scenario}.csv')
        if self.rows:
            with open(path, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(self.rows[0]))
                w.writeheader()
                w.writerows(self.rows)
        return path

    def report(self):
        wall = time.time() - self.t_start
        print('\n' + '=' * 78)
        print(f'SCENARIO: {self.scenario}    samples={len(self.rows)}  '
              f'wall={wall:.0f}s')
        if self.sim_t is not None and self.sim_t0 is not None:
            rtf = (self.sim_t - self.sim_t0) / wall if wall > 0 else 0.0
            print(f'real-time factor {rtf:.2f}  '
                  f'(sim advanced {self.sim_t - self.sim_t0:.0f}s in {wall:.0f}s wall)')
            if rtf < 0.8:
                print('  NOTE: RTF < 0.8. guidance_node times its failsafes on the '
                      'WALL clock,\n        so its timeouts and the vehicle\'s motion '
                      'run at different rates.')
        print('=' * 78)

        print(f'{"phase":<14}{"secs":>6}{"surge":>8}{"sway":>8}{"heave":>8}'
              f'{"yawrate":>9}{"depth":>8}{"pitch":>8}{"dist":>8}')
        for ph in self.phases:
            rows = self.rows_of(ph.name)
            if not rows:
                continue
            print(f'{ph.name:<14}{rows[-1]["t"] - rows[0]["t"]:>6.1f}'
                  f'{self.mean(rows, "surge"):>8.2f}{self.mean(rows, "sway"):>8.2f}'
                  f'{self.mean(rows, "vz"):>8.2f}{self.mean(rows, "yawrate"):>9.1f}'
                  f'{rows[-1]["depth"]:>8.2f}{self.mean(rows, "pitch"):>8.1f}'
                  f'{rows[-1]["dist"]:>8.1f}')

        print('-' * 78)
        checks = getattr(self, 'check_' + self.scenario)()
        for ok, text in checks:
            print(f'  [{"PASS" if ok else "FAIL"}] {text}')
        n_bad = sum(1 for ok, _ in checks if not ok)
        print('-' * 78)
        print(f'{len(checks) - n_bad}/{len(checks)} checks passed')
        print(f'csv: {self.write_csv()}')
        print('=' * 78 + '\n')
        return n_bad == 0

    # Each check is (bool, human-readable claim including the measured number),
    # so a FAIL line says what actually happened, not just that it failed.

    def check_manual(self):
        fwd = self.rows_of('forward')
        yawp = self.rows_of('yaw-left')
        stf = self.rows_of('strafe-left')
        dsc = self.rows_of('descend')
        out = []

        v = self.mean(fwd, 'surge')
        out.append((v > 0.5, f'forward stick drives forward: surge {v:+.2f} m/s '
                             f'(expect > +0.5, README measured 0.97 at full)'))
        cross = abs(self.mean(fwd, 'sway'))
        out.append((cross < 0.25, f'forward is not a strafe: |sway| {cross:.2f} m/s '
                                  f'(expect < 0.25)'))

        r = self.mean(yawp, 'yawrate')
        out.append((r > 8.0, f'yaw stick rotates counter-clockwise (heading up): '
                             f'{r:+.1f} deg/s (expect > +8, README measured ~26)'))
        spd = math.hypot(self.mean(yawp, 'surge'), self.mean(yawp, 'sway'))
        out.append((spd < 0.35, f'yaw is a rotation, not a strafe: speed {spd:.2f} m/s '
                                f'(expect < 0.35 - this is bug #5, watch it)'))

        v = self.mean(stf, 'sway')
        out.append((v > 0.3, f'strafe stick drives to port: sway {v:+.2f} m/s '
                             f'(expect > +0.3)'))

        if dsc:
            dz = dsc[-1]['depth'] - dsc[0]['depth']
            out.append((dz > 1.0, f'descend stick increases depth: {dz:+.2f} m '
                                  f'over the phase (expect > +1.0)'))

        # Reported as an observation about guidance_node, not a sim requirement.
        # thruster_bridge clamps its input, so the sim is unaffected either way -
        # but the topic is the same one the ESP32 reads on the real vehicle.
        both = self.rows_of('fwd+yaw')
        if both:
            hi = max(r[f'm{i}'] for r in both for i in range(1, 7))
            out.append((hi <= 1900.0,
                        f'/motor_data stays inside 1100-1900 on a two-axis stick: '
                        f'peak {hi:.0f} (guidance_node\'s joystick branch does not '
                        f'clamp; its GOTO and autonomous branches do)'))
        return out

    def check_depth_hold(self):
        hold = self.rows_of('hold')
        out = []
        if not hold:
            return [(False, 'no hold phase recorded')]

        d0 = hold[0]['depth']
        dmin = min(r['depth'] for r in hold)
        dmax = max(r['depth'] for r in hold)
        worst = max(abs(dmax - d0), abs(dmin - d0))
        secs = hold[-1]['t'] - hold[0]['t']
        # Uncommanded, +0.1 kg of trim lifts the vehicle at ~1 cm/s.
        drift = 0.01 * secs

        out.append((worst < 0.6,
                    f'depth held within {worst:.2f} m of {d0:.2f} m over {secs:.0f}s '
                    f'(expect < 0.6; deadband alone is {GOTO_DEPTH_DEADBAND})'))
        out.append((worst < drift,
                    f'better than doing nothing: {worst:.2f} m excursion vs '
                    f'{drift:.2f} m of uncontrolled buoyant rise'))
        out.append((hold[-1]['depth'] > d0 - 0.6,
                    f'did not simply float away: ended at {hold[-1]["depth"]:.2f} m'))
        # Sign check: if GOTO_DEPTH_KP had the wrong sign the error would run
        # away rather than being arrested, so this is the sign test.
        out.append((dmin > d0 - 1.0 and dmax < d0 + 1.0,
                    f'GOTO_DEPTH_KP sign correct - error arrested, not amplified '
                    f'(range {dmin:.2f} to {dmax:.2f} m)'))
        vert = self.mean(hold, 'm2')
        out.append((abs(vert - NEUTRAL) > 1.0,
                    f'vertical thrusters actually worked for it: mean motor2 '
                    f'{vert:.0f} (1500 would mean it never left the deadband)'))
        return out

    def check_goto(self):
        tr = self.rows_of('transit')
        lo = self.rows_of('loiter')
        ab = self.rows_of('after-abort')
        out = []
        if not tr:
            return [(False, 'no transit phase recorded')]

        # Closest approach over transit AND loiter, not the last row of transit:
        # the transit phase ends the instant distance drops under GOTO_ARRIVAL_M,
        # so testing that same row against that same threshold is a coin flip on
        # the rounding. What we actually want to know is whether it got there.
        d0 = tr[0]['dist']
        dmin = min(r['dist'] for r in (tr + lo))
        secs = tr[-1]['t'] - tr[0]['t']
        out.append((dmin <= GOTO_ARRIVAL_M,
                    f'reached the target: {d0:.1f} m -> closest {dmin:.2f} m, '
                    f'transit took {secs:.0f}s (arrival radius {GOTO_ARRIVAL_M})'))

        # Did it turn before it drove? Heading error starts ~-149 deg, so the
        # first thing that should happen is rotation with no forward thrust.
        first = tr[:min(len(tr), 60)]      # first ~6 s
        turned = max(abs(r['yawrate']) for r in first) if first else 0.0
        out.append((turned > 8.0,
                    f'turned toward the bearing first: peak {turned:.1f} deg/s '
                    f'in the first seconds (target was ~211 deg, heading ~0)'))

        backtrack = 0.0
        best = d0
        for r in tr:
            best = min(best, r['dist'])
            backtrack = max(backtrack, r['dist'] - best)
        out.append((backtrack < 3.0,
                    f'closed in without wandering: worst backtrack {backtrack:.1f} m '
                    f'(expect < 3.0)'))

        dep = [r['depth'] for r in tr]
        swing = max(dep) - min(dep)
        out.append((swing < 1.5,
                    f'held depth through the transit: {swing:.2f} m total swing '
                    f'({min(dep):.2f} to {max(dep):.2f} m)'))

        if lo:
            far = max(r['dist'] for r in lo)
            out.append((far < GOTO_ARRIVAL_M + 2.0,
                        f'loitered on station: never drifted past {far:.1f} m '
                        f'over {lo[-1]["t"] - lo[0]["t"]:.0f}s'))
        if ab:
            worst = max(abs(r[f'm{i}'] - NEUTRAL) for r in ab[-10:] for i in range(1, 7))
            out.append((worst < 2.0,
                        f'stick abort returned to manual and stopped: max motor '
                        f'deviation {worst:.0f} PWM from neutral'))
        return out

    def check_leak(self):
        on = self.rows_of('leak-on')
        off = self.rows_of('leak-off')
        out = []
        if not on:
            return [(False, 'no leak-on phase recorded')]

        d0 = on[0]['depth']
        # The ascent, before it reaches the surface and stops.
        asc = [r for r in on if r['depth'] > LEAK_SURFACE_M + 0.2]
        if asc:
            horiz = max(abs(r[f'm{i}'] - NEUTRAL) for r in asc for i in (1, 3, 4, 6))
            out.append((horiz < 1.0,
                        f'horizontal thrusters killed: max deviation {horiz:.0f} PWM '
                        f'(expect exactly 1500 on motors 1/3/4/6)'))
            vert = max(r['m2'] for r in asc)
            out.append((abs(vert - LEAK_ASCEND_PWM) < 2.0,
                        f'vertical thrusters at full emergency ascent: motor2 peaked '
                        f'{vert:.0f} (expect {LEAK_ASCEND_PWM:.0f})'))

        dmin = min(r['depth'] for r in on)
        out.append((dmin < d0 - 1.0,
                    f'LEAK_ASCEND sign correct - it surfaced rather than diving: '
                    f'{d0:.2f} m -> {dmin:.2f} m'))
        out.append((dmin <= LEAK_SURFACE_M + 0.3,
                    f'reached the surface: shallowest {dmin:.2f} m '
                    f'(target <= {LEAK_SURFACE_M})'))

        surf = [r for r in on if r['depth'] <= LEAK_SURFACE_M]
        if surf:
            held = max(abs(r['m2'] - NEUTRAL) for r in surf[-10:])
            out.append((held < 2.0,
                        f'stopped thrusting at the surface: motor2 within '
                        f'{held:.0f} PWM of neutral once shallow'))
        else:
            out.append((False, f'never got shallower than {LEAK_SURFACE_M} m'))

        if off:
            worst = max(abs(r[f'm{i}'] - NEUTRAL) for r in off[-10:] for i in range(1, 7))
            out.append((worst < 2.0,
                        f'auto-cleared to manual after drying out: max motor '
                        f'deviation {worst:.0f} PWM'))
        return out


def main():
    rclpy.init()
    node = SimPilot()
    ok = False
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        ok = node.report()
    except KeyboardInterrupt:
        node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
