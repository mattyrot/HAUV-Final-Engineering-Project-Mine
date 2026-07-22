"""
acoustic_qgc_bridge.py — PC (topside) bridge: QGroundControl <-> Subsonus acoustic link.

QGC connects here over UDP MAVLink. This bridge:
  - Translates QGC MANUAL_CONTROL  -> 8-byte command packet  -> Subsonus (down to ROV)
  - Translates 32-byte telemetry packet (up from ROV) -> synthesized MAVLink -> QGC

The heavy MAVLink stays local (QGC <-> this bridge, over localhost). Only the
compact 8/32-byte packets ever cross the acoustic link, so it fits the bandwidth.

Set DIRECT_TCP_TEST=True to bench-test over Ethernet (UP Board bridge in server
mode) with no Subsonus/water. Set False for the real acoustic link.

Requires:  pip install pymavlink
Run:       python tools/acoustic_qgc_bridge.py
QGC:       add a UDP comm link to 127.0.0.1:14550 (or let auto-connect find it).
"""

import socket
import struct
import threading
import time
import io
import math

from pymavlink.dialects.v20 import ardupilotmega as mavlink2

# ── link mode ────────────────────────────────────────────────────────────────
DIRECT_TCP_TEST = True
if DIRECT_TCP_TEST:
    LINK_IP   = '192.168.168.101'   # UP Board bench server
    LINK_PORT = 17000
else:
    LINK_IP   = '168.254.1.80'      # PC's Subsonus transparent modem port
    LINK_PORT = 16740

# QGC listens on 14550 (auto-connect). We bind our own port and SEND to QGC's
# 14550; QGC auto-connects and replies to us. Do NOT bind 14550 — QGC owns it.
QGC_TARGET       = ('127.0.0.1', 14550)
MAVLINK_BIND_PORT = 14551           # our local port; QGC replies here
RECONNECT_S      = 3.0
CMD_HZ           = 8                # command packets down to the ROV
HEARTBEAT_HZ     = 1
TELEM_TX_HZ      = 4                # synthesized MAVLink up to QGC
FAILSAFE_TIMEOUT = 1.5              # s without MANUAL_CONTROL -> command neutral
LINK_TIMEOUT     = 3.0              # s without telemetry -> warn QGC link is lost

# ── packet formats (must match acoustic_bridge_node.py) ──────────────────────
CMD_MAGIC = 0xAC
TEL_MAGIC = 0xAB
CMD_FMT   = '!BbbbbbH'
CMD_SIZE  = struct.calcsize(CMD_FMT)   # 8
TEL_FMT   = '!BhhhHhhhHhiiHBH'
TEL_SIZE  = struct.calcsize(TEL_FMT)   # 32
GOTO_MAGIC = 0xAD
GOTO_FMT   = '!Bii'                      # magic(B) lat(i) lon(i), each deg*1e7
GOTO_SIZE  = struct.calcsize(GOTO_FMT)   # 9

HEALTH_ORDER = ['bno055', 'bar100', 'bme280', 'leak',
                'dvl', 'subsonus_vel', 'subsonus_ori', 'gps']


def _clamp_i8(v):
    return max(-128, min(127, int(v)))


class AcousticQGCBridge:
    def __init__(self):
        # Telemetry state (decoded from the acoustic packet)
        self.t = {
            'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0, 'depth': 0.0,
            'dvl_e': 0.0, 'dvl_n': 0.0, 'dvl_u': 0.0, 'dvl_range': 0.0,
            'temp': 0.0, 'lat': 0.0, 'lon': 0.0,
            'pressure': 0, 'leak': 0, 'health': 0,
        }
        self.have_telem = False
        self.prev_leak = 0
        self._leak_alert_t = 0.0    # last leak STATUSTEXT time (for repeat while active)
        self.prev_health = 0xFFFF
        self._last_tel_print = 0.0
        self.last_telem_time = 0.0    # last telemetry packet from the ROV
        self.link_lost = False

        # Latest command from QGC (forwarded to ROV at CMD_HZ)
        self.cmd = {'fwd': 0.0, 'strafe': 0.0, 'yaw': 0.0, 'vert': 0.0,
                    'cam': 0.0, 'buttons': 0}
        self.last_mc_time = 0.0   # last MANUAL_CONTROL from QGC (for failsafe)
        self.armed = True  # boot armed so QGC shows "ready" (matches the tethered bridge)
        self.custom_mode = 19  # ArduSub MANUAL

        # MAVLink encoder (to QGC)
        self.mav_out = io.BytesIO()
        self.mav = mavlink2.MAVLink(self.mav_out)
        self.mav.srcSystem = 1     # HAUV vehicle (same as the tethered bridge)
        self.mav.srcComponent = 1

        # QGC UDP socket — bind our own port, send to QGC's 14550.
        # No SO_REUSEADDR on purpose: if a second bridge is already running, the
        # bind fails loudly rather than silently double-sending to QGC (which makes
        # telemetry flicker between the two instances' data).
        self.qgc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.qgc_sock.bind(('0.0.0.0', MAVLINK_BIND_PORT))
        except OSError as e:
            raise SystemExit(
                f'\nCannot bind UDP {MAVLINK_BIND_PORT}: {e}\n'
                f'Another acoustic_qgc_bridge is probably already running — '
                f'only run ONE instance.\n')
        self.qgc_sock.settimeout(0.1)
        self.qgc_addr = QGC_TARGET   # send target; refined once QGC replies

        # Link (Subsonus/bench) socket
        self._link = None
        self._link_lock = threading.Lock()

        print(f'QGC bridge — link mode: '
              f'{"DIRECT TCP " + LINK_IP if DIRECT_TCP_TEST else "ACOUSTIC " + LINK_IP}:{LINK_PORT}')
        print(f'Sending MAVLink to QGC at {QGC_TARGET[0]}:{QGC_TARGET[1]}, '
              f'listening on {MAVLINK_BIND_PORT}')

    # ── MAVLink send helper ──────────────────────────────────────────────────

    def _send_qgc(self, packet):
        if self.qgc_addr and packet:
            try:
                self.qgc_sock.sendto(packet, self.qgc_addr)
            except Exception as e:
                print(f"Error sending to QGC: {e}")

    def _emit(self, build):
        """Encode one MAVLink message via the BytesIO buffer and send to QGC."""
        self.mav_out.seek(0)
        self.mav_out.truncate(0)
        try:
            build()
            self._send_qgc(self.mav_out.getvalue())
        except Exception as e:
            print(f"Error building MAVLink message: {e}")

    # ── link (Subsonus/bench) send/recv ──────────────────────────────────────

    def _send_command_packet(self):
        with self._link_lock:
            link = self._link
        if link is None:
            return
        # Failsafe: if QGC control has gone silent, command neutral (stop) rather
        # than repeating the last stick input forever.
        if time.time() - self.last_mc_time > FAILSAFE_TIMEOUT:
            c = {'fwd': 0.0, 'strafe': 0.0, 'yaw': 0.0, 'vert': 0.0,
                 'cam': 0.0, 'buttons': 0}
        else:
            c = self.cmd
        try:
            pkt = struct.pack(
                CMD_FMT, CMD_MAGIC,
                _clamp_i8(c['fwd'] * 100), _clamp_i8(c['strafe'] * 100),
                _clamp_i8(c['yaw'] * 100), _clamp_i8(c['vert'] * 100),
                _clamp_i8(c['cam'] * 100), int(c['buttons']) & 0xFFFF,
            )
            link.sendall(pkt)
        except Exception as e:
            print(f"Send command error: {e}")
            with self._link_lock:
                if self._link is link:
                    self._link = None
            try:
                link.close()
            except Exception:
                pass

    def _link_loop(self):
        while True:
            sock = None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect((LINK_IP, LINK_PORT))
                sock.settimeout(1.0)
                with self._link_lock:
                    self._link = sock
                print(f'Link connected: {LINK_IP}:{LINK_PORT}')
                self._recv_telemetry(sock)
            except Exception as e:
                print(f'Link error: {e} — retry in {RECONNECT_S}s')
            with self._link_lock:
                if self._link is sock:
                    self._link = None
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass
            time.sleep(RECONNECT_S)

    def _recv_telemetry(self, sock):
        buf = b''
        while True:
            with self._link_lock:
                if self._link is not sock:
                    break
            try:
                chunk = sock.recv(256)
                if not chunk:
                    break
                buf += chunk
                while len(buf) >= TEL_SIZE:
                    idx = buf.find(bytes([TEL_MAGIC]))
                    if idx < 0:
                        buf = b''
                        break
                    if idx > 0:
                        buf = buf[idx:]
                    if len(buf) < TEL_SIZE:
                        break
                    self._decode_telemetry(buf[:TEL_SIZE])
                    buf = buf[TEL_SIZE:]
            except socket.timeout:
                pass

    def _decode_telemetry(self, raw):
        (magic, pitch, roll, yaw, depth, dvl_e, dvl_n, dvl_u, rng, temp,
         lat, lon, pressure, leak, health) = struct.unpack(TEL_FMT, raw)
        if magic != TEL_MAGIC:
            return
        self.t.update({
            'pitch': pitch / 100.0, 'roll': roll / 100.0, 'yaw': yaw / 10.0,
            'depth': depth / 100.0, 'dvl_e': dvl_e / 1000.0,
            'dvl_n': dvl_n / 1000.0, 'dvl_u': dvl_u / 1000.0,
            'dvl_range': rng / 100.0, 'temp': temp / 100.0,
            'lat': lat / 1e6, 'lon': lon / 1e6,
            'pressure': pressure, 'leak': leak, 'health': health,
        })
        self.have_telem = True
        self.last_telem_time = time.time()
        now = time.time()
        if now - self._last_tel_print >= 1.0:   # throttle console to ~1 Hz
            self._last_tel_print = now
            print(f"[Telemetry] P={self.t['pitch']:.1f}deg R={self.t['roll']:.1f}deg "
                  f"Y={self.t['yaw']:.1f}deg D={self.t['depth']:.2f}m "
                  f"Temp={self.t['temp']:.1f}C Health={self.t['health']:08b}")
        self._check_alerts()

    def _check_alerts(self):
        # Leak: emergency STATUSTEXT on the edge, then repeated every 3 s while it
        # persists so a dropped telemetry/statustext can't hide an active leak.
        # The ROV auto-surfaces on the same signal (board-side), so the text says so.
        now = time.time()
        if self.t['leak'] and (not self.prev_leak or now - self._leak_alert_t >= 3.0):
            self._emit(lambda: self.mav.statustext_send(
                severity=0, text=b'LEAK! Auto-surfacing'))
            self._leak_alert_t = now
        self.prev_leak = self.t['leak']
        # Newly-failed sensors -> critical STATUSTEXT
        h = self.t['health']
        newly_failed = (~h) & self.prev_health & 0xFF
        if newly_failed:
            names = [n for i, n in enumerate(HEALTH_ORDER)
                     if newly_failed & (1 << i)]
            self._emit(lambda: self.mav.statustext_send(
                severity=2, text=('FAIL: ' + ','.join(names))[:50].encode()))
        self.prev_health = h | 0xFF00  # only track low 8 bits

    # ── QGC receive ──────────────────────────────────────────────────────────

    def _qgc_loop(self):
        print(f"QGC receive thread started on port {MAVLINK_BIND_PORT}...")
        while True:
            try:
                data, addr = self.qgc_sock.recvfrom(1024)
                # print(f"Received {len(data)} bytes from {addr}")
            except socket.timeout:
                continue
            except Exception as e:
                print(f"QGC recv error: {e}")
                continue
            if self.qgc_addr != addr:
                self.qgc_addr = addr
                print(f'QGC detected at {addr[0]}:{addr[1]}')
            try:
                msgs = self.mav.parse_buffer(data)
                if msgs:
                    # print(f"Parsed {len(msgs)} messages from QGC")
                    for msg in msgs:
                        self._handle_qgc_msg(msg)
            except Exception as e:
                print(f"Error parsing MAVLink from QGC: {e}")

    def _handle_qgc_msg(self, msg):
        name = msg.get_type()
        if name == 'MANUAL_CONTROL':
            self.cmd['fwd']    = msg.x / 1000.0
            self.cmd['strafe'] = msg.y / 1000.0
            self.cmd['yaw']    = msg.r / 1000.0
            self.cmd['vert']   = (msg.z - 500) / 500.0
            self.cmd['buttons'] = msg.buttons or 0
            self.last_mc_time = time.time()
        elif name == 'COMMAND_LONG':
            self._handle_command_long(msg)
        elif name == 'SET_MODE':
            self._set_qgc_mode(int(msg.custom_mode))
        elif name == 'PARAM_REQUEST_LIST':
            self._send_params()
        elif name == 'PARAM_REQUEST_READ':
            self._emit(lambda: self.mav.param_value_send(
                param_id=b'SYSID_THISMAV'.ljust(16, b'\0'),
                param_value=1.0, param_type=9, param_count=2, param_index=0))
        elif name == 'MISSION_REQUEST_LIST':
            mtype = getattr(msg, 'mission_type', 0)
            self._emit(lambda: self.mav.mission_count_send(
                msg.get_srcSystem(), msg.get_srcComponent(), 0, mission_type=mtype))
        elif name == 'MISSION_CLEAR_ALL':
            mtype = getattr(msg, 'mission_type', 0)
            self._emit(lambda: self.mav.mission_ack_send(
                msg.get_srcSystem(), msg.get_srcComponent(), 0, mission_type=mtype))
        elif name == 'MISSION_COUNT':
            mtype = getattr(msg, 'mission_type', 0)
            self._emit(lambda: self.mav.mission_request_int_send(
                msg.get_srcSystem(), msg.get_srcComponent(), 0, mission_type=mtype))
        elif name == 'MISSION_ITEM':
            self._handle_mission_item(msg, scaled=False)
        elif name == 'MISSION_ITEM_INT':
            self._handle_mission_item(msg, scaled=True)

    def _handle_mission_item(self, msg, scaled):
        """QGC 'Go to location' item -> GOTO packet down to the ROV + MISSION_ACK to QGC.
        MISSION_ITEM (float) x/y are degrees; MISSION_ITEM_INT x/y are deg*1e7."""
        try:
            lat = (msg.x / 1e7) if scaled else float(msg.x)
            lon = (msg.y / 1e7) if scaled else float(msg.y)
        except Exception:
            return
        if abs(lat) > 90 or abs(lon) > 180 or (lat == 0 and lon == 0):
            return
        self._send_goto_packet(lat, lon)
        print(f'GOTO -> ROV (acoustic): {lat:.7f}, {lon:.7f}')
        mtype = getattr(msg, 'mission_type', 0)
        self._emit(lambda: self.mav.mission_ack_send(
            msg.get_srcSystem(), msg.get_srcComponent(), 0, mission_type=mtype))

    def _send_goto_packet(self, lat, lon):
        with self._link_lock:
            link = self._link
        if link is None:
            return
        try:
            link.sendall(struct.pack(GOTO_FMT, GOTO_MAGIC,
                                     int(lat * 1e7), int(lon * 1e7)))
        except Exception:
            with self._link_lock:
                if self._link is link:
                    self._link = None

    def _set_qgc_mode(self, mode):
        """Track QGC's mode. MANUAL (19) also cancels any auto-GOTO on the ROV:
        a 0,0 goto packet is treated by guidance as 'cancel -> return to manual'."""
        self.custom_mode = mode
        if mode == 19:  # ArduSub MANUAL
            self._send_goto_packet(0.0, 0.0)
            print('QGC Manual -> GOTO cancel sent')

    def _handle_command_long(self, msg):
        MAV_CMD_COMPONENT_ARM_DISARM = 400
        MAV_CMD_DO_SET_MODE = 176
        result = 0  # MAV_RESULT_ACCEPTED
        if msg.command == MAV_CMD_COMPONENT_ARM_DISARM:
            self.armed = (msg.param1 == 1.0)
            print('Armed' if self.armed else 'Disarmed')
        elif msg.command == MAV_CMD_DO_SET_MODE:
            self._set_qgc_mode(int(msg.param2))
        else:
            result = 3  # MAV_RESULT_UNSUPPORTED
        self._emit(lambda: self.mav.command_ack_send(
            command=msg.command, result=result))

    def _send_params(self):
        params = [('SYSID_THISMAV', 1), ('ARMING_CHECK', 0)]
        for i, (nm, val) in enumerate(params):
            self._emit(lambda nm=nm, val=val, i=i: self.mav.param_value_send(
                param_id=nm.encode().ljust(16, b'\0'),
                param_value=float(val), param_type=9,
                param_count=len(params), param_index=i))

    # ── synthesized MAVLink up to QGC ────────────────────────────────────────

    def _send_heartbeat(self):
        base_mode = 0x40 | 0x01
        if self.armed:
            base_mode |= 0x80
        self._emit(lambda: self.mav.heartbeat_send(
            type=12, autopilot=3, base_mode=base_mode,
            custom_mode=self.custom_mode, system_status=4))

    def _send_telemetry_mav(self):
        if self.qgc_addr is None:
            return
        t = self.t
        tb = int((time.time() * 1000)) & 0xFFFFFFFF

        self._emit(lambda: self.mav.attitude_send(
            time_boot_ms=tb,
            roll=math.radians(t['roll']), pitch=math.radians(t['pitch']),
            yaw=math.radians(t['yaw']), rollspeed=0.0, pitchspeed=0.0, yawspeed=0.0))

        self._emit(lambda: self.mav.global_position_int_send(
            time_boot_ms=tb,
            lat=int(t['lat'] * 1e7), lon=int(t['lon'] * 1e7),
            alt=0, relative_alt=int(-t['depth'] * 1000),
            vx=int(t['dvl_e'] * 100), vy=int(t['dvl_n'] * 100),
            vz=int(-t['dvl_u'] * 100),
            hdg=int(t['yaw'] * 100) % 36000))

        gs = math.sqrt(t['dvl_e'] ** 2 + t['dvl_n'] ** 2)
        self._emit(lambda: self.mav.vfr_hud_send(
            airspeed=gs, groundspeed=gs, heading=int(t['yaw']) % 360,
            throttle=0, alt=0.0, climb=t['dvl_u']))

        self._emit(lambda: self.mav.scaled_pressure_send(
            time_boot_ms=tb,
            press_abs=float(t['pressure']) if t['pressure'] else 1013.25,
            press_diff=0.0, temperature=int(t['temp'] * 100)))

        self._emit(lambda: self.mav.gps_raw_int_send(
            time_usec=int(time.time() * 1e6),
            fix_type=3 if t['lat'] else 0,
            lat=int(t['lat'] * 1e7), lon=int(t['lon'] * 1e7),
            alt=0, eph=9999, epv=9999, vel=0, cog=0, satellites_visible=255))

        self._emit(lambda: self.mav.sys_status_send(
            onboard_control_sensors_present=0x1023,
            onboard_control_sensors_enabled=0x1023,
            onboard_control_sensors_health=0x1023,
            load=0, voltage_battery=12000, current_battery=-1,
            battery_remaining=100, drop_rate_comm=0, errors_comm=0,
            errors_count1=0, errors_count2=0, errors_count3=0, errors_count4=0))

        if t['dvl_range'] > 0:
            self._emit(lambda: self.mav.distance_sensor_send(
                time_boot_ms=tb, min_distance=1, max_distance=5000,
                current_distance=max(1, min(int(t['dvl_range'] * 100), 65535)),
                type=0, id=0, orientation=25, covariance=255))

    def _check_link_health(self):
        """Warn QGC (once) when telemetry from the ROV goes stale, and again on
        recovery — so a dropped acoustic link doesn't look like a healthy vehicle."""
        if not self.have_telem:
            return   # nothing received yet; nothing to lose
        stale = (time.time() - self.last_telem_time) > LINK_TIMEOUT
        if stale and not self.link_lost:
            self.link_lost = True
            self._emit(lambda: self.mav.statustext_send(
                severity=2, text=b'ACOUSTIC LINK LOST'))
            print('*** Acoustic link LOST ***')
        elif not stale and self.link_lost:
            self.link_lost = False
            self._emit(lambda: self.mav.statustext_send(
                severity=6, text=b'Acoustic link restored'))
            print('Acoustic link restored')

    # ── main loop ────────────────────────────────────────────────────────────

    def run(self):
        threading.Thread(target=self._link_loop, daemon=True).start()
        threading.Thread(target=self._qgc_loop, daemon=True).start()

        last_hb = last_cmd = last_tel = 0.0
        try:
            while True:
                now = time.time()
                if now - last_hb >= 1.0 / HEARTBEAT_HZ:
                    self._send_heartbeat()
                    last_hb = now
                if now - last_cmd >= 1.0 / CMD_HZ:
                    self._send_command_packet()
                    last_cmd = now
                if now - last_tel >= 1.0 / TELEM_TX_HZ:
                    self._send_telemetry_mav()
                    self._check_link_health()
                    last_tel = now
                time.sleep(0.01)
        except KeyboardInterrupt:
            print('\nStopped.')


if __name__ == '__main__':
    AcousticQGCBridge().run()
