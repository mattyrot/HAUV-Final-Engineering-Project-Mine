#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
import socket
import struct
import threading
import math
import time
import queue

SUBSONUS_IP = '192.168.168.103'
SUBSONUS_PORT = 16719
PACKET_SYSTEM_STATE = 20
AN_PACKET_HEADER_SIZE = 5


def crc16_ccitt(data) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
        crc &= 0xFFFF
    return crc


def calculate_header_lrc(id_, length, crc) -> int:
    return id_ ^ length ^ (crc & 0xFF) ^ (crc >> 8)


def encode_request(packet_ids) -> bytes:
    payload = bytes(packet_ids)
    crc = crc16_ccitt(payload)
    lrc = calculate_header_lrc(1, len(payload), crc)
    return bytes([lrc, 1, len(payload)]) + struct.pack('<H', crc) + payload


def encode_packets_period(packet_id, period, permanent=0, clear_existing=1) -> bytes:
    """Packets Period Packet (ID 181) — request periodic output of a packet.
    period is in units of the device packet timer (default 1000 us base),
    so period=50 -> 20 Hz. permanent=0 keeps it session-only."""
    payload = bytes([permanent, clear_existing, packet_id]) + struct.pack('<I', period)
    crc = crc16_ccitt(payload)
    lrc = calculate_header_lrc(181, len(payload), crc)
    return bytes([lrc, 181, len(payload)]) + struct.pack('<H', crc) + payload


# Identical to the proven ANDecoder in subsonus_test.py
class ANDecoder:
    def __init__(self):
        self.buffer = bytearray()
        self.decode_iterator = 0
        self.crc_errors = 0

    def add_data(self, data: bytes):
        self.buffer.extend(data)

    def decode(self):
        buf = self.buffer
        buf_len = len(buf)
        while (self.decode_iterator + AN_PACKET_HEADER_SIZE) <= buf_len:
            i = self.decode_iterator
            header_lrc = buf[i]
            id_ = buf[i + 1]
            length = buf[i + 2]
            crc = buf[i + 3] | (buf[i + 4] << 8)
            if header_lrc == calculate_header_lrc(id_, length, crc):
                data_start = i + AN_PACKET_HEADER_SIZE
                data_end = data_start + length
                if data_end > buf_len:
                    return None
                payload = bytes(buf[data_start:data_end])
                if crc16_ccitt(payload) == crc:
                    self.decode_iterator = data_end
                    if self.decode_iterator > 10 * 1024 * 1024:
                        self.buffer = self.buffer[self.decode_iterator:]
                        self.decode_iterator = 0
                    return id_, payload
                else:
                    self.crc_errors += 1
            self.decode_iterator += 1
        if self.decode_iterator > buf_len:
            self.buffer = bytearray()
            self.decode_iterator = 0
        return None


class SubsonusNode(Node):
    def __init__(self):
        super().__init__('subsonus_node')
        self.vel_pub = self.create_publisher(Twist, '/subsonus/velocity', 10)
        self.ori_pub = self.create_publisher(Twist, '/subsonus/orientation', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/subsonus/gps', 10)
        self.sock = None
        self.decoder = ANDecoder()
        self._last_req = 0
        self._last_state_time = 0.0
        self._publish_queue = queue.Queue(maxsize=50)
        # Timer runs in the spin thread — safe for publishing
        self.create_timer(0.02, self._publish_callback)
        threading.Thread(target=self._recv_loop, daemon=True).start()

    def _connect(self):
        while rclpy.ok():
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(1.0)
                s.connect((SUBSONUS_IP, SUBSONUS_PORT))
                self.sock = s
                self.decoder = ANDecoder()
                self._last_req = 0
                # Ask the device to stream System State (packet 20) at ~20 Hz
                try:
                    s.sendall(encode_packets_period(PACKET_SYSTEM_STATE, 50))
                    self.get_logger().info('Requested periodic System State (packet 20) @ 20 Hz')
                except Exception as e:
                    self.get_logger().warn(f'Failed to send packets-period request: {e}')
                self.get_logger().info(f'Connected to Subsonus {SUBSONUS_IP}:{SUBSONUS_PORT}')
                return
            except Exception as e:
                self.get_logger().warn(f'Connect failed: {e} — retrying in 3 s')
                time.sleep(3)

    def _recv_loop(self):
        self._connect()
        while rclpy.ok():
            now = time.time()
            # Re-assert the System State stream every 1 s. We deliberately do NOT
            # spam a device-info request here — doing so disrupts stream alignment
            # and the decoder stops locking onto packet 20.
            if now - self._last_req >= 1.0:
                try:
                    self.sock.sendall(encode_packets_period(PACKET_SYSTEM_STATE, 50))
                except Exception:
                    pass
                self._last_req = now

            try:
                raw = self.sock.recv(4096)
                if not raw:
                    raise ConnectionError('connection closed')
                self.decoder.add_data(raw)
                while True:
                    result = self.decoder.decode()
                    if result is None:
                        break
                    pid, payload = result
                    self._handle_packet(pid, payload)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().warn(f'Recv error: {e} — reconnecting')
                try:
                    self.sock.close()
                except Exception:
                    pass
                self._connect()

    def _publish_callback(self):
        try:
            while True:
                vel, ori, gps = self._publish_queue.get_nowait()
                self.vel_pub.publish(vel)
                self.ori_pub.publish(ori)
                self.gps_pub.publish(gps)
        except queue.Empty:
            pass

    def _handle_packet(self, pid: int, payload: bytes):
        if pid == PACKET_SYSTEM_STATE and len(payload) >= 116:
            self._last_state_time = time.time()
            # Subsonus System State (packet 20) is 116 bytes — 4 extra bytes at
            # offset 12 shift the standard ANPP layout by +4. Offsets below were
            # verified against real packet bytes (position matched known location,
            # g_force at 64 = 1.0 g, velocity ~0 on bench).
            lat, lon, height = struct.unpack_from('<3d', payload, 16)   # radians / m
            vn, ve, vd = struct.unpack_from('<3f', payload, 40)
            roll, pitch, heading = struct.unpack_from('<3f', payload, 68)

            vel = Twist()
            vel.linear.x = float(vn)
            vel.linear.y = float(ve)
            vel.linear.z = float(vd)

            ori = Twist()
            ori.linear.x = math.degrees(heading) % 360.0
            ori.linear.y = math.degrees(pitch)
            ori.linear.z = math.degrees(roll)

            gps = NavSatFix()
            gps.header.stamp = self.get_clock().now().to_msg()
            gps.header.frame_id = 'subsonus'
            gps.latitude = math.degrees(lat)
            gps.longitude = math.degrees(lon)
            gps.altitude = float(height)

            try:
                self._publish_queue.put_nowait((vel, ori, gps))
            except queue.Full:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = SubsonusNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
