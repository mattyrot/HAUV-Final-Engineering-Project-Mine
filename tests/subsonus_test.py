#!/usr/bin/env python3
"""
Mirrors example_sync.py exactly — no external libraries.
Run: python3 subsonus_test.py --ip 192.168.168.103
"""
import time
import socket
import struct
import argparse

AN_PACKET_HEADER_SIZE = 5


# ── ANPP helpers ──────────────────────────────────────────────────────────────

def calculate_crc16(data) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
        crc &= 0xFFFF
    return crc


def calculate_header_lrc(id_, length, crc) -> int:
    return id_ ^ length ^ (crc & 0xFF) ^ (crc >> 8)


def encode_request(packet_id: int) -> bytes:
    """Build ANPP Request Packet (ID=1) for a single packet ID."""
    payload = bytes([packet_id])
    crc = calculate_crc16(payload)
    lrc = calculate_header_lrc(1, len(payload), crc)
    return bytes([lrc, 1, len(payload)]) + struct.pack('<H', crc) + payload


# ── ANDecoder — mirrors AN SDK ANDecoder ──────────────────────────────────────

class ANPacket:
    def __init__(self):
        self.id = 0
        self.length = 0
        self.data = b''


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

                if calculate_crc16(payload) == crc:
                    self.decode_iterator = data_end
                    if self.decode_iterator > 10 * 1024 * 1024:
                        self.buffer = self.buffer[self.decode_iterator:]
                        self.decode_iterator = 0
                    pkt = ANPacket()
                    pkt.id = id_
                    pkt.length = length
                    pkt.data = payload
                    return pkt
                else:
                    self.crc_errors += 1

            self.decode_iterator += 1

        if self.decode_iterator > buf_len:
            self.buffer = bytearray()
            self.decode_iterator = 0

        return None


# ── Packet printers ───────────────────────────────────────────────────────────

def print_packet(pkt: ANPacket):
    if pkt.id == 3 and len(pkt.data) >= 24:
        sw, dev_id, hw, s0, s1, s2 = struct.unpack_from('<IIIIII', pkt.data)
        print(f"  Device Information: device_id={dev_id} sw={sw} hw={hw} serial=({s0},{s1},{s2})")

    elif pkt.id == 20 and len(pkt.data) >= 100:
        sys_status, filt_status = struct.unpack_from('<HH', pkt.data, 0)
        lat, lon, h = struct.unpack_from('<ddd', pkt.data, 12)
        vn, ve, vd = struct.unpack_from('<fff', pkt.data, 36)
        roll, pitch, heading = struct.unpack_from('<fff', pkt.data, 64)
        print(f"  System State: sys_status=0x{sys_status:04x} filter=0x{filt_status:04x}")
        print(f"    position: lat={lat*57.2958:.6f} lon={lon*57.2958:.6f} h={h:.2f}m")
        print(f"    velocity: N={vn:.3f} E={ve:.3f} D={vd:.3f} m/s")
        print(f"    orientation: roll={roll*57.2958:.2f} pitch={pitch*57.2958:.2f} heading={heading*57.2958:.2f} deg")

    elif pkt.id == 28 and len(pkt.data) >= 48:
        ax, ay, az = struct.unpack_from('<fff', pkt.data, 0)
        gx, gy, gz = struct.unpack_from('<fff', pkt.data, 12)
        print(f"  Raw Sensors: accel=({ax:.3f},{ay:.3f},{az:.3f}) gyro=({gx:.3f},{gy:.3f},{gz:.3f})")

    else:
        print(f"  Packet id={pkt.id} length={pkt.length} data={pkt.data[:16].hex()}")


# ── Main — mirrors example_sync.py structure ──────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', '-i', required=True)
    parser.add_argument('--port', '-p', type=int, default=16719)
    args = parser.parse_args()

    print(f"Connecting via TCP to {args.ip}:{args.port}...")
    conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    conn.settimeout(1.0)
    conn.connect((args.ip, args.port))
    print("Connection established. Requesting Device Information...")

    decoder = ANDecoder()
    device_found = False

    # Send initial request for Device Information (packet 3) — same as official example
    request_bytes = encode_request(3)
    print(f"Request packet: {request_bytes.hex()}")
    conn.sendall(request_bytes)
    last_request_time = time.time()

    print("Listening for packets... (Press Ctrl+C to exit)\n")
    print("-" * 40)

    try:
        while True:
            # Retry request every 1 second until device responds — same as official example
            if not device_found and time.time() - last_request_time > 1.0:
                conn.sendall(request_bytes)
                print(f"[retry] Sent request, crc_errors so far: {decoder.crc_errors}")
                last_request_time = time.time()

            # Read — mirrors read_from_socket()
            try:
                raw_data = conn.recv(1024)
            except Exception:
                raw_data = b""

            if raw_data:
                print(f"RX {len(raw_data)} bytes: {raw_data[:32].hex()}")
                decoder.add_data(raw_data)

            # Decode loop — mirrors official example
            while True:
                pkt = decoder.decode()
                if pkt is None:
                    break
                if pkt.id == 3:
                    device_found = True
                print(f"Packet id={pkt.id} len={pkt.length}")
                print_packet(pkt)
                print("-" * 40)

    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        conn.close()


if __name__ == '__main__':
    main()
