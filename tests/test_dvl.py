#!/usr/bin/env python3
"""
Wayfinder binary protocol over PD6 TCP.
DVL connects OUT to us after CS — we act as TCP server on port 1037.
"""
import socket
import struct
import time
import threading

DVL_IP = "192.168.168.102"
MY_IP  = "192.168.168.1"
CMD_PORT  = 1033
DATA_PORT = 1037

WAYFINDER_SOP = bytes([0xAA, 0x10, 0x01, 0x74, 0x00, 0x10])

def parse_wayfinder_packet(data):
    if len(data) < 6:
        return None
    # find SOP in case packet has offset
    idx = data.find(WAYFINDER_SOP)
    if idx == -1:
        return None
    data = data[idx:]
    try:
        offset = 15  # skip SOP (6) + Data ID (9)
        offset += 6  # sys_type, sub_type, fw x4
        offset += 8  # year, month, day, hour, min, sec, ms(2)
        offset += 1  # coord_sys
        vx, vy, vz, ve = struct.unpack_from('<ffff', data, offset); offset += 16
        r1, r2, r3, r4 = struct.unpack_from('<ffff', data, offset); offset += 16
        mean_range = struct.unpack_from('<f', data, offset)[0]; offset += 4
        sos        = struct.unpack_from('<f', data, offset)[0]; offset += 4
        bt_status  = struct.unpack_from('<H', data, offset)[0]; offset += 2
        bit_flags  = struct.unpack_from('<H', data, offset)[0]; offset += 2
        v_in       = struct.unpack_from('<f', data, offset)[0]
        return {'vx': vx, 'vy': vy, 'vz': vz,
                'mean_range': mean_range, 'sos': sos,
                'bt_status': f'0x{bt_status:04X}', 'voltage_in': v_in}
    except Exception as e:
        print(f"  Parse error: {e}")
        return None

# Start TCP server
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((MY_IP, DATA_PORT))
server.listen(1)
server.settimeout(20)
print(f"TCP server listening on {MY_IP}:{DATA_PORT}...")

# Send CS
print("Sending CS to start pinging...")
cmd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
cmd.bind((MY_IP, 0))
cmd.settimeout(5)
cmd.connect((DVL_IP, CMD_PORT))
print(cmd.recv(256).decode().strip())
cmd.sendall(b"CS\r\n")
time.sleep(0.5)
cmd.close()
print("CS sent. Waiting for DVL to connect...\n")

try:
    conn, addr = server.accept()
    print(f"DVL connected from {addr}\n")
    buf = b''
    while True:
        try:
            chunk = conn.recv(4096)
            if not chunk:
                print("DVL disconnected.")
                break
            buf += chunk
            print(f"Raw ({len(chunk)} bytes): {chunk[:32].hex()}")
            parsed = parse_wayfinder_packet(buf)
            if parsed:
                import math
                print(f"  Vx={parsed['vx']:.4f}  Vy={parsed['vy']:.4f}  Vz={parsed['vz']:.4f} m/s")
                print(f"  Range: {parsed['mean_range']:.3f} m   SoS: {parsed['sos']:.1f} m/s")
                print(f"  BT status: {parsed['bt_status']}   Voltage: {parsed['voltage_in']:.2f} V")
                buf = b''
            print()
        except KeyboardInterrupt:
            break
    conn.close()
except socket.timeout:
    print("Timeout — DVL did not connect. Check PD6 config in web UI.")

server.close()
