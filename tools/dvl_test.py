"""
Standalone Pathfinder DVL test — Ethernet, no ROS.

1. Connects TCP -> 192.168.168.102:1033 and sends init commands
2. Listens UDP on LISTEN_PORT for PD6 datagrams
3. Parses and prints all PD6 sentences in real time

NOTE: The DVL web UI must send PD6 to THIS machine's IP.
      Default is 192.168.168.101 (UP Board) — change in the web UI if running on Windows.
"""

import socket
import time

DVL_IP       = '192.168.168.102'
DVL_CMD_PORT = 1033
LISTEN_PORT  = 1037   # must match DVL web UI PD6 port

# ---- init command sequence (from working serial driver) ----
INIT_COMMANDS = [
    b'===\r\n',        # break out of any running state
    b'CR1\r\n',        # factory defaults
    b'CP1\r\n',        # required
    b'PD6\r\n',        # PD6 output format
    b'EX11110\r\n',    # coordinate transformation
    b'EA+4500\r\n',    # heading alignment
    b'EZ11000010\r\n', # sensor source: internal SoS, depth, temperature
    b'CK\r\n',         # store parameters
    b'CS\r\n',         # start pinging
]


# ---- PD6 parsers -------------------------------------------------------

def parse_sa(parts):
    """SA — System Attitude"""
    return f"[SA] pitch={parts[1]}°  roll={parts[2]}°  heading={parts[3]}°"

def parse_ts(parts):
    """TS — Timing / Scaling"""
    return (f"[TS] time={parts[1]}  salinity={parts[2]} ppt  "
            f"temp={parts[3]}°C  depth={parts[4]}m  SoS={parts[5]}m/s")

def parse_be(parts):
    """BE — Earth-referenced velocity (mm/s → m/s)"""
    try:
        e = int(parts[1]) / 1000.0
        n = int(parts[2]) / 1000.0
        u = int(parts[3]) / 1000.0
        status = parts[4].strip().rstrip('*')
        return f"[BE] E={e:+.3f}  N={n:+.3f}  U={u:+.3f} m/s   status={status}"
    except ValueError:
        return f"[BE] bad data: {parts}"

def parse_bd(parts):
    """BD — Earth-referenced distance"""
    return (f"[BD] E={parts[1]}m  N={parts[2]}m  U={parts[3]}m  "
            f"range_to_bottom={parts[4]}m  dt={parts[5]}s")

def parse_bs(parts):
    """BS — Ship (body) referenced velocity"""
    try:
        port = int(parts[1]) / 1000.0
        aft  = int(parts[2]) / 1000.0
        up   = int(parts[3]) / 1000.0
        status = parts[4].strip().rstrip('*')
        return f"[BS] port={port:+.3f}  aft={aft:+.3f}  up={up:+.3f} m/s   status={status}"
    except ValueError:
        return f"[BS] bad data: {parts}"

def parse_bi(parts):
    """BI — Bottom Track Instrument-referenced velocity (beam coords, mm/s -> m/s)"""
    try:
        vals = [int(p) / 1000.0 for p in parts[1:5]]
        status = parts[5].strip().rstrip('*')
        return (f"[BI] b1={vals[0]:+.3f}  b2={vals[1]:+.3f}  "
                f"b3={vals[2]:+.3f}  b4={vals[3]:+.3f} m/s   status={status}")
    except (ValueError, IndexError):
        return f"[BI] bad data: {parts}"

def parse_hm(parts):
    """HM — System Health Monitor"""
    try:
        return (f"[HM] leak_A={parts[1]}  leak_B={parts[2]}  "
                f"raw_A={parts[3]}  raw_B={parts[4]}  "
                f"Vtx={parts[5].lstrip('*')}V  "
                f"Itx={parts[6].lstrip('*')}A  "
                f"impedance={parts[7].lstrip('*').rstrip()} ohm")
    except IndexError:
        return f"[HM] {parts}"

PARSERS = {
    'SA': parse_sa,
    'TS': parse_ts,
    'BE': parse_be,
    'BD': parse_bd,
    'BS': parse_bs,
    'BI': parse_bi,
    'HM': parse_hm,
}

def parse_pd6_packet(raw: bytes):
    text = raw.decode('ascii', errors='replace')
    for line in text.strip().splitlines():
        line = line.strip().lstrip(':')
        if not line:
            continue
        tag = line[:2]
        parts = line.split(',')
        parser = PARSERS.get(tag)
        if parser:
            try:
                print(parser(parts))
            except Exception as e:
                print(f"  [parse error {tag}]: {e}  raw={line}")
        else:
            print(f"  [??] {line}")


# ---- TCP command sender -------------------------------------------------

def send_init(tcp: socket.socket):
    print(f"Connected to DVL at {DVL_IP}:{DVL_CMD_PORT}")
    print("Sending init commands:")
    for cmd in INIT_COMMANDS:
        tcp.sendall(cmd)
        label = cmd.decode().strip()
        try:
            tcp.settimeout(0.5)
            resp = tcp.recv(256).decode('ascii', errors='replace').strip()
            print(f"  {label!r:20s}  -> {resp!r}")
        except socket.timeout:
            print(f"  {label!r:20s}  -> (no response)")
        time.sleep(0.15)
    print("DVL pinging.\n")


# ---- Main loop ---------------------------------------------------------

def main():
    # TCP command channel
    tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp.settimeout(5)
    try:
        tcp.connect((DVL_IP, DVL_CMD_PORT))
    except (ConnectionRefusedError, socket.timeout) as e:
        print(f"ERROR: Cannot reach DVL at {DVL_IP}:{DVL_CMD_PORT} — {e}")
        return

    send_init(tcp)

    # UDP data channel
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp.bind(('0.0.0.0', LISTEN_PORT))
    udp.settimeout(3.0)
    print(f"Listening for PD6 on UDP port {LISTEN_PORT} ...")
    print("(Ctrl-C to stop)\n")
    print("-" * 60)

    try:
        while True:
            try:
                data, addr = udp.recvfrom(4096)
                print(f"\n--- packet from {addr[0]} ---")
                parse_pd6_packet(data)
            except socket.timeout:
                print("[waiting for DVL data...]")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        tcp.close()
        udp.close()


if __name__ == '__main__':
    main()
