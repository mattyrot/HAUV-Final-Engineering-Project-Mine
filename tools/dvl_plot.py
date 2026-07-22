"""
Real-time Pathfinder DVL plotter.
Shows live velocity (BE) and range-to-bottom.
Run: python tools/dvl_plot.py
"""

import socket
import threading
import time
import collections
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation

DVL_IP       = '192.168.168.102'
DVL_CMD_PORT = 1033
LISTEN_PORT  = 1037
HISTORY      = 200  # number of samples to show

INIT_COMMANDS = [
    b'===\r\n',
    b'CR1\r\n',
    b'CP1\r\n',
    b'PD6\r\n',
    b'EX11110\r\n',
    b'EA+4500\r\n',
    b'EZ11000010\r\n',
    b'CK\r\n',
    b'CS\r\n',
]

# Shared state updated by receiver thread
data = {
    't':     collections.deque(maxlen=HISTORY),
    'E':     collections.deque(maxlen=HISTORY),
    'N':     collections.deque(maxlen=HISTORY),
    'U':     collections.deque(maxlen=HISTORY),
    'range': collections.deque(maxlen=HISTORY),
    'valid': collections.deque(maxlen=HISTORY),  # True = status A
}
t0 = time.time()
lock = threading.Lock()


def send_init(tcp):
    for cmd in INIT_COMMANDS:
        tcp.sendall(cmd)
        time.sleep(0.15)
        try:
            tcp.settimeout(0.4)
            tcp.recv(256)
        except socket.timeout:
            pass
    print('DVL pinging.')


def parse_be(line):
    parts = line.split(',')
    try:
        e = int(parts[1]) / 1000.0
        n = int(parts[2]) / 1000.0
        u = int(parts[3]) / 1000.0
        status = parts[4].strip().rstrip('*')
        return e, n, u, status == 'A'
    except (ValueError, IndexError):
        return None


def parse_bd(line):
    parts = line.split(',')
    try:
        return float(parts[4])  # range_to_bottom
    except (ValueError, IndexError):
        return None


def receiver():
    tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp.settimeout(5)
    tcp.connect((DVL_IP, DVL_CMD_PORT))
    send_init(tcp)

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp.bind(('0.0.0.0', LISTEN_PORT))
    udp.settimeout(2.0)

    be_latest  = (0.0, 0.0, 0.0, False)
    rng_latest = 0.0

    while True:
        try:
            raw, _ = udp.recvfrom(4096)
            text = raw.decode('ascii', errors='replace')
            for line in text.strip().splitlines():
                line = line.strip().lstrip(':')
                if line.startswith('BE'):
                    result = parse_be(line)
                    if result:
                        be_latest = result
                elif line.startswith('BD'):
                    result = parse_bd(line)
                    if result is not None:
                        rng_latest = result

            with lock:
                now = time.time() - t0
                # Replace -32.768 sentinel with None for gaps in plot
                e, n, u, valid = be_latest
                data['t'].append(now)
                data['E'].append(e   if valid else None)
                data['N'].append(n   if valid else None)
                data['U'].append(u   if valid else None)
                data['range'].append(rng_latest)
                data['valid'].append(valid)

        except socket.timeout:
            pass
        except Exception as e:
            print(f'Receiver error: {e}')


def main():
    t = threading.Thread(target=receiver, daemon=True)
    t.start()
    print('Waiting for DVL data...')

    fig, (ax_vel, ax_rng) = plt.subplots(2, 1, figsize=(10, 7), sharex=False)
    fig.suptitle('Pathfinder DVL — Live', fontsize=13)

    line_e, = ax_vel.plot([], [], color='tab:blue',   label='East (E)')
    line_n, = ax_vel.plot([], [], color='tab:orange',  label='North (N)')
    line_u, = ax_vel.plot([], [], color='tab:green',   label='Up (U)')
    ax_vel.set_ylabel('Velocity (m/s)')
    ax_vel.set_xlabel('Time (s)')
    ax_vel.legend(loc='upper left')
    ax_vel.set_ylim(-0.5, 0.5)
    ax_vel.axhline(0, color='gray', linewidth=0.5)
    ax_vel.grid(True, alpha=0.3)

    line_r, = ax_rng.plot([], [], color='tab:red', label='Range to bottom')
    ax_rng.set_ylabel('Range (m)')
    ax_rng.set_xlabel('Time (s)')
    ax_rng.legend(loc='upper left')
    ax_rng.set_ylim(0, 5)
    ax_rng.grid(True, alpha=0.3)

    status_text = ax_vel.text(
        0.99, 0.95, 'NO LOCK', transform=ax_vel.transAxes,
        ha='right', va='top', fontsize=10,
        color='red', fontweight='bold'
    )

    def update(_frame):
        with lock:
            if not data['t']:
                return line_e, line_n, line_u, line_r, status_text

            ts  = list(data['t'])
            E   = list(data['E'])
            N   = list(data['N'])
            U   = list(data['U'])
            rng = list(data['range'])
            valid = list(data['valid'])

        line_e.set_data(ts, E)
        line_n.set_data(ts, N)
        line_u.set_data(ts, U)
        line_r.set_data(ts, rng)

        for ax in (ax_vel, ax_rng):
            ax.set_xlim(max(0, ts[-1] - 30), ts[-1] + 1)

        # Auto-scale velocity axis with some padding
        vals = [v for v in E + N + U if v is not None]
        if vals:
            lo, hi = min(vals), max(vals)
            pad = max(0.1, (hi - lo) * 0.2)
            ax_vel.set_ylim(lo - pad, hi + pad)

        # Auto-scale range axis
        if rng:
            ax_rng.set_ylim(0, max(rng) * 1.3 + 0.1)

        locked = valid[-1] if valid else False
        status_text.set_text('BOTTOM LOCK' if locked else 'NO LOCK')
        status_text.set_color('green' if locked else 'red')

        return line_e, line_n, line_u, line_r, status_text

    ani = animation.FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
