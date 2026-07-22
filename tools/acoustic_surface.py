"""
acoustic_surface.py — runs on PC (Windows)
Surface-side acoustic bridge: sends joystick commands to ROV via Subsonus,
displays incoming telemetry (IMU, depth, DVL velocity, temperature, GPS).

Requirements:
    pip install pygame

Run:
    python tools/acoustic_surface.py
"""

import struct
import socket
import threading
import time
import sys

try:
    import pygame
    HAVE_PYGAME = True
except ImportError:
    HAVE_PYGAME = False
    print('pygame not installed — joystick unavailable.  pip install pygame')

# ── link mode ────────────────────────────────────────────────────────────────
# DIRECT_TCP_TEST = True: bench test over plain Ethernet — connect straight to the
#   UP Board (which runs the bridge as a TCP server). No Subsonus, no water.
# DIRECT_TCP_TEST = False: real link — connect to the PC's Subsonus modem port.
DIRECT_TCP_TEST = False

if DIRECT_TCP_TEST:
    SUBSONUS_IP   = '192.168.168.101'   # UP Board — bridge server
    SUBSONUS_PORT = 17000               # must match TEST_PORT in acoustic_bridge_node
else:
    SUBSONUS_IP   = '168.254.1.80'      # PC's Subsonus (Ethernet 5 direct link)
    SUBSONUS_PORT = 16740

CMD_HZ        = 10   # command send rate

CMD_MAGIC = 0xAC
TEL_MAGIC = 0xAB

CMD_FMT  = '!BbbbbbH'
CMD_SIZE = struct.calcsize(CMD_FMT)   # 8

TEL_FMT  = '!BhhhHhhhHhiiHBH'
TEL_SIZE = struct.calcsize(TEL_FMT)   # 32

# Bit order of the health bitmask — must match HEALTH_ORDER in health_monitor_node.
HEALTH_ORDER = ['bno055', 'bar100', 'bme280', 'leak',
                'dvl', 'subsonus_vel', 'subsonus_ori', 'gps']

telemetry   = {}
tel_lock    = threading.Lock()
tel_updated = threading.Event()


# ── helpers ───────────────────────────────────────────────────────────────────

def _clamp_i8(v):
    return max(-128, min(127, int(v)))


def pack_command(fwd, strafe, yaw, vert, cam, buttons):
    return struct.pack(
        CMD_FMT,
        CMD_MAGIC,
        _clamp_i8(fwd    * 100),
        _clamp_i8(strafe * 100),
        _clamp_i8(yaw    * 100),
        _clamp_i8(vert   * 100),
        _clamp_i8(cam    * 100),
        int(buttons) & 0xFFFF,
    )


def unpack_telemetry(raw):
    if len(raw) < TEL_SIZE:
        return None
    magic, pitch, roll, yaw, depth, dvl_e, dvl_n, dvl_u, rng, temp, lat, lon, \
        pressure, leak, health = struct.unpack(TEL_FMT, raw[:TEL_SIZE])
    if magic != TEL_MAGIC:
        return None
    return {
        'pitch':     pitch / 100.0,
        'roll':      roll  / 100.0,
        'yaw':       yaw   / 10.0,
        'depth':     depth / 100.0,
        'dvl_e':     dvl_e / 1000.0,
        'dvl_n':     dvl_n / 1000.0,
        'dvl_u':     dvl_u / 1000.0,
        'dvl_range': rng   / 100.0,
        'temp':      temp  / 100.0,
        'lat':       lat   / 1e6,
        'lon':       lon   / 1e6,
        'pressure':  pressure,
        'leak':      leak,
        'health':    health,
    }


def health_str(mask):
    """Render the health bitmask as FAIL list (or 'all OK')."""
    failed = [name for i, name in enumerate(HEALTH_ORDER) if not (mask & (1 << i))]
    return 'all OK' if not failed else 'FAIL: ' + ','.join(failed)


# ── receiver thread ───────────────────────────────────────────────────────────

def receiver_thread(sock):
    buf = b''
    while True:
        try:
            chunk = sock.recv(256)
            if not chunk:
                print('\nConnection closed by remote.')
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
                tel = unpack_telemetry(buf[:TEL_SIZE])
                buf = buf[TEL_SIZE:]
                if tel:
                    with tel_lock:
                        telemetry.update(tel)
                    tel_updated.set()
        except Exception as e:
            print(f'\nReceiver error: {e}')
            break


# ── display ───────────────────────────────────────────────────────────────────

def display():
    with tel_lock:
        t = dict(telemetry)
    if not t:
        print('\r[waiting for telemetry...]', end='', flush=True)
        return
    print(
        f'\r'
        f'P={t["pitch"]:+6.1f}deg '
        f'R={t["roll"]:+6.1f}deg '
        f'Y={t["yaw"]:5.1f}deg  '
        f'Depth={t["depth"]:5.2f}m  '
        f'DVL E={t["dvl_e"]:+6.3f} N={t["dvl_n"]:+6.3f} U={t["dvl_u"]:+6.3f} m/s  '
        f'Rng={t["dvl_range"]:5.2f}m  '
        f'Temp={t["temp"]:5.1f}C  '
        f'Press={t["pressure"]:5d}mbar  '
        f'{"** LEAK **" if t["leak"] else "dry":9s} '
        f'GPS {t["lat"]:.6f},{t["lon"]:.6f}  '
        f'[{health_str(t["health"])}]   ',
        end='', flush=True
    )


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    print(f'Connecting to Subsonus at {SUBSONUS_IP}:{SUBSONUS_PORT}...')
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    try:
        sock.connect((SUBSONUS_IP, SUBSONUS_PORT))
    except Exception as e:
        print(f'Connection failed: {e}')
        sys.exit(1)
    sock.settimeout(None)
    print('Connected.\n')

    threading.Thread(target=receiver_thread, args=(sock,), daemon=True).start()

    joy = None
    if HAVE_PYGAME:
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            joy = pygame.joystick.Joystick(0)
            joy.init()
            print(f'Joystick: {joy.get_name()}')
        else:
            print('No joystick found — sending neutral commands only.')
    else:
        print('pygame not available — sending neutral commands only.')

    print('Running. Ctrl-C to stop.\n')

    interval = 1.0 / CMD_HZ
    try:
        while True:
            t0 = time.time()

            fwd = strafe = yaw = vert = cam = 0.0
            buttons = 0

            if joy:
                pygame.event.pump()
                fwd    =  joy.get_axis(1)
                strafe =  joy.get_axis(0)
                yaw    =  joy.get_axis(3)
                vert   =  joy.get_axis(4)
                # cam: axis 5 (up trigger, rest=1) vs axis 2 (down trigger, rest=1)
                cam_up   = (1.0 - joy.get_axis(5)) / 2.0   # 0 at rest, +1 fully pressed
                cam_down = (1.0 - joy.get_axis(2)) / 2.0
                cam = cam_up - cam_down
                for i in range(min(16, joy.get_numbuttons())):
                    if joy.get_button(i):
                        buttons |= (1 << i)

            try:
                sock.sendall(pack_command(fwd, strafe, yaw, vert, cam, buttons))
            except Exception as e:
                print(f'\nSend error: {e}')
                break

            display()

            elapsed = time.time() - t0
            sleep_t = interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print('\nStopped.')
    finally:
        sock.close()
        if HAVE_PYGAME:
            pygame.quit()


if __name__ == '__main__':
    main()
