#!/usr/bin/env python3
import paramiko, sys, time

HOST, USER, PASS = "192.168.168.101", "up", "qwerty"
FQBN = "esp32:esp32:esp32da"
SKETCH_DIR = "/tmp/bar_test"
SKETCH = f"{SKETCH_DIR}/bar_test.ino"

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(HOST, username=USER, password=PASS)

def run(cmd, timeout=120):
    print(f"$ {cmd}")
    _, o, e = client.exec_command(cmd, timeout=timeout)
    out = o.read().decode(); err = e.read().decode()
    if out: print(out)
    if err: print(err, file=sys.stderr)
    return out

# Find ESP32 port
port = run("for d in /dev/ttyUSB*; do udevadm info $d 2>/dev/null | grep -q 'ID_VENDOR=Silicon_Labs' && echo $d && break; done").strip()
print(f"ESP32 on {port}")

# Upload sketch
run("mkdir -p /tmp/bar_test; pkill -9 -f micro_ros_agent 2>/dev/null; screen -X -S micro_ros quit 2>/dev/null; sleep 2")
sftp = client.open_sftp()
sftp.put(r"C:\Users\matma\Desktop\rov_ws - Copilot\bar_test\bar_test.ino", SKETCH)
sftp.close()

# Check MS5837 lib
out = run("ls /home/up/Arduino/libraries/ | grep -i ms5837")
if not out.strip():
    print("MS5837 library not found — installing...")
    run("arduino-cli lib install 'BlueRobotics MS5837 Library'")

# Compile + upload
run(f"arduino-cli compile --fqbn {FQBN} {SKETCH_DIR} 2>&1", timeout=180)
run(f"arduino-cli upload -p {port} --fqbn {FQBN} {SKETCH_DIR} 2>&1", timeout=60)

# Read serial for 10 seconds
print("\n=== Serial output ===")
_, o, _ = client.exec_command(f"timeout 12 cat {port}", timeout=15)
time.sleep(1)
run(f"stty -F {port} 115200 raw; sleep 0.5")
_, o2, _ = client.exec_command(f"stty -F {port} 115200 raw -echo && timeout 10 cat {port}", timeout=15)
print(o2.read().decode())

client.close()
