#!/usr/bin/env python3
"""Deploy and flash the ESP32 sketch to the UP Board."""
import paramiko
import sys
import time

HOST = "192.168.168.101"
USER = "up"
PASS = "qwerty"
SKETCH = "/home/up/rov_ws/src/esp_sketches/rov_esp_main/rov_esp_main.ino"
FQBN = "esp32:esp32:esp32da"

def run(client, cmd, timeout=120):
    print(f"$ {cmd}")
    _, stdout, stderr = client.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode()
    err = stderr.read().decode()
    if out: print(out)
    if err: print(err, file=sys.stderr)
    return out, err

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(HOST, username=USER, password=PASS)
print("Connected.")

# Copy sketch
sftp = client.open_sftp()
sftp.put(
    r"C:\Users\matma\Desktop\rov_ws - Copilot\src\esp_sketches\rov_esp_main\rov_esp_main.ino",
    SKETCH
)
sftp.close()
print("Sketch uploaded.")

# Kill micro_ros_agent
run(client, "screen -S micro_ros -X quit 2>/dev/null; pkill -f micro_ros_agent; sleep 1")

# Find ESP32 port (Silicon Labs)
out, _ = run(client, "for d in /dev/ttyUSB*; do udevadm info $d 2>/dev/null | grep -q 'ID_VENDOR=Silicon_Labs' && echo $d && break; done")
esp_port = out.strip()
if not esp_port:
    print("ESP32 port not found!")
    sys.exit(1)
print(f"ESP32 on {esp_port}")

# Compile
print("Compiling...")
out, err = run(client, f"arduino-cli compile --fqbn {FQBN} {SKETCH} 2>&1", timeout=180)

# Upload
print("Uploading...")
out, err = run(client, f"arduino-cli upload -p {esp_port} --fqbn {FQBN} {SKETCH} 2>&1", timeout=60)

# Restart micro_ros_agent
run(client, f"screen -dmS micro_ros bash -c 'source /opt/ros/foxy/setup.bash && source /home/up/rov_ws/install/setup.bash && ros2 run micro_ros_agent micro_ros_agent serial -b 115200 --dev {esp_port}'")
print("micro_ros_agent restarted.")

client.close()
print("Done. Reset the ESP32 and check QGC for temperature.")
