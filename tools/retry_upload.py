#!/usr/bin/env python3
import paramiko, sys, time

HOST = "192.168.168.101"
USER = "up"
PASS = "qwerty"
SKETCH = "/home/up/rov_ws/src/esp_sketches/rov_esp_main/rov_esp_main.ino"
FQBN = "esp32:esp32:esp32da"
PORT = "/dev/ttyUSB1"

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

# Kill anything holding the port
run(client, "screen -X -S micro_ros quit 2>/dev/null; pkill -9 -f micro_ros_agent 2>/dev/null; pkill -9 -f arduino-cli 2>/dev/null; sleep 2")

# Upload
print("Uploading...")
out, err = run(client, f"arduino-cli upload -p {PORT} --fqbn {FQBN} {SKETCH} 2>&1", timeout=90)

if "error" in out.lower() or "error" in err.lower():
    print("Upload may have failed, check output above.")
else:
    print("Upload likely succeeded.")

# Restart agent
run(client, f"screen -dmS micro_ros bash -c 'source /opt/ros/foxy/setup.bash && source /home/up/rov_ws/install/setup.bash && ros2 run micro_ros_agent micro_ros_agent serial -b 115200 --dev {PORT}'")
print("Agent restarted. Press ESP32 reset button.")

client.close()
