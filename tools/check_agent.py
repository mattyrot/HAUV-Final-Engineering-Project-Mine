#!/usr/bin/env python3
import paramiko, time

HOST, USER, PASS = "192.168.168.101", "up", "qwerty"

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(HOST, username=USER, password=PASS)

# Capture agent screen output
_, out, _ = client.exec_command("screen -S micro_ros -X hardcopy /tmp/agent_log.txt; cat /tmp/agent_log.txt")
print("=== micro_ros agent screen ===")
print(out.read().decode())

# Check if agent process is alive
_, out, _ = client.exec_command("pgrep -a micro_ros_agent")
print("=== agent process ===")
print(out.read().decode())

# Check esp32 topics
_, out, _ = client.exec_command("source /opt/ros/foxy/setup.bash && timeout 3 ros2 topic hz /esp32/bno055_data 2>&1 || echo 'topic check done'")
print("=== topic hz ===")
print(out.read().decode())

client.close()
