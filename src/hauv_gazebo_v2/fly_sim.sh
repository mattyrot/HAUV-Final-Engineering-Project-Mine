#!/bin/bash
# Fly the simulated HAUV yourself, from the keyboard.
#
# RUN THIS IN A WSL TERMINAL, not through a tool or a pipe - it needs a real
# terminal to read keys from:
#
#     wsl -d Ubuntu-Foxy
#     bash "/mnt/c/Users/matma/Desktop/rov_ws - Copilot/src/hauv_gazebo_v2/fly_sim.sh"
#     bash ".../fly_sim.sh" headless      # no Gazebo window
#
# Brings up gzserver (+ gzclient), both bridges and guidance_node, then hands
# you the controls. Ctrl-C or q stops everything.
#
# Deliberately no `set -u`: /opt/ros/foxy/setup.bash reads AMENT_TRACE_SETUP_FILES
# unset and would abort the script.
set -o pipefail

WS="$HOME/hauv_sim_ws"
MODE="${1:-gui}"
SPAWN_Z="${2:--3.0}"
LOGS="/tmp/hauv_sim_results_v2"
mkdir -p "$LOGS"

if [ ! -f "$WS/install/setup.bash" ]; then
  echo "Workspace not built - run wsl_build.sh first."
  exit 1
fi
source /opt/ros/foxy/setup.bash
source "$WS/install/setup.bash"

GUI=false
if [ "$MODE" != "headless" ]; then
  HOST_IP=$(grep -m1 nameserver /etc/resolv.conf | awk '{print $2}')
  export DISPLAY="${HOST_IP}:0"
  export LIBGL_ALWAYS_INDIRECT=0
  if timeout 5 xdpyinfo >/dev/null 2>&1; then
    GUI=true
    echo "GUI on DISPLAY=$DISPLAY"
  else
    echo "No X server at $DISPLAY - running headless."
    echo "(Start VcXsrv with 'Disable access control' ticked to see the window.)"
  fi
fi

cleanup() {
  kill "$LAUNCH_PID" "$GUID_PID" 2>/dev/null
  pkill -x gzserver 2>/dev/null
  pkill -x gzclient 2>/dev/null
  pkill -f 'thruster_bridge' 2>/dev/null
  pkill -f 'sensor_bridge' 2>/dev/null
  pkill -f 'guidance_node' 2>/dev/null
  pkill -f 'robot_state_publisher' 2>/dev/null
}
LAUNCH_PID=""; GUID_PID=""
trap cleanup EXIT INT TERM

pkill -x gzserver 2>/dev/null
pkill -f 'guidance_node' 2>/dev/null
sleep 2

echo "[1/3] simulation (spawn_z=$SPAWN_Z)"
ros2 launch hauv_gazebo_v2 hauv_sim.launch.py \
     gui:="$GUI" spawn_z:="$SPAWN_Z" > "$LOGS/fly.sim.log" 2>&1 &
LAUNCH_PID=$!
echo -n "      waiting for spawn "
for i in $(seq 1 60); do
  grep -q 'Successfully spawned' "$LOGS/fly.sim.log" 2>/dev/null && { echo " ok (${i}s)"; break; }
  kill -0 "$LAUNCH_PID" 2>/dev/null || { echo " LAUNCH DIED"; tail -20 "$LOGS/fly.sim.log"; exit 1; }
  echo -n "."; sleep 1
done

sleep 4
echo "[2/3] guidance_node"
ros2 run autopilot_pkg guidance_node > "$LOGS/fly.guidance.log" 2>&1 &
GUID_PID=$!
sleep 3
kill -0 "$GUID_PID" 2>/dev/null || { echo "guidance died:"; cat "$LOGS/fly.guidance.log"; exit 1; }

if [ "$GUI" = "true" ]; then
  echo "      tip: right-click the vehicle in Gazebo -> Follow, so it stays in shot"
fi
echo "[3/3] your controls:"
exec ros2 run hauv_gazebo_v2 teleop_key
