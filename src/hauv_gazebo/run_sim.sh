#!/bin/bash
# Run the HAUV Gazebo simulation.
#
#   wsl -d Ubuntu-Foxy -- bash "/mnt/c/Users/matma/Desktop/rov_ws - Copilot/src/hauv_gazebo/run_sim.sh"          # with GUI
#   wsl -d Ubuntu-Foxy -- bash ".../run_sim.sh" headless                                                          # no GUI
#
# GUI needs an X server on Windows (VcXsrv / XLaunch) with "Disable access
# control" ticked - WSLg is not present on this machine, so DISPLAY has to point
# at the Windows host explicitly.
set -e

WS="$HOME/hauv_sim_ws"
MODE="${1:-gui}"

if [ ! -f "$WS/install/setup.bash" ]; then
  echo "Workspace not built yet - run wsl_build.sh first."
  exit 1
fi

source /opt/ros/foxy/setup.bash
source "$WS/install/setup.bash"

if [ "$MODE" = "headless" ]; then
  GUI=false
  echo "Running HEADLESS (no window)."
else
  GUI=true
  # WSL2 sees Windows as the nameserver address.
  HOST_IP=$(grep -m1 nameserver /etc/resolv.conf | awk '{print $2}')
  export DISPLAY="${HOST_IP}:0"
  export LIBGL_ALWAYS_INDIRECT=0
  echo "Running with GUI on DISPLAY=$DISPLAY"
  if ! timeout 5 xdpyinfo >/dev/null 2>&1; then
    echo ""
    echo "  !! Cannot reach an X server at $DISPLAY."
    echo "     Start VcXsrv (XLaunch) on Windows with 'Disable access control'"
    echo "     ticked, then run this again - or use 'headless'."
    echo ""
    exit 1
  fi
  echo "X server reachable."
fi

# Anything left over from a previous run will fight for the topics.
pkill -x gzserver 2>/dev/null || true
pkill -x gzclient 2>/dev/null || true
sleep 1

echo ""
echo "Ctrl-C to stop."
echo "Topics: /motor_data (in)  /esp32/bno055_data /esp32/bar100_data /gps/fix (out)"
echo ""
exec ros2 launch hauv_gazebo hauv_sim.launch.py gui:=$GUI
