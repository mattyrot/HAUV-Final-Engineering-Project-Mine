#!/bin/bash
# Fly guidance_node in the simulation and check what it does.
#
#   wsl -d Ubuntu-Foxy -- bash ".../run_guidance_test.sh" manual
#   wsl -d Ubuntu-Foxy -- bash ".../run_guidance_test.sh" depth_hold
#   wsl -d Ubuntu-Foxy -- bash ".../run_guidance_test.sh" goto
#   wsl -d Ubuntu-Foxy -- bash ".../run_guidance_test.sh" leak
#
# Add "gui" as a second argument to watch it happen:
#   wsl -d Ubuntu-Foxy -- bash ".../run_guidance_test.sh" goto gui
# That needs VcXsrv running on Windows with "Disable access control" ticked -
# there is no WSLg on this machine, so DISPLAY has to point at the host.
#
# Brings up gzserver + both bridges + guidance_node + sim_pilot, waits for the
# scripted run to finish, tears everything down and prints the verdict. Headless
# by default: none of these tests need rendering, and the GUI costs a core.
#
# Each scenario spawns at its own depth, which matters more than it looks:
#   depth_hold/goto at -6 m and leak at -8 m, because at the default -0.15 m the
#   vehicle parks against the surface slab at ~0.066 m, the depth error never
#   leaves GOTO_DEPTH_DEADBAND (0.3 m), and depth hold would "pass" by doing
#   nothing at all.
# Deliberately no `set -u`: /opt/ros/foxy/setup.bash reads AMENT_TRACE_SETUP_FILES
# without a default and aborts the script under it.
set -o pipefail

WS="$HOME/hauv_sim_ws"
SCENARIO="${1:-manual}"
MODE="${2:-headless}"
RESULTS="/tmp/hauv_sim_results_v2"
mkdir -p "$RESULTS"

GUI=false
if [ "$MODE" = "gui" ]; then
  # WSL2 reaches the Windows host at the nameserver address.
  HOST_IP=$(grep -m1 nameserver /etc/resolv.conf | awk '{print $2}')
  export DISPLAY="${HOST_IP}:0"
  export LIBGL_ALWAYS_INDIRECT=0
  if ! timeout 5 xdpyinfo >/dev/null 2>&1; then
    echo "No X server at $DISPLAY."
    echo "Start VcXsrv (XLaunch) with 'Disable access control' ticked and"
    echo "'Native opengl' UNticked, then run this again - or drop the 'gui' arg."
    exit 1
  fi
  GUI=true
  echo "GUI on DISPLAY=$DISPLAY"
fi

case "$SCENARIO" in
  manual)     SPAWN_Z="-3.0" ; PUB_LEAK="true"  ;;
  depth_hold) SPAWN_Z="-6.0" ; PUB_LEAK="true"  ;;
  goto)       SPAWN_Z="-6.0" ; PUB_LEAK="true"  ;;
  # sim_pilot owns /esp32/leak for this one - see the launch file comment.
  leak)       SPAWN_Z="-8.0" ; PUB_LEAK="false" ;;
  *) echo "unknown scenario '$SCENARIO' (manual|depth_hold|goto|leak)"; exit 2 ;;
esac

if [ ! -f "$WS/install/setup.bash" ]; then
  echo "Workspace not built - run wsl_build.sh first."
  exit 1
fi

source /opt/ros/foxy/setup.bash
source "$WS/install/setup.bash"

cleanup() {
  kill "$LAUNCH_PID" "$GUID_PID" 2>/dev/null
  pkill -x gzserver 2>/dev/null
  pkill -x gzclient 2>/dev/null
  pkill -f 'thruster_bridge|sensor_bridge|guidance_node|robot_state_publisher' 2>/dev/null
  sleep 1
}
LAUNCH_PID=""; GUID_PID=""
trap cleanup EXIT INT TERM

# Anything left from a previous run fights for the same topics.
pkill -x gzserver 2>/dev/null
pkill -f 'thruster_bridge|sensor_bridge|guidance_node' 2>/dev/null
sleep 2

SIM_LOG="$RESULTS/$SCENARIO.sim.log"
GUID_LOG="$RESULTS/$SCENARIO.guidance.log"
PILOT_LOG="$RESULTS/$SCENARIO.pilot.log"

echo "=== $SCENARIO : spawn_z=$SPAWN_Z publish_leak=$PUB_LEAK gui=$GUI ==="
echo "[1/4] gzserver + bridges"
ros2 launch hauv_gazebo_v2 hauv_sim.launch.py \
     gui:="$GUI" spawn_z:="$SPAWN_Z" publish_leak:="$PUB_LEAK" \
     > "$SIM_LOG" 2>&1 &
LAUNCH_PID=$!

# Wait for the model to actually exist rather than guessing at a sleep - on a
# loaded box gzserver can take a while, and starting guidance early means its
# IMU offset calibration averages 20 readings of nothing.
echo -n "      waiting for spawn "
for i in $(seq 1 60); do
  if grep -q 'Successfully spawned' "$SIM_LOG" 2>/dev/null; then
    echo " ok (${i}s)"
    break
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo " LAUNCH DIED"; tail -30 "$SIM_LOG"; exit 1
  fi
  echo -n "."
  sleep 1
done
if ! grep -q 'Successfully spawned' "$SIM_LOG" 2>/dev/null; then
  echo " TIMEOUT"; tail -30 "$SIM_LOG"; exit 1
fi

# NO `gz camera -f` HERE, DELIBERATELY. It looks like the obvious way to lock the
# view onto the vehicle, but every form of it (-c user_camera, -w underwater,
# default::user_camera) BLOCKS FOREVER against this setup - measured rc=124 under
# an 8 s timeout, same as `gz camera -l`. Dropped in unguarded it hangs the whole
# test run at this line. Follow from the GUI instead.
if [ "$GUI" = "true" ]; then
  echo "      to track the vehicle: right-click it in the GUI -> Follow"
  echo "      (a 29 m GOTO transit drives out of shot otherwise)"
fi

# Let it settle before guidance latches its IMU offsets off the first readings.
sleep 5

echo "[2/4] guidance_node"
ros2 run autopilot_pkg guidance_node > "$GUID_LOG" 2>&1 &
GUID_PID=$!
sleep 3
if ! kill -0 "$GUID_PID" 2>/dev/null; then
  echo "      guidance_node died on startup:"; cat "$GUID_LOG"; exit 1
fi

echo "[3/4] sim_pilot scenario=$SCENARIO  (this is the run)"
ros2 run hauv_gazebo_v2 sim_pilot --ros-args \
     -p scenario:="$SCENARIO" -p out_dir:="$RESULTS" 2>&1 | tee "$PILOT_LOG"
RC="${PIPESTATUS[0]}"

echo "[4/4] guidance_node said:"
grep -E 'GOTO|LEAK|Leak|envelope|IMU offset|Switched|mode' "$GUID_LOG" | tail -25

echo ""
echo "=== $SCENARIO finished, sim_pilot rc=$RC ==="
exit "$RC"
