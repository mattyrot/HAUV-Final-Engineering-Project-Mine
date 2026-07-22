#!/bin/bash
# ============================================================================
#  hauv — HAUV ROS2 management tool  (runs on the UP Board)
#  Replaces start_nodes.sh / stop_nodes.sh.
#
#  Usage:
#    ./hauv.sh start   [--ethernet|--acoustic]   Start the stack (default: ethernet)
#    ./hauv.sh stop                              Stop everything
#    ./hauv.sh restart [--ethernet|--acoustic]   Stop then start
#    ./hauv.sh check                             Verify nodes + sensor topics live
#    ./hauv.sh status                            Show running screens
#    ./hauv.sh help                              Topic abbreviations + troubleshooting
#
#  QGC mode:
#    --ethernet : tethered — mavlink_bridge_node; QGC -> 192.168.168.101:14550
#    --acoustic : untethered — acoustic_bridge_node; run acoustic_qgc_bridge.py on the PC
# ============================================================================

set -o pipefail   # not nounset: ROS setup.bash references unset vars

WS=~/rov_ws
SP=$WS/install/autopilot_pkg/lib/python3.8/site-packages/autopilot_pkg
GPS_SP=$WS/install/gps_pkg/lib/python3.8/site-packages/gps_pkg
ENV="source /opt/ros/foxy/setup.bash && source $WS/install/setup.bash"

if [ -t 1 ]; then
  RED=$'\e[31m'; GRN=$'\e[32m'; YLW=$'\e[33m'; CYN=$'\e[36m'; BLD=$'\e[1m'; DIM=$'\e[2m'; RST=$'\e[0m'
else RED=; GRN=; YLW=; CYN=; BLD=; DIM=; RST=; fi

# ── Sensor / topic registry — the single source of truth ─────────────────────
#   ABBREV | TOPIC | TYPE | MIN_HZ (0 = variable/optional) | SOURCE | DESCRIPTION
TOPICS=(
  "IMU|/esp32/bno055_data|geometry_msgs/Twist|5|micro_ros (ESP32)|BNO055 orientation — linear.x=yaw/heading, y=pitch, z=roll (deg)"
  "DEPTH|/esp32/bar100_data|geometry_msgs/Vector3|5|micro_ros (ESP32)|BAR100 — x=depth(m), y=pressure(mbar), z=temp(C)"
  "ENV|/esp32/bme280_data|geometry_msgs/Vector3|5|micro_ros (ESP32)|BME280 cabin air — x=temp(C), y=pressure(hPa), z=humidity(%)"
  "LEAK|/esp32/leak|std_msgs/Float64|1|micro_ros (ESP32)|Leak probe — 0=dry, 1=LEAK!"
  "DVLV|/dvl/velocity_data|geometry_msgs/Twist|1|dvl_node|DVL velocity — x=E, y=N, z=U (m/s); only with bottom lock"
  "RANGE|/dvl/range_to_bottom|std_msgs/Float64|1|dvl_node|DVL range to seabed (m); only with bottom lock"
  "GPS|/gps/fix|sensor_msgs/NavSatFix|1|gps_node|u-blox NEO-M8N fix (lat/lon/alt)"
  "USBLV|/subsonus/velocity|geometry_msgs/Twist|0|subsonus_node|Subsonus USBL velocity (variable rate)"
  "USBLO|/subsonus/orientation|geometry_msgs/Twist|0|subsonus_node|Subsonus USBL orientation"
  "USBLG|/subsonus/gps|sensor_msgs/NavSatFix|0|subsonus_node|Subsonus surface GPS"
  "MOTOR|/motor_data|geometry_msgs/Twist|10|guidance_node|Motor PWM — lin.x/y/z=M1-3, ang.x/y/z=M4-6 (1100-1900)"
  "LIGHT|/lights_servo_data|geometry_msgs/Vector3|1|guidance_node|x=light1 PWM, y=light2 PWM, z=cam servo angle"
  "HSTAT|/health/status|std_msgs/UInt16|1|health_monitor|Per-sensor OK bitmask"
  "JOY|/joy|sensor_msgs/Joy|0|joy_node / mavlink_bridge|Joystick / QGC (ethernet) commands"
  "JOYA|/joy_acoustic|sensor_msgs/Joy|0|acoustic_bridge|Acoustic QGC commands"
)

# ── helpers ──────────────────────────────────────────────────────────────────

find_esp32_port() {
  for dev in /dev/ttyUSB*; do
    if udevadm info "$dev" 2>/dev/null | grep -q "ID_VENDOR=Silicon_Labs"; then
      echo "$dev"; return; fi
  done
}

launch() {  # launch <screen_name> <command string>
  screen -dmS "$1" bash -c "$ENV && $2"
  sleep 1
}

abbrev_to_topic() {  # <ABBREV|/topic> -> topic path (empty if unknown)
  case "$1" in /*) echo "$1"; return;; esac
  local up; up=$(echo "$1" | tr '[:lower:]' '[:upper:]')
  for e in "${TOPICS[@]}"; do
    IFS='|' read -r ab topic _ <<< "$e"
    [ "$ab" = "$up" ] && { echo "$topic"; return; }
  done
}

topic_to_abbrev() {  # /topic -> ABBREV (empty if not in registry)
  for e in "${TOPICS[@]}"; do
    IFS='|' read -r ab topic _ <<< "$e"
    [ "$topic" = "$1" ] && { echo "$ab"; return; }
  done
}

ros_env() { source /opt/ros/foxy/setup.bash >/dev/null 2>&1; source "$WS/install/setup.bash" >/dev/null 2>&1; }

# ── start ────────────────────────────────────────────────────────────────────

do_start() {
  local mode="$1"
  echo "${BLD}Starting HAUV — QGC mode: ${CYN}${mode}${RST}"
  echo ""

  local port; port=$(find_esp32_port)
  if [ -z "$port" ]; then
    echo "${RED}ERROR: ESP32 not found on any /dev/ttyUSB* (Silicon Labs CP2102).${RST}"
    echo "  Is it plugged in?  Run './hauv.sh help' for troubleshooting."
    return 1
  fi
  echo "ESP32 detected on ${GRN}${port}${RST}"

  echo "1. micro-ROS agent (ESP32 sensors)"
  # Reset the ESP32 with esptool BEFORE the agent takes the port. This is the
  # ONLY software reset this board honours - verified 2026-07-20:
  #   * bare DTR pulse (what this script used to do) -> no reset at all
  #   * DTR asserted during reset -> boots into the DOWNLOAD bootloader:
  #     silent, solid LED, no data
  #   * esptool --after hard_reset -> boots the app correctly
  # Without this, "hauv.sh restart" leaves the ESP32 unreachable until someone
  # presses the physical RST button (micro-ROS cannot re-establish a session
  # after its agent disappears). See CLAUDE.md.
  # Make 'start' idempotent. Running it twice used to leave TWO guidance nodes
  # both publishing /motor_data (seen as 40 Hz instead of 20 Hz) - two things
  # fighting over the thrusters. A stale agent also holds /dev/ttyUSB0, which
  # makes the esptool reset below fail silently and the ESP32 never connects.
  if screen -ls 2>/dev/null | grep -qE '\.(micro_ros|guidance|dvl|gps|subsonus|health|video|mavlink|acoustic)'; then
    echo "   stack already running - stopping it first"
    do_stop >/dev/null 2>&1
    sleep 2
  fi

  ESPTOOL=$(ls $HOME/.arduino15/packages/esp32/tools/esptool_py/*/esptool.py 2>/dev/null | head -1)
  if [ -n "$ESPTOOL" ]; then
    echo "   resetting ESP32 (esptool)..."
    if python3 "$ESPTOOL" --chip esp32 --port "$port" --after hard_reset chip_id >/tmp/esptool.log 2>&1; then
      sleep 8    # setup() runs sensor init before the app starts talking
    else
      echo "   ESP32 reset FAILED (see /tmp/esptool.log) - press its RST button"
      tail -2 /tmp/esptool.log | sed 's/^/     /'
    fi
  else
    echo "   esptool not found - if the ESP32 does not appear, press its RST button"
  fi
  launch micro_ros "ros2 run micro_ros_agent micro_ros_agent serial -b 115200 --dev $port"
  sleep 6

  echo "2. guidance_node";       launch guidance "python3 $SP/guidance_node.py"
  echo "3. dvl_node";            launch dvl      "python3 $SP/dvl_node.py"
  echo "4. gps_node";            launch gps      "python3 $GPS_SP/gps_node.py"
  echo "5. subsonus_node";       launch subsonus "python3 $SP/subsonus_node.py"
  echo "6. health_monitor_node"; launch health   "python3 $SP/health_monitor_node.py"
  echo "7. video stream -> QGC (UDP 5600)"
  screen -dmS video bash -c "gst-launch-1.0 -e v4l2src device=/dev/video0 ! image/jpeg,width=1280,height=720,framerate=30/1 ! jpegdec ! videoconvert ! videoflip method=rotate-180 ! x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast ! rtph264pay config-interval=1 pt=96 ! udpsink host=192.168.168.100 port=5600" 2>/dev/null
  sleep 1

  if [ "$mode" = "acoustic" ]; then
    echo "8. acoustic_bridge_node (QGC over acoustic)"
    launch acoustic "python3 $SP/acoustic_bridge_node.py"
    echo ""
    echo "${YLW}Acoustic mode:${RST} now run this on the PC (only ONE instance):"
    echo "   ${DIM}python tools/acoustic_qgc_bridge.py${RST}"
    echo "   Point QGC at 127.0.0.1:14551 (auto-connect UDP disabled)."
  else
    echo "8. mavlink_bridge_node (QGC over ethernet)"
    launch mavlink "python3 -c 'from mavlink_bridge_pkg.mavlink_bridge_node import main; main()'"
    echo ""
    echo "${YLW}Ethernet mode:${RST} connect QGC to 192.168.168.101:14550 (or UDP auto-connect)."
  fi

  echo ""
  echo "${GRN}Started.${RST}  Verify with:  ${BLD}./hauv.sh check${RST}"
}

# ── stop ─────────────────────────────────────────────────────────────────────

do_stop() {
  echo "Stopping all HAUV nodes..."
  pkill screen 2>/dev/null || true
  pkill gst-launch-1.0 2>/dev/null || true
  sleep 1
  for n in guidance_node dvl_node gps_node health_monitor_node subsonus_node \
           acoustic_bridge_node mavlink_bridge_node micro_ros_agent joy_node camera_node; do
    killall "$n" 2>/dev/null || true
  done
  echo "${GRN}All nodes stopped.${RST}"
}

# ── status ───────────────────────────────────────────────────────────────────

do_status() {
  echo "${BLD}Screen sessions:${RST}"
  screen -ls | grep -E '\.(micro_ros|guidance|dvl|gps|subsonus|health|video|mavlink|acoustic)' \
    || echo "  ${YLW}(none running)${RST}"
}

# ── check — verify sensor topics are actually publishing ─────────────────────

do_check() {
  echo "${BLD}HAUV system check${RST}"
  echo ""
  do_status
  echo ""
  echo "${BLD}Sensor topics (sampling 4 s)...${RST}"

  local specs=""
  for e in "${TOPICS[@]}"; do
    IFS='|' read -r ab topic type minhz src desc <<< "$e"
    specs+="('$ab','$topic','$type',$minhz,'$src'),"
  done

  # Run the checker with the ROS environment sourced (needs rclpy)
  ( source /opt/ros/foxy/setup.bash >/dev/null 2>&1
    source "$WS/install/setup.bash" >/dev/null 2>&1
    python3 - "$specs" <<'PYEOF'
import sys, time, importlib, rclpy
from rclpy.node import Node
specs = eval('[' + sys.argv[1] + ']')
try:
    tty = sys.stdout.isatty()
except Exception:
    tty = False
G='\033[32m' if tty else ''; R='\033[31m' if tty else ''
Y='\033[33m' if tty else ''; D='\033[2m' if tty else ''; X='\033[0m' if tty else ''
counts = {s[1]: 0 for s in specs}
rclpy.init()
n = Node('hauv_check')
for ab, topic, typ, minhz, src in specs:
    pkg, msg = typ.split('/')
    cls = getattr(importlib.import_module(pkg + '.msg'), msg)
    n.create_subscription(cls, topic,
        (lambda t: (lambda m: counts.__setitem__(t, counts[t] + 1)))(topic), 10)
t0 = time.time()
while time.time() - t0 < 4.0:
    rclpy.spin_once(n, timeout_sec=0.1)
missing = []
for ab, topic, typ, minhz, src in specs:
    hz = counts[topic] / 4.0
    if hz > 0 and (minhz == 0 or hz >= minhz):
        print(f"  {G}[ OK ]{X} {ab:6s} {hz:5.1f} Hz  {D}{topic}{X}")
    elif hz > 0:
        print(f"  {Y}[LOW ]{X} {ab:6s} {hz:5.1f} Hz  {D}{topic}  (expect >= {minhz}){X}")
    elif minhz == 0:
        print(f"  {D}[ -- ]{X} {ab:6s}  no data  {D}{topic}  (optional / variable){X}")
    else:
        print(f"  {R}[FAIL]{X} {ab:6s}  no data  {topic}  <- from {src}")
        missing.append((ab, src))
print()
if missing:
    print(f"{R}Missing sensors:{X}")
    srcs = sorted(set(s for _, s in missing))
    for s in srcs:
        abbrevs = ', '.join(ab for ab, sr in missing if sr == s)
        print(f"  {abbrevs}  <- check '{s}'")
    print(f"\nRun {D}./hauv.sh help{X} for step-by-step troubleshooting.")
else:
    print(f"{G}All expected topics are publishing.{X}")
rclpy.shutdown()
PYEOF
  )
}

# ── topics / echo / view shortcuts ───────────────────────────────────────────

do_topics() {
  echo "${BLD}Live topics${RST} ${DIM}(abbreviation shown where known)${RST}"
  local list
  list=$( ros_env; ros2 topic list 2>/dev/null | sort )
  if [ -z "$list" ]; then
    echo "  ${YLW}(no topics — are the nodes up? try ./hauv.sh check)${RST}"; return
  fi
  while IFS= read -r t; do
    printf "  ${CYN}%-7s${RST} %s\n" "$(topic_to_abbrev "$t")" "$t"
  done <<< "$list"
}

do_echo() {
  local q="${1:-}"
  if [ -z "$q" ]; then
    echo "Usage: ./hauv.sh echo <ABBREV|/topic>   e.g.  ./hauv.sh echo IMU"; return 1
  fi
  local topic; topic=$(abbrev_to_topic "$q")
  if [ -z "$topic" ]; then
    echo "${RED}Unknown topic/abbrev '$q'.${RST}  See './hauv.sh topics' or './hauv.sh help'."; return 1
  fi
  echo "${DIM}echo $topic  —  Ctrl-C to stop${RST}"
  # Direct rclpy subscriber (ros2 topic echo is unreliable on this box's DDS).
  ( ros_env
    python3 -u - "$topic" <<'PYEOF'
import sys, time, importlib, rclpy
from rclpy.node import Node
from rosidl_runtime_py import message_to_yaml
topic = sys.argv[1]
rclpy.init(); n = Node('hauv_echo')
cls = None; t0 = time.time()
while cls is None and time.time() - t0 < 5.0:
    for tp, types in n.get_topic_names_and_types():
        if tp == topic and types:
            pkg, _, msg = types[0].split('/')
            cls = getattr(importlib.import_module(pkg + '.msg'), msg); break
    rclpy.spin_once(n, timeout_sec=0.2)
if cls is None:
    print(f'No publisher found for {topic}.'); sys.exit(1)
n.create_subscription(cls, topic, lambda m: print(message_to_yaml(m), '---'), 10)
try: rclpy.spin(n)
except KeyboardInterrupt: pass
PYEOF
  )
}

do_view() {
  local name="${1:-}"
  if [ -z "$name" ]; then
    do_status
    echo ""
    echo "Usage: ./hauv.sh view <name>   e.g.  ./hauv.sh view mavlink"
    echo "${DIM}(inside a screen: press Ctrl-A then D to detach and leave it running)${RST}"
    return
  fi
  screen -r "$name"
}

# ── help ─────────────────────────────────────────────────────────────────────

do_help() {
  cat <<EOF
${BLD}HAUV management tool${RST}

${BLD}COMMANDS${RST}
  ${CYN}start${RST} [--ethernet|--acoustic]   Start the full stack (default: ethernet)
  ${CYN}stop${RST}                            Stop all nodes
  ${CYN}restart${RST} [--ethernet|--acoustic] Stop, then start
  ${CYN}check${RST}                           Verify nodes + every sensor topic is live
  ${CYN}status${RST}                          List running screen sessions
  ${CYN}topics${RST}                          List live topics (with abbreviations)
  ${CYN}echo${RST} <ABBREV|/topic>            Print live values, e.g. 'echo IMU' (Ctrl-C to stop)
  ${CYN}view${RST} <screen>                   Attach to a node's screen log, e.g. 'view mavlink'
                                  (detach with Ctrl-A then D)
  ${CYN}help${RST}                            This page

${BLD}QGC CONNECTION MODES${RST}
  ${CYN}--ethernet${RST}  Tethered. Starts mavlink_bridge_node. In QGC connect to
              192.168.168.101:14550 (or enable UDP auto-connect). Full 20 Hz
              telemetry + video.
  ${CYN}--acoustic${RST}  Untethered. Starts acoustic_bridge_node. On the PC run
              (ONE only) 'python tools/acoustic_qgc_bridge.py' and point QGC at
              127.0.0.1:14551 with UDP auto-connect DISABLED. Compact telemetry,
              no video.

${BLD}SENSOR / TOPIC ABBREVIATIONS${RST}
EOF
  for e in "${TOPICS[@]}"; do
    IFS='|' read -r ab topic type minhz src desc <<< "$e"
    printf "  ${CYN}%-6s${RST} ${DIM}%-26s${RST} %s\n" "$ab" "$topic" "$desc"
  done
  cat <<EOF

${BLD}INSPECT A TOPIC${RST} (use the topic path from above)
  ros2 topic echo /esp32/bno055_data      # print live values
  ros2 topic hz   /esp32/bno055_data      # measure rate (FRQ)
  ros2 topic list                         # everything currently advertised
  ros2 node list                          # running nodes
  ${DIM}(If the CLI shows nothing though nodes are up: 'ros2 daemon stop && ros2 daemon start')${RST}

${BLD}TROUBLESHOOTING — no sensor data?${RST}
  ${YLW}IMU / DEPTH / ENV / LEAK missing (all ESP32 sensors):${RST}
    1. Is the ESP32 plugged in?   ls /dev/ttyUSB*
    2. Is micro-ROS running?      ./hauv.sh status   (look for 'micro_ros')
    3. Restart just the ESP32 link:  ./hauv.sh stop && ./hauv.sh start
    4. Stuck I2C bus (data was flowing then stopped): the DTR reset in 'start'
       usually clears it; health_monitor also auto-resets after 20 s of both
       ESP32 I2C sensors being dead.
    5. Check the port owner:      lsof | grep /dev/ttyUSB0

  ${YLW}GPS missing:${RST}  u-blox is a separate USB device — check gps_node in status,
    and see 'fix=3 sats=N' in its log (needs sky view; indoors sats are few).

  ${YLW}DVL (DVLV/RANGE) missing:${RST}  normal out of water — the DVL only publishes
    with bottom lock. Also the DVL must be pinging (TCP CS command / web UI PD6).

  ${YLW}USBL (Subsonus) missing:${RST}  the unit trickles at ~0.25 Hz and must be
    powered + on the network (192.168.168.103). subsonus_node requests packet 20.

  ${YLW}MOTOR missing:${RST}  guidance_node not running — ./hauv.sh start.

  ${YLW}Telemetry flickers in QGC (acoustic):${RST}  you have TWO
    acoustic_qgc_bridge.py running — kill all but one.
EOF
}

# ── main ─────────────────────────────────────────────────────────────────────

MODE="ethernet"
for a in "$@"; do
  case "$a" in
    --acoustic) MODE="acoustic" ;;
    --ethernet) MODE="ethernet" ;;
  esac
done

case "${1:-help}" in
  start)   do_start "$MODE" ;;
  stop)    do_stop ;;
  restart) do_stop; sleep 1; do_start "$MODE" ;;
  check)   do_check ;;
  status)  do_status ;;
  topics|list) do_topics ;;
  echo)    do_echo "${2:-}" ;;
  view)    do_view "${2:-}" ;;
  help|-h|--help) do_help ;;
  *) echo "Unknown command '${1}'"; echo; do_help; exit 1 ;;
esac
