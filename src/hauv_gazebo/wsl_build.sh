#!/bin/bash
# Build / refresh the HAUV Gazebo simulation inside WSL.
#
#   wsl -d Ubuntu-Foxy -- bash "/mnt/c/Users/matma/Desktop/rov_ws - Copilot/src/hauv_gazebo/wsl_build.sh"
#
# The package source lives in the Windows repo (so it is versioned with
# everything else), but the workspace is built on the WSL filesystem - building
# on /mnt/c goes through the 9p mount and is painfully slow.
set -e

WIN_SRC="/mnt/c/Users/matma/Desktop/rov_ws - Copilot/src"
WS="$HOME/hauv_sim_ws"

echo "[1/3] syncing sources  $WIN_SRC -> $WS/src"
mkdir -p "$WS/src"
# autopilot_pkg comes along because guidance_node is the thing under test - the
# sim exists to fly it. Its other nodes (dvl_node, subsonus_node) import
# hardware libraries that are not installed here, but that only matters if you
# run them; the package builds fine.
for pkg in hauv_gazebo autopilot_pkg; do
  rm -rf "$WS/src/$pkg"
  cp -r "$WIN_SRC/$pkg" "$WS/src/$pkg"
  # strip anything Windows-side that should not be built
  rm -rf "$WS/src/$pkg"/**/__pycache__ 2>/dev/null || true
  # CRLF in a shell/python file breaks it on Linux
  find "$WS/src/$pkg" -type f \( -name '*.py' -o -name '*.sh' \) \
       -exec sed -i 's/\r$//' {} + 2>/dev/null || true
  echo "      $pkg"
done

echo "[2/3] building"
source /opt/ros/foxy/setup.bash
cd "$WS"
colcon build --packages-select hauv_gazebo autopilot_pkg 2>&1 | tail -4

# colcon on these boxes installs console_scripts to install/<pkg>/bin/ instead
# of install/<pkg>/lib/<pkg>/, which breaks both `ros2 run` and Node(executable=)
# in launch files. Same quirk as hauv_description and mavlink_bridge_pkg.
for pkg in hauv_gazebo autopilot_pkg; do
  D="$WS/install/$pkg"
  if [ -d "$D/bin" ]; then
    mkdir -p "$D/lib/$pkg"
    cp -f "$D/bin/"* "$D/lib/$pkg/" 2>/dev/null || true
    chmod +x "$D/lib/$pkg/"* 2>/dev/null || true
    echo "      (applied colcon bin/ -> lib/ fix: $pkg)"
  fi
done

echo "[3/3] checking"
source "$WS/install/setup.bash"
echo -n "      executables: "
ros2 pkg executables hauv_gazebo 2>/dev/null | awk '{print $2}' | tr '\n' ' '
echo ""
for f in urdf/hauv_gazebo.urdf worlds/underwater.world launch/hauv_sim.launch.py; do
  p="$WS/install/hauv_gazebo/share/hauv_gazebo/$f"
  [ -f "$p" ] && echo "      OK      $f" || echo "      MISSING $f"
done

echo ""
echo "Done. To run (headless):"
echo "    source /opt/ros/foxy/setup.bash && source $WS/install/setup.bash"
echo "    ros2 launch hauv_gazebo hauv_sim.launch.py gui:=false"
