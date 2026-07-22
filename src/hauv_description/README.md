# hauv_description

URDF model, meshes and RViz2 visualisation for the HAUV.

**Visualisation only — there is no physics here.** It displays measured state:
the thrusters spin to match what `guidance_node` is commanding, and the hull
rolls, pitches, turns and sinks to match the real IMU and depth sensor. For
thrust → motion you need Gazebo (on Foxy that means Gazebo Classic 11 with
hand-written buoyancy and thruster plugins).

## Running it

```bash
# live - mirrors the real vehicle
DISPLAY=:0 ros2 launch hauv_description display.launch.py

# offline - no vehicle needed
DISPLAY=:0 ros2 launch hauv_description display.launch.py live:=false

# headless - no RViz, just the model + TF (useful for checking the pipeline)
ros2 launch hauv_description display.launch.py rviz:=false
```

Over SSH there is no `DISPLAY`, so prefix with `DISPLAY=:0` to use the UP
Board's own screen, or RViz aborts with "cannot connect to X server".

| Argument | Default | Meaning |
|---|---|---|
| `live` | `true` | Hull pose from the real IMU + depth sensor |
| `rviz` | `true` | Also start RViz2 |
| `zero_on_start` | `true` | Treat the first pitch/roll reading as level |

## What each node does

| Node | Job |
|---|---|
| `robot_state_publisher` | Turns the URDF + `/joint_states` into the TF tree |
| `motor_to_joint_states` | `/motor_data` (PWM) → `/joint_states`, so the thrusters spin |
| `attitude_to_tf` | Real BNO055 attitude + BAR100 depth → `world`→`base_link` TF |

`motor_to_joint_states` runs in **both** modes. It publishes at 30 Hz whether or
not `/motor_data` is arriving, so the six continuous joints always have a
transform. That is deliberate — `joint_state_publisher` is **not installed** on
this box, and this node covers the same need.

`zero_on_start` matters in practice: the vehicle is rarely sitting flat on the
bench (it read pitch −17.6°, roll 72.8° during testing), and without levelling
the model looks permanently capsized. Yaw is never zeroed — it is a real heading.

If the model climbs when the vehicle dives, set `invert_depth:=true` on
`attitude_to_tf`. Depth is positive-down and world Z is up, so `z = -depth` is
the expected mapping and was confirmed correct during testing.

## Two install quirks on this box

**1. colcon puts the entry points in the wrong place.** It installs
`console_scripts` to `install/hauv_description/bin/` instead of
`install/hauv_description/lib/hauv_description/`, which breaks both `ros2 run`
and `Node(executable=...)` in the launch file. Same quirk CLAUDE.md documents
for `mavlink_bridge_pkg`. After every `colcon build`:

```bash
D=~/rov_ws/install/hauv_description
mkdir -p $D/lib/hauv_description
cp -f $D/bin/motor_to_joint_states $D/bin/attitude_to_tf $D/lib/hauv_description/
chmod +x $D/lib/hauv_description/*
```

**2. `rviz2` cannot render on the UP Board's own screen (`DISPLAY=:0`).**
Run it over X forwarding to a PC instead — that works.

It segfaults on `:0`. A gdb backtrace pins it exactly:

```
#3  Ogre::GLRenderSystem::GLRenderSystem()      <- legacy GL render system ctor
#2  … std::string::operator=  -> SEGFAULT
```

GLX context creation on the local display fails first (`XCB error 148, major
code 140`), so `glGetString` hands Ogre a **NULL**, and assigning NULL to a
`std::string` segfaults. It is not the model, the meshes, the triangle count or
memory — an empty scene with `LIBGL_ALWAYS_SOFTWARE=1` crashes identically.
`QT_X11_NO_MITSHM`, `QT_XCB_GL_INTEGRATION=none` and `QT_OPENGL=software` do not
help, and reinstalling all five rviz packages changed nothing. `rviz_rendering`
hardcodes `RenderSystem_GL`, so the working `RenderSystem_GL3Plus.so` sitting
next to it cannot be selected by configuration.

**The fix — X forwarding.** Start an X server on the PC (VcXsrv / XLaunch), then:

```bash
# from the PC, with the X server running
DISPLAY=localhost:0.0 ssh -Y up@192.168.168.101
# then on the board:
source /opt/ros/foxy/setup.bash && source ~/rov_ws/install/setup.bash
ros2 launch hauv_description display.launch.py
```

Verified working: RViz opens on the PC and the model tracks the live vehicle.
VcXsrv's GLX returns proper driver strings, so Ogre initialises normally.

Keep the SSH session open — the X tunnel dies with it. Note this pushes all
geometry over indirect GLX, so the 2 M triangles are being sent across the
network; if it drags, decimate the meshes (see below).

Caveat for a *headless* board: with no monitor and no forwarding there is no
display at all, and RViz cannot run. `rviz2 --help` also fails in that case —
it initialises Qt even for `--help`, so "`--help` aborts" means "no DISPLAY",
**not** "broken binary".

## Verified headless

With `rviz:=false` the whole pipeline checks out on the vehicle:

- `robot_state_publisher` loads all 16 segments from the URDF
- `/joint_states` carries all six thruster joints
- `world`→`base_link` TF tracks the real sensors — TF `z = +0.269` against a raw
  depth reading of `−0.269`, and rotation ≈ identity after levelling

So when RViz runs anywhere, it will just work. Because ROS 2 is networked, RViz
does **not** have to run on the vehicle — any machine on the same
`ROS_DOMAIN_ID` can subscribe to `/robot_description`, `/joint_states` and `/tf`
and render the model on a decent GPU. That is the better arrangement anyway: the
UP Board is a weak, headless vehicle computer.

## Model notes

Frame: **X forward, Y port, Z up**, origin on the pressure-tube axis, centred
fore-aft — the same frame the CAD and the thrust-allocation matrix use.

16 links, 15 joints, single root `base_link`. Six `continuous` thruster joints
about their own +X; lights, Subsonus, Bar30 and the indicator are `fixed`;
`camera_link` and `dvl_link` are empty reference frames.

Meshes are exported from Fusion in **millimetres**, so every one carries
`scale="0.001 0.001 0.001"`. Delete that and the vehicle renders 1000× too big.

**Size:** ~29 MB of STL, ~2.03 M triangles (T200 186k × 6, Lumen 164k × 4,
Subsonus 200k). That was never reached during testing because RViz crashed
first, so its real frame rate here is still unknown. If it drags once RViz
works, decimate those three vendor meshes in Blender (Decimate modifier, ~0.1
ratio) — visual meshes do not need machining fidelity.

**The thruster mesh is the whole T200**, so spinning a joint turns the duct with
the rotor. Splitting the propeller bodies out of `T200.STEP` into their own mesh
and child link would fix it — a small export job, not a redesign.
