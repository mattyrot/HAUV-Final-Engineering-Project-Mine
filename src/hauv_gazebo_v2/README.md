# hauv_gazebo_v2

Gazebo Classic 11 simulation of the HAUV, wired so **`guidance_node` flies the
simulated vehicle unmodified** — it reads the same `/esp32/*` and `/gps/fix`
topics and writes the same `/motor_data` it uses on hardware.

## Where it runs

**WSL `Ubuntu-Foxy`, not the UP Board.** The UP Board is a 4-core Atom that
already sits at load ~4.4 encoding video; Gazebo would bury it, and the vehicle
computer should not be running a simulator. WSL has Gazebo 11.11, ROS 2 Foxy,
all four `gazebo_ros` plugin libraries, and 6 cores.

```bash
# build / re-sync after editing on the Windows side
wsl -d Ubuntu-Foxy -- bash "/mnt/c/Users/matma/Desktop/rov_ws - Copilot/src/hauv_gazebo_v2/wsl_build.sh"

# run headless (light, good for control testing)
source /opt/ros/foxy/setup.bash && source ~/hauv_sim_ws/install/setup.bash
ros2 launch hauv_gazebo_v2 hauv_sim.launch.py gui:=false

# with the Gazebo GUI - needs an X server (VcXsrv/XLaunch), same as RViz
DISPLAY=<your-windows-ip>:0 ros2 launch hauv_gazebo_v2 hauv_sim.launch.py
```

Source lives in the Windows repo (versioned with everything else); the build
happens on the WSL filesystem because building across `/mnt/c` is painfully slow.

| Argument | Default | Meaning |
|---|---|---|
| `gui` | `true` | Run `gzclient` |
| `bridges` | `true` | Run the thruster/sensor bridges |
| `spawn_z` | `-0.15` | Spawn depth (world Z, negative = under water) |
| `publish_leak` | `true` | Let `sensor_bridge` synthesise `/esp32/leak=0.0`. Set `false` to inject a leak yourself |

## Flying it yourself, from the keyboard

**Run this in a real WSL terminal** — it reads keys from the terminal, so it
cannot be driven through a pipe or a tool:

```bash
wsl -d Ubuntu-Foxy
bash "/mnt/c/Users/matma/Desktop/rov_ws - Copilot/src/hauv_gazebo_v2/fly_sim.sh"
bash ".../fly_sim.sh" headless        # no Gazebo window
```

That starts the sim, both bridges and `guidance_node`, then hands you:

```
 arrow up/down     forward / reverse        w / s   ascend / descend
 arrow left/right  turn left / right        a / d   strafe left / right
 space  ALL STOP        + / -  speed scale        q  quit
```

Controls are **sticky** — a key sets the thrust and it holds until you change it
or press space. A terminal reports key presses but never releases, so
hold-to-drive would stutter on the auto-repeat delay; sticky also suits a vehicle
that takes several seconds to accelerate.

There is no gamepad option, and it is not worth chasing: WSL has no
`/dev/input`, no USB passthrough and no loadable kernel modules, so `joy_node`
cannot see a controller without rebuilding the WSL kernel. The keyboard arrives
on stdin, which WSL has always had. `teleop_key` publishes `/joy` in exactly the
shape `joy_node` would, so `guidance_node` cannot tell the difference.

## Flying guidance_node

`run_guidance_test.sh` brings up the sim, both bridges, `guidance_node` and a
scripted pilot, runs one scenario, tears it all down and prints PASS/FAIL:

```bash
wsl -d Ubuntu-Foxy -- bash ".../run_guidance_test.sh" manual      # drive every axis
wsl -d Ubuntu-Foxy -- bash ".../run_guidance_test.sh" depth_hold  # hold against the buoyant trim
wsl -d Ubuntu-Foxy -- bash ".../run_guidance_test.sh" goto        # turn, transit 29 m, loiter, abort
wsl -d Ubuntu-Foxy -- bash ".../run_guidance_test.sh" leak        # auto-surface failsafe
```

All four features work: **23 of 24 checks pass**. The one failure is deliberate —
it flags a real `guidance_node` defect (out-of-range PWM on two-axis stick input,
see `DEVELOPMENT.md`), and is left failing rather than softened. Logs and a CSV
of every run land in `/tmp/hauv_sim_results_v2/`.

**There is no joystick in WSL, and `guidance_node` will not fly without one.**
Its `_last_joy_time` starts at 0.0, so `JOY_FAILSAFE_TIMEOUT` (1.5 s) zeroes
manual thrust and `GOTO_COMMS_TIMEOUT` (5 s) *aborts auto-GOTO* before you ever
see it navigate. `sim_pilot` stands in for both the joystick and QGC: it
publishes `/joy` at 20 Hz throughout, and injects `/guidance/goto_target`
exactly as `mavlink_bridge_node` does on "Go to location".

Depth-dependent scenarios spawn deep on purpose. At the default `-0.15` m the
vehicle parks against the surface slab at ~0.066 m, the depth error never leaves
`GOTO_DEPTH_DEADBAND` (0.3 m), and depth hold "passes" by doing nothing.

## What it gives you

| Topic | Direction | Notes |
|---|---|---|
| `/motor_data` | in | PWM 1100–1900 from guidance → thruster forces |
| `/esp32/bno055_data` | out | From the simulated IMU, degrees, yaw as 0–360 heading |
| `/esp32/bar100_data` | out | Depth from the model's Z (positive down) |
| `/esp32/leak` | out | Synthesised dry; `publish_leak:=false` to inject a leak by hand |
| `/gps/fix` | out | Model position → lat/lon, so auto-GOTO has something to chase |

## Measured behaviour

```
forward   0.971 m/s    yaw   ~26 deg/s (pure rotation)
reverse   0.919 m/s    vertical  1.11 m/s up / 0.86 m/s down
neutral   drifts up ~0.016 m/s (see limitations)
```

Driven through `guidance_node` at full stick (which commands yaw harder than it
commands forward — `yaw·400` on all four horizontals vs `400·cos45` for surge):

```
forward   0.99 m/s, but pitches nose-up 17.5 deg and CLIMBS at 0.30 m/s
strafe    1.00 m/s to port         yaw   136 deg/s  <-- 5x the figure above
descend   0.85 m/s, pitches nose-down 21.8 deg
```

The forward-thrust climb and the yaw discrepancy are both open — see
`DEVELOPMENT.md`.

## Five bugs found and fixed getting here

Worth reading before trusting or changing the model.

**1. It sank — 4.8 kg negative (−47 N).** The buoyancy volume was the pressure
tube's CAD volume (0.006026 m³), not the vehicle's displacement. Trimmed to
0.010802 m³ → **+0.1 kg positive**, the correct ROV direction (surfaces on power
loss). The URDF header already called this out and prescribed ~7.7 L of foam.

**2. It capsized — centre of buoyancy 8 mm *below* centre of mass.** An inverted
pendulum: pitch ran 9°→32° in 40 s and never settled. `center_of_volume` z raised
to 0.045 (33 mm above CG) → settles stably at ~3°. Also the header's WARNING 2,
and the same fix it recommends (foam mounted high).

**3. `velocity_decay` was ~30× too strong.** Gazebo applies it **per timestep,
not per second**, so `0.20` at `dt=0.001` is a ~5 ms damping constant. Full
vertical thrust produced 0.001 m/s — slower than passive drift. Retuned to
`0.006 / 0.02` for ~1 m/s terminal.

**4. Thrust was applied along the wrong axis.** The joints declare
`axis="0 0 1"`, so the propeller spins about local **Z** and thrust acts along
**Z**, not X. Using `force.x` shoved the vertical pair *forwards* below the
centre of mass — "full vertical thrust" produced a ±20° pitching couple and no
vertical motion.

**5. Two separate mapping/geometry errors cancelled the horizontal thrusters.**
- All four horizontals pointed **forward** (rear pair exported at 45° instead of
  135°), so guidance's `(+,+,−,−)` forward mixing summed to **exactly zero**.
  Rear pair rotated to ±135°.
- **Motors 4 and 6 were swapped** in the motor→thruster map (inherited from
  `motor_to_joint_states.py`, whose README admits the mapping was never
  verified). With them swapped, yaw commands came out as pure *strafe* — the
  four yaw moments cancelled. The correct assignment is derived from physics in
  `thruster_bridge.py`: `m1→t1, m3→t2, m6→t3, m4→t4`.

> **`hauv_description/motor_to_joint_states.py` still has the 4/6 swap.** It only
> spins propellers in RViz so nothing breaks visually, but it is wrong.

## Limitations — read before believing a result

- **The water surface is faked with a collision slab.** Gazebo's
  `BuoyancyPlugin` has no free surface and applies buoyancy at every height, so
  the vehicle used to rise forever (it reached z = +10 m — 10 m in the air,
  still "floating"). A static collision slab in `underwater.world` puts a
  ceiling at z = 0: the vehicle now rises, meets it and parks at depth
  ~+0.066 m indefinitely. It is a rigid contact, not surface dynamics — it
  touches and settles rather than bobbing, and nothing pushes it back down, so
  the vehicle cannot be launched from above the water. Spawn below z = 0 (the
  launch file does, at −0.15).
- **Buoyant ascent is slow, by design.** At +0.1 kg trim an uncommanded vehicle
  climbs ~1 cm/s, so recovering from 12 m takes ~20 minutes. That is realistic;
  use the vertical thrusters if you want to get up quickly.
- **Drag is linear and isotropic**, not quadratic, and there is **no added
  mass**. It was tuned at ~1 m/s and is only honest near there. Fine for
  exercising control loops; not a hydrodynamic model. Proper added mass needs a
  custom plugin — the modern gz-sim hydrodynamics stack targets Humble+, and
  this vehicle is on Foxy.
- **Vertical thrust induces pitch** (~±20° at full). Real: the thruster pair
  straddles a centre of mass that sits 22 mm aft of their midpoint, so equal
  thrust makes a couple. Trim the CG or expect the controller to handle it.
- The camera plugin needs rendering; headless logs harmless `CameraSensor` errors.

## colcon quirk

Entry points install to `install/hauv_gazebo_v2/bin/` instead of
`lib/hauv_gazebo_v2/`, which breaks `ros2 run` and launch. `wsl_build.sh` copies
them across automatically — same workaround as `hauv_description` and
`mavlink_bridge_pkg`.
