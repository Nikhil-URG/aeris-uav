# AERIS Demo Runbook

This runbook reproduces the complete AERIS demonstration in simulation. It is
also the storyboard for a screen-recorded demo.

## 0. Prerequisites

- Ubuntu/Pop!_OS with the AERIS environment installed (`docs/installation.md`).
- PX4 checkout at `~/aeris_dev/PX4-Autopilot`, pinned to `v1.17.0`.
- QGroundControl running and connected. PX4 requires a GCS for arming in this
  SITL configuration.
- Workspace built and environment verified:

```bash
cd /home/lucifer/Documents/Project_Aeris
source scripts/aeris_env.sh
bash scripts/check_environment.sh
bash scripts/build.sh
```

Expected: `18 ok, 0 warnings, 0 failures` and a clean colcon build.

Run each terminal from the repository root and source the environment in every
new terminal:

```bash
source scripts/aeris_env.sh
```

Start QGroundControl once:

```bash
scripts/run_qgroundcontrol.sh
```

## 1. Nominal inspection demo

### Terminal A — camera simulation

```bash
PX4_SIM_MODEL=gz_x500_mono_cam_down bash scripts/launch_simulation.sh
```

Wait for Gazebo, PX4 and QGC. Spawn the two red inspection targets:

```bash
bash scripts/spawn_targets.sh
```

### Terminal B — camera bridge and perception

```bash
source scripts/aeris_env.sh
ros2 launch aeris_bringup perception.launch.py
```

Expected: `Camera info: 1280x960 fx=539.9 ...`, then detections on
`/aeris/perception/detections`.

### Terminal C — mission

```bash
source scripts/aeris_env.sh
ros2 launch aeris_bringup mission.launch.py
```

### Terminal D — start

```bash
source scripts/aeris_env.sh
ros2 service call /aeris_mission_manager/start std_srvs/srv/Trigger
```

Expected: takeoff to 5 m, transit to `wp_01`, 3 s inspection, `wp_02`,
3 s inspection, RTL, landing, `COMPLETE`. The run log is
`results/logs/<run_id>/mission.jsonl` with two `target_detected` events.

## 2. Obstacle-avoidance demo

Stop Terminal A and restart with the lidar airframe:

```bash
PX4_SIM_MODEL=gz_x500_lidar_2d bash scripts/launch_simulation.sh
bash scripts/spawn_obstacle.sh
```

Terminal B becomes:

```bash
source scripts/aeris_env.sh
ros2 launch aeris_bringup navigation.launch.py
```

Terminal C remains the mission launch. Start it from Terminal D.

Expected JSONL events:

```text
obstacle_detected: range_m=3.75, bearing_rad=-0.204
replan: detour_x=1.54, detour_y=1.94, detour_z=-5.0
waypoint_reached: wp_02
mission_complete
```

## 3. Safety demo

Use the short `safety_test` mission and start the supervisor:

```bash
source scripts/aeris_env.sh
MISSION_FILE=$(ros2 pkg prefix aeris_bringup)/share/aeris_bringup/config/missions/safety_test.yaml
ros2 launch aeris_bringup mission.launch.py mission_file:="$MISSION_FILE"
ros2 launch aeris_bringup safety.launch.py
```

Start the mission, wait for `[INSPECT]`, then inject a fault:

```bash
ros2 service call /aeris_mission_manager/start std_srvs/srv/Trigger
ros2 service call /aeris_safety/inject_fault aeris_interfaces/srv/InjectFault \
  "{fault: low_battery, enable: true}"
```

Expected:

- `/aeris/safety/status` turns `safe=false` with `active_faults: [low_battery]`;
- the supervisor dispatches `rtl` in under 100 ms;
- PX4 logs `Returning to launch`, lands and disarms;
- the mission JSONL records `safety_abort` with `action=rtl`.

For the stale-pose case use `fault: stale_pose`; the supervisor commands
`land` instead.

The complete automated scenario runner is:

```bash
bash scripts/run_safety_fault_test.sh low_battery
```

## 4. City-grid GUI demo

Use the lightweight airframe; the grid demo does not need the camera:

```bash
PX4_SIM_MODEL=gz_x500 bash scripts/launch_simulation.sh
bash scripts/spawn_grid.sh
```

Launch the full grid stack and GUI:

```bash
source scripts/aeris_env.sh
ros2 launch aeris_bringup grid_demo.launch.py
```

Click a free cell in the PyQt5 window. Expected behavior:

- the A* path appears immediately (yellow line);
- the drone takes off, follows the path around the dark building cells;
- the progress bar and NED position update live;
- at the goal the state becomes `AT_GOAL`, then `RETURNING`;
- the drone returns to the green start cell, lands and disarms;
- `docs/experiments/grid_mission.jsonl` is written.

Click another free cell without restarting the stack; the offboard controller
accepts a new mission after `DONE`. Use the **Go Home** button to target the
start cell, or **Land** to land immediately.

Headless alternative:

```bash
ros2 launch aeris_bringup grid_navigation.launch.py
ros2 service call /aeris_grid_navigation/set_goal \
  aeris_interfaces/srv/GridGoal "{x: 10, y: 10}"
```

## 5. Evidence to show

| Artifact | Path |
|---|---|
| Nominal mission | `docs/experiments/week3_mission.jsonl` |
| Perception | `docs/experiments/week4_mission.jsonl` |
| Obstacle detour | `docs/experiments/week5_mission.jsonl` |
| Safety nominal | `docs/experiments/week6_nominal_mission.jsonl` |
| Safety faults | `docs/experiments/week6_*_mission.jsonl` |
| Grid missions | `docs/experiments/grid_*mission.jsonl` |
| Grid validation | `docs/experiments/grid_validation.txt` |
| GUI screenshot | `docs/experiments/plots/grid_gui_repeat.png` |
| Demo video (MP4) | `docs/experiments/media/aeris_demo.mp4` |
| README hero GIF | `docs/experiments/media/aeris_demo.gif` |
| Original recording | `docs/experiments/media/aeris_demo_full.mkv` |
| Benchmark metrics | `docs/experiments/benchmark.json` |
| Plots | `docs/experiments/plots/` |
| Evaluation report | `docs/evaluation.md` |
| Grid design | `docs/grid_navigation.md` |

## 6. Recorded demo and storyboard

The checked-in SITL recording shows the city-grid GUI and Gazebo side by side:
goal selection, A* path, transit, return to start, and landing.

- **README hero:** `docs/experiments/media/aeris_demo.gif` — 900×506, 6× speed,
  13 s (8.1 MiB)
- **Full video:** `docs/experiments/media/aeris_demo.mp4` — 1280×720,
  30 fps, 2:20, H.264 (3.2 MiB)
- **Original recording:** `docs/experiments/media/aeris_demo_full.mkv` —
  1280×720, 60 fps (7.4 MiB)

The GIF is trimmed to the mission and sped up for README loading. The MP4 is
the full recording for detailed review.

| Scene | Duration | Content |
|---|---|---|
| 1 | 15 s | Title, repository URL, architecture diagram from `docs/architecture.md` |
| 2 | 20 s | Terminal: environment check and build summary |
| 3 | 45 s | Gazebo + QGC; takeoff and two-waypoint mission |
| 4 | 30 s | Perception log and detected targets |
| 5 | 40 s | Obstacle visible in Gazebo, detour, obstacle/replan events |
| 6 | 45 s | Safety status, injected fault, RTL/land, disarmed vehicle |
| 7 | 45 s | City grid, click-to-goal, A* path, live progress, return and landing |
| 8 | 20 s | Benchmark plots and test summary |

Recording notes:

- Start each scenario from a fresh SITL process so the airframe and battery
  state are clean.
- Keep QGroundControl visible for the arming and flight-state panels.
- Zoom terminal text to a readable size; the important lines are the state
  transitions and JSONL events.
- Do not edit logs after recording; use the checked-in evidence for the
  written report.

## 7. Cleanup

Stop the ROS terminals with `Ctrl+C`, then stop PX4/Gazebo. QGroundControl can
remain open for the next scenario. If a launch leaves a stale node behind:

```bash
ps -eo pid,args | grep -E 'mission_manager|offboard_controller|safety_node'
```

Kill the whole process group, not just the `ros2 launch` front-end.
