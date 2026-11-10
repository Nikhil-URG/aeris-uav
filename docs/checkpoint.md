# AERIS Checkpoint

**Checkpoint date:** 2026-11-10  
**Phase:** Post-Week 7 extension complete — city-grid A* navigation and GUI delivered  
**Repository:** `/home/lucifer/Documents/Project_Aeris`  
**Branch:** `main`  
**Week 1 tag:** `week-1-checkpoint` (commit `0486c1e`)  
**Week 2 tag:** `week-2-checkpoint` (commit `0666e09`)  
**Week 3 tag:** `week-3-checkpoint` (commit `ba262c9`)  
**Week 4 tag:** `week-4-checkpoint` (commit `8618b01`)  
**Week 5 tag:** `week-5-checkpoint` (commit `4237774`)  
**Week 6 tag:** `week-6-checkpoint` (commit `94e8f02`)  
**Week 7 tag:** `week-7-checkpoint` (commit `b8c39df`)  
**Grid extension tag:** `grid-navigation-checkpoint` (this checkpoint commit)  
**Remote:** `git@github.com:Nikhil-URG/aeris-uav.git`  
**Web:** <https://github.com/Nikhil-URG/aeris-uav>

This document is the portable handoff. It records what is verified, how to reproduce the environment, what bit us along the way, and what to do next. Read this before touching the code after a break.

---

## 1. Project context

AERIS is a simulated industrial-inspection UAV built on **PX4 + ROS 2 Jazzy**. PX4 owns low-level flight stabilization (EKF2, position/attitude control, actuators). The ROS 2 application owns mission-level intelligence and talks to PX4 through **uXRCE-DDS**.

```text
Gazebo Harmonic / X500
          │ sensors
          ▼
PX4 v1.17.0
  EKF2 → position → attitude → actuators
          │ uXRCE-DDS
          ▼
ROS 2 Jazzy
  aeris_flight_control (C++)
    ├── telemetry_monitor        (Week 1)
    └── offboard_controller      (Week 2)
  aeris_bringup (Python)
    ├── telemetry.launch.py
    └── offboard.launch.py
```

Week 2 added a service-triggered offboard controller: readiness checks, 20 Hz offboard heartbeat, mode request, arming, takeoff to 5 m, position hold and controlled landing.

Week 3 adds the mission manager: waypoint missions, a state machine, validation and structured logging.

---

## 2. Verified state at checkpoint

| Item | State |
|---|---|
| PX4-Autopilot | `v1.17.0`, commit `d6f12ad1c4` |
| `px4_msgs` submodule | `v1.17.0`, commit `86d8239e96` |
| Gazebo Harmonic | `8.15.0` |
| Micro XRCE-DDS Agent | `v2.4.3` at `/usr/local/bin/MicroXRCEAgent` |
| QGroundControl | `v5.1.5` AppImage at `~/Applications/QGroundControl.AppImage` |
| ROS 2 | Jazzy (`/opt/ros/jazzy`) |
| System Python | 3.12.3 at `/usr/bin/python3` |
| NVIDIA driver | `595.104.02` (`nvidia-smi` works after reboot) |
| Workspace build | `px4_msgs`, `aeris_interfaces`, `aeris_flight_control`, `aeris_mission_manager`, `aeris_perception`, `aeris_navigation`, `aeris_safety`, `aeris_grid_navigation`, `aeris_grid_gui`, `aeris_bringup` |
| Telemetry rate | **50.005 Hz** on `/fmu/out/vehicle_local_position_v1` |
| Offboard stream | **20 Hz** (`OffboardControlMode` + `TrajectorySetpoint`) |
| Idle heartbeat | **2 Hz** while no mission is active |
| Clean flight | Arm → 5 m → 10 s hold → land → disarm |
| Takeoff time | ~7.8 s to 5 m |
| Steady-state hold error | 0.01–0.03 m |
| Landing to disarm | ~10.9 s |
| Final post-landing state | `AUTO_LOITER`, disarmed, `pre_flight_checks_pass=true` |
| Abort path | External QGC land during HOLD → controller sent `NAV_LAND` → safe landing |
| Mission manager | YAML validation, state machine, JSONL logging |
| Two-waypoint mission | 2/2 waypoints, RTL, land, `COMPLETE`, ~33.8 s |
| Mission manager tests | 11/11 passing |
| Mission evidence | `docs/experiments/week3_mission.jsonl` |
| Camera | `gz_x500_mono_cam_down`, 1280×960, ~12.8 Hz |
| Perception | OpenCV red-panel detector, 4/4 tests |
| Mission detections | 2/2 waypoints, confidence 1.0, range ~6.3 m |
| Perception evidence | `docs/experiments/week4_mission.jsonl` |
| Lidar | `gz_x500_lidar_2d`, 270°, ~30 Hz |
| Navigation | Sector obstacle map + corridor check, 4/4 tests |
| Obstacle replan | 3.75 m obstacle → detour `(1.54, 1.94)` → wp_02 reached |
| Navigation evidence | `docs/experiments/week5_mission.jsonl` |
| Safety supervisor | Pose, battery, geofence, mission heartbeat; 10 Hz `SafetyStatus` |
| Fault injection | `stale_pose`, `low_battery`, `geofence_breach`, `mission_heartbeat_loss` |
| Safety actions | Stale pose → land; other faults → RTL; all flights disarmed |
| Safety tests | 11/11 evaluator tests; workspace total 32/32 |
| Safety evidence | `docs/experiments/week6_*_mission.jsonl` |
| Benchmark set | 8 runs: 4 completed, 4 planned safety aborts |
| Benchmark success | 4/4 planned completions; 4/4 safe fault outcomes |
| Detection benchmark | 2/2 waypoints, confidence 1.00, rate 100% |
| Obstacle benchmark | 3.75 m detection, 1 replan, original waypoint reached |
| Safety latency | 41–98 ms from fault injection to action dispatch |
| Evaluator tests | 4/4 |
| City grid | 11×11 cells, 4 m cells, 12 buildings, 44 occupied cells |
| Grid planner | 8-connected A*, no corner cutting, 9/9 tests |
| Grid GUI | PyQt5 click-to-goal, live path/drone/progress, 1/1 test |
| Grid missions | 3/3 complete: `(0,0)`, `(10,10)`, `(2,5)` after `DONE` |
| Grid clearance | 2.269 m minimum; 0 positions inside buildings |
| Grid evidence | `docs/experiments/grid_*` |
| QGC | Connected; PX4 logs `[mavlink] partner IP: 127.0.0.1` |
| Environment check | `20 ok, 0 warnings, 0 failures` |

Milestone 1–7 and the grid extension acceptance are complete.

---

## 3. Environment and important paths

| Item | Path / value |
|---|---|
| Repository | `/home/lucifer/Documents/Project_Aeris` |
| PX4 checkout | `~/aeris_dev/PX4-Autopilot` (override `AERIS_PX4_DIR`) |
| Micro XRCE agent source | `~/aeris_dev/Micro-XRCE-DDS-Agent` |
| ROS workspace | `ros2_ws/` |
| QGC AppImage | `~/Applications/QGroundControl.AppImage` |
| Simulation logs | `results/logs/` (ignored by Git) |
| Week 2 flight evidence | `docs/experiments/week2_offboard_run.log` |
| Week 3 mission evidence | `docs/experiments/week3_mission.jsonl` |
| Week 4 perception evidence | `docs/experiments/week4_mission.jsonl` |
| Week 5 navigation evidence | `docs/experiments/week5_mission.jsonl` |
| Development journal | `docs/experiments/development_log.md` |

The machine is Pop!_OS 24.04 (Ubuntu 24.04 `noble` base) with an Intel i5-13420H, 15 GiB RAM, and an RTX 4050 Laptop GPU.

### The conda rule

Miniforge is installed and its `python3` is Python 3.13. ROS 2 and PX4 require the system Python 3.12. **Always source the AERIS environment script before ROS/PX4 commands:**

```bash
source scripts/aeris_env.sh
```

It deactivates/strips conda, unsets `CONDA_PREFIX` and related markers (CMake's `FindPython3` treats an active conda prefix as a virtualenv), and sources ROS plus the workspace.

---

## 4. Repository map

```text
.
├── README.md
├── claude.md                       # original project brief
├── docs/
│   ├── architecture.md             # layers, frames, interfaces, ADRs
│   ├── installation.md             # setup + troubleshooting
│   ├── flight_control.md           # telemetry + offboard contract/results
│   ├── mission_design.md           # Week 3 state machine and mission schema
│   ├── perception.md               # Week 4 camera + OpenCV detection
│   ├── navigation.md               # Week 5 lidar + replanning
│   ├── safety.md                   # Week 6 supervisor + faults
│   ├── evaluation.md               # Week 7 benchmarks + report
│   ├── demo.md                     # reproducible demo runbook
│   ├── grid_navigation.md          # city grid, A* and GUI
│   ├── checkpoint.md               # this file
│   └── experiments/
│       ├── development_log.md
│       ├── week2_offboard_run.log
│       ├── week3_mission.jsonl
│       ├── week3_mission_run.log
│       ├── week4_mission.jsonl
│       ├── week4_mission_run.log
│       ├── week4_perception.log
│       ├── week5_mission.jsonl
│       ├── week5_mission_run.log
│       ├── week5_navigation.log
│       ├── week6_nominal_mission.jsonl
│       ├── week6_stale_pose_mission.jsonl
│       ├── week6_low_battery_mission.jsonl
│       ├── week6_geofence_breach_mission.jsonl
│       ├── week6_mission_heartbeat_loss_mission.jsonl
│       ├── week6_*_run.log / week6_*_safety.log
│       ├── benchmark.json / benchmark.csv
│       ├── grid_mission.jsonl
│       ├── grid_repeat_goal.jsonl
│       ├── grid_repeat_short.jsonl
│       ├── grid_validation.txt
│       └── plots/
├── px4/README.md                   # upstream pin and upgrade policy
├── ros2_ws/src/
│   ├── px4_msgs/                   # Git submodule, v1.17.0
│   ├── aeris_interfaces/           # AERIS msgs/srvs
│   ├── aeris_flight_control/       # C++ telemetry + offboard controller
│   ├── aeris_mission_manager/      # Python mission manager
│   ├── aeris_perception/           # OpenCV target detection
│   ├── aeris_navigation/           # Lidar obstacle detection
│   ├── aeris_safety/               # Health monitoring + fault injection
│   ├── aeris_grid_navigation/      # Occupancy grid + A* executor
│   ├── aeris_grid_gui/             # PyQt5 goal/progress GUI
│   └── aeris_bringup/              # launch + YAML + missions + grids
├── scripts/
│   ├── setup.sh
│   ├── aeris_env.sh
│   ├── install_dependencies.sh
│   ├── bootstrap_px4.sh
│   ├── install_microxrce_agent.sh
│   ├── install_qgroundcontrol.sh
│   ├── build.sh
│   ├── launch_simulation.sh
│   ├── run_qgroundcontrol.sh
│   ├── spawn_targets.sh
│   ├── spawn_obstacle.sh
│   ├── spawn_grid.sh
│   ├── spawn_buildings.py
│   ├── run_safety_fault_test.sh
│   ├── check_environment.sh
│   └── evaluate_mission.py         # metrics, CSV and plots
├── simulation/
│   ├── models/red_target/          # Week 4 inspection target
│   ├── models/tall_obstacle/       # Week 5 obstacle
│   ├── worlds/
│   └── scenarios/
├── tests/                          # evaluator + integration/scenario tests
└── results/                        # logs/plots/reports (ignored)
```

Key Week 2–6 files:

- `ros2_ws/src/aeris_flight_control/src/offboard_controller.cpp`
- `ros2_ws/src/aeris_mission_manager/aeris_mission_manager/mission.py`
- `ros2_ws/src/aeris_mission_manager/aeris_mission_manager/node.py`
- `ros2_ws/src/aeris_perception/aeris_perception/detector.py`
- `ros2_ws/src/aeris_perception/aeris_perception/node.py`
- `ros2_ws/src/aeris_navigation/src/navigation_node.cpp`
- `ros2_ws/src/aeris_navigation/include/aeris_navigation/scan_analysis.hpp`
- `ros2_ws/src/aeris_safety/src/safety_node.cpp`
- `ros2_ws/src/aeris_safety/include/aeris_safety/safety_evaluator.hpp`
- `ros2_ws/src/aeris_grid_navigation/aeris_grid_navigation/grid.py`
- `ros2_ws/src/aeris_grid_navigation/aeris_grid_navigation/node.py`
- `ros2_ws/src/aeris_grid_gui/aeris_grid_gui/gui.py`
- `ros2_ws/src/aeris_bringup/config/offboard.yaml`
- `ros2_ws/src/aeris_bringup/config/offboard_mission.yaml`
- `ros2_ws/src/aeris_bringup/config/gz_camera_bridge.yaml`
- `ros2_ws/src/aeris_bringup/config/gz_lidar_bridge.yaml`
- `ros2_ws/src/aeris_bringup/config/safety.yaml`
- `ros2_ws/src/aeris_bringup/config/offboard_grid.yaml`
- `ros2_ws/src/aeris_bringup/config/safety_grid.yaml`
- `ros2_ws/src/aeris_bringup/config/grids/city_grid.yaml`
- `ros2_ws/src/aeris_bringup/config/missions/inspection_alpha.yaml`
- `ros2_ws/src/aeris_bringup/config/missions/safety_test.yaml`
- `ros2_ws/src/aeris_bringup/launch/offboard.launch.py`
- `ros2_ws/src/aeris_bringup/launch/mission.launch.py`
- `ros2_ws/src/aeris_bringup/launch/perception.launch.py`
- `ros2_ws/src/aeris_bringup/launch/navigation.launch.py`
- `ros2_ws/src/aeris_bringup/launch/safety.launch.py`
- `ros2_ws/src/aeris_bringup/launch/grid_navigation.launch.py`
- `ros2_ws/src/aeris_bringup/launch/grid_gui.launch.py`
- `ros2_ws/src/aeris_bringup/launch/grid_demo.launch.py`
- `docs/flight_control.md`
- `docs/perception.md`
- `docs/navigation.md`
- `docs/safety.md`
- `docs/evaluation.md`
- `docs/demo.md`
- `docs/grid_navigation.md`
- `scripts/evaluate_mission.py`
- `scripts/spawn_grid.sh`
- `scripts/spawn_buildings.py`
- `tests/test_evaluate_mission.py`
- `docs/experiments/week2_offboard_run.log`
- `docs/experiments/week3_mission.jsonl`
- `docs/experiments/week4_mission.jsonl`
- `docs/experiments/week5_mission.jsonl`
- `docs/experiments/week6_*_mission.jsonl`
- `docs/experiments/benchmark.json`
- `docs/experiments/grid_mission.jsonl`
- `docs/experiments/grid_repeat_goal.jsonl`
- `docs/experiments/grid_repeat_short.jsonl`
- `docs/experiments/grid_validation.txt`
- `docs/experiments/plots/`

---

## 5. How to resume on any machine

### 5.1 Clone and initialize

```bash
git clone --recursive git@github.com:Nikhil-URG/aeris-uav.git
cd aeris-uav
# If cloned without --recursive:
# git submodule update --init --recursive
```

### 5.2 Environment

```bash
source scripts/aeris_env.sh
bash scripts/check_environment.sh
```

Expected: system Python 3.12, ROS Jazzy, Gazebo 8.x, PX4 `v1.17.0`, agent installed, workspace built, `18 ok, 0 warnings, 0 failures`.

If dependencies are missing, run the setup scripts in order:

```bash
bash scripts/install_dependencies.sh
bash scripts/bootstrap_px4.sh
bash scripts/install_microxrce_agent.sh
bash scripts/build.sh
```

### 5.3 Build

```bash
bash scripts/build.sh
```

If the build picks up conda Python, see §6.1 and remove the stale build tree before rebuilding.

### 5.4 Run the simulation

Terminal 1 — DDS agent + PX4 + Gazebo:

```bash
bash scripts/launch_simulation.sh
```

Headless (no GUI):

```bash
HEADLESS=1 bash scripts/launch_simulation.sh
```

Terminal 2 — telemetry:

```bash
source scripts/aeris_env.sh
ros2 launch aeris_bringup telemetry.launch.py
```

QGC:

```bash
scripts/run_qgroundcontrol.sh
```

### 5.5 Offboard flight demo

With PX4, Gazebo and QGC running:

```bash
source scripts/aeris_env.sh
ros2 launch aeris_bringup offboard.launch.py
```

In another terminal, trigger the mission:

```bash
ros2 service call /aeris_offboard_controller/start std_srvs/srv/Trigger
```

Abort:

```bash
ros2 service call /aeris_offboard_controller/abort std_srvs/srv/Trigger
```

Expected sequence:

```text
WAIT_FOR_READY → PRESTREAM → REQUEST_OFFBOARD → ARM
→ TAKEOFF → HOLD → LAND → DONE
```

Do not interact with QGC during an automated run unless you intend to exercise the external-land/abort path.

### 5.6 Full mission demo

The mission launch starts telemetry, the offboard controller and the mission manager:

```bash
source scripts/aeris_env.sh
ros2 launch aeris_bringup mission.launch.py
```

In another terminal:

```bash
ros2 service call /aeris_mission_manager/start std_srvs/srv/Trigger
```

The default `inspection_alpha` mission takes off, visits two inspection waypoints, holds 3 s at each, returns to launch and lands. The structured log is written to `results/logs/<run_id>/mission.jsonl`. Abort with:

```bash
ros2 service call /aeris_mission_manager/abort std_srvs/srv/Trigger
```

Do not run `offboard.launch.py` and `mission.launch.py` at the same time: both start an offboard controller with the same node name.

### 5.7 Verify topics

```bash
source scripts/aeris_env.sh
ros2 topic list | grep '/fmu/'
ros2 topic hz /fmu/out/vehicle_local_position_v1
```

Expected: ~50 Hz, `xy_valid: true`, `z_valid: true`, `v_xy_valid: true`, `v_z_valid: true`.

### 5.8 Perception demo

Camera-enabled simulation:

```bash
PX4_SIM_MODEL=gz_x500_mono_cam_down bash scripts/launch_simulation.sh
```

Spawn the two red targets:

```bash
bash scripts/spawn_targets.sh
```

Start the camera bridge and perception:

```bash
source scripts/aeris_env.sh
ros2 launch aeris_bringup perception.launch.py
```

Then run the full mission as in §5.6. The JSONL log should contain `target_detected` events during each `INSPECT` phase.

### 5.9 Lidar and obstacle-avoidance demo

```bash
PX4_SIM_MODEL=gz_x500_lidar_2d bash scripts/launch_simulation.sh
bash scripts/spawn_obstacle.sh
source scripts/aeris_env.sh
ros2 launch aeris_bringup navigation.launch.py
```

Then run the mission as in §5.6. The mission manager should log `obstacle_detected`, a lateral `replan`, reach the original waypoint and complete.

### 5.10 Safety and fault-injection demo

Start the mission stack with the short safety mission and the supervisor:

```bash
source scripts/aeris_env.sh
MISSION_FILE=$(ros2 pkg prefix aeris_bringup)/share/aeris_bringup/config/missions/safety_test.yaml
ros2 launch aeris_bringup mission.launch.py mission_file:="$MISSION_FILE"
ros2 launch aeris_bringup safety.launch.py
```

Start the mission, then inject a fault during the hold:

```bash
ros2 service call /aeris_mission_manager/start std_srvs/srv/Trigger
ros2 service call /aeris_safety/inject_fault aeris_interfaces/srv/InjectFault \
  "{fault: stale_pose, enable: true}"
```

Expected: `stale_pose` → land; `low_battery`, `geofence_breach`, `mission_heartbeat_loss` → RTL. The mission JSONL records `safety_abort`, and the vehicle lands and disarms. The complete scenario runner is:

```bash
bash scripts/run_safety_fault_test.sh low_battery
```

### 5.11 Evaluation and demo

Regenerate the benchmark report, CSV and plots from the checked-in evidence:

```bash
source scripts/aeris_env.sh
python3 scripts/evaluate_mission.py \
  --json docs/experiments/benchmark.json \
  --csv docs/experiments/benchmark.csv \
  --plot-dir docs/experiments/plots
python3 -m pytest tests/test_evaluate_mission.py -q
```

The full report is `docs/evaluation.md`; the terminal-by-terminal demo and
video storyboard are `docs/demo.md`.

### 5.12 City-grid navigation demo

Start the lightweight airframe and spawn the city:

```bash
source scripts/aeris_env.sh
PX4_SIM_MODEL=gz_x500 bash scripts/launch_simulation.sh
bash scripts/spawn_grid.sh
```

Launch the grid stack with the GUI:

```bash
ros2 launch aeris_bringup grid_demo.launch.py
```

Click a free cell, or call the service:

```bash
ros2 service call /aeris_grid_navigation/set_goal \
  aeris_interfaces/srv/GridGoal "{x: 0, y: 0}"
```

The drone plans the shortest free path, flies to the goal, returns to the
start cell, and lands. Repeated goals work without restarting the stack.
Headless operation:

```bash
ros2 launch aeris_bringup grid_navigation.launch.py
```

See `docs/grid_navigation.md` and `docs/experiments/grid_*`.

---

## 6. Hard-won gotchas

### 6.1 Conda leaks into CMake

Even with `PATH` cleaned, `CONDA_PREFIX` made CMake select `/home/lucifer/miniforge3/bin/python3` (3.13). The build failed with `ModuleNotFoundError: No module named 'em'`.

`scripts/aeris_env.sh` unsets `CONDA_PREFIX`, `CONDA_DEFAULT_ENV`, `CONDA_SHLVL`, `CONDA_PYTHON_EXE`, `CONDA_EXE`, `_CONDA_EXE`, `_CONDA_ROOT`, `_CE_CONDA`, and strips conda paths from `PATH`, `PYTHONPATH`, `LD_LIBRARY_PATH`, `CMAKE_PREFIX_PATH` and `PKG_CONFIG_PATH`.

If a build already cached the wrong interpreter:

```bash
rm -rf ros2_ws/build ros2_ws/install ros2_ws/log
bash scripts/build.sh
```

Check `_Python3_EXECUTABLE:INTERNAL` in `ros2_ws/build/<package>/CMakeCache.txt`; it must point to `/usr/bin/python3`.

### 6.2 ROS setup scripts are not `set -u` safe

`/opt/ros/jazzy/local_setup.sh` reads `AMENT_PYTHON_EXECUTABLE` without a default. Sourcing it under `set -u` aborts with `unbound variable`. `scripts/aeris_env.sh` disables nounset only while sourcing ROS, then restores the caller's setting.

### 6.3 PX4 v1.17 uses `_v1` DDS topic suffixes

The active local-position topic is `/fmu/out/vehicle_local_position_v1`; status is `/fmu/out/vehicle_status_v1`. The unsuffixed names can appear in the ROS graph if a subscriber creates them, but they have no publisher. Always discover topic names after starting SITL:

```bash
ros2 topic list | grep '/fmu/out'
```

### 6.4 Never pipe PX4's interactive shell through `tee`

PX4 redraws its prompt continuously when stdout is a pipe. An early run produced two **7 GB** log files. `scripts/launch_simulation.sh` now runs PX4 under a PTY with `script` and rotates logs over 200 MB.

### 6.5 Offboard signal loss blocks preflight after landing

PX4 sets `offboard_control_signal_lost=true` when the offboard stream stops, which keeps `pre_flight_checks_pass` false even in another mode. The controller now publishes a **2 Hz idle `OffboardControlMode` heartbeat** while no mission is streaming. Without it, a second run cannot pass preflight checks.

### 6.6 Restore AUTO_LOITER, not POSCTL

In this GCS-only SITL setup there is no RC/manual-control link. `POSCTL` requires one and keeps preflight checks false. After landing, the controller requests `AUTO_LOITER` (custom main mode 4, sub mode 3), which allows `pre_flight_checks_pass=true` again.

### 6.7 External AUTO_LAND is a landing, not a failure

If an operator or QGC commands `AUTO_LAND` during HOLD, PX4 leaves offboard mode. The controller detects `nav_state == AUTO_LAND` and transitions to `LAND` instead of `ABORT_LAND`. Other offboard losses still abort.

### 6.8 Stale offboard launch processes interfere

Multiple `ros2 launch` instances from earlier tests can publish competing heartbeats. Before a flight, verify:

```bash
ps -eo pid,args | grep -E 'offboard_controller|offboard.launch'
```

Stop old instances before starting a new mission.

### 6.9 QGC on Wayland

`scripts/run_qgroundcontrol.sh` forces `QT_QPA_PLATFORM=xcb`. The AppImage may ignore SIGTERM; stop it with SIGKILL if needed.

### 6.10 NVIDIA driver mismatch

The first session reported `Failed to initialize NVML: Driver/library version mismatch`. A reboot resolved it; the driver now reports `595.104.02`. If it returns after a system update, reboot before diagnosing Gazebo rendering.

### 6.11 PX4 is external

PX4-Autopilot is **not** vendored. It lives at `~/aeris_dev/PX4-Autopilot` and is pinned to `v1.17.0` (commit `d6f12ad1c4`). The `px4_msgs` submodule is pinned to the matching `v1.17.0` commit. Do not upgrade either independently; follow `px4/README.md`.

### 6.12 Goal sequencing, not latched `goal_reached`

The offboard controller latches `goal_reached` until a new goal arrives. A mission manager that only waits for `goal_reached` can mistake the previous waypoint for the next one. `GoTo` responses and `FlightStatus` carry a monotonically increasing `goal_sequence`; always match the sequence returned by the service call.

### 6.13 Mission mode disables auto-land

`offboard.yaml` is the standalone demo config (`auto_land_after_hold: true`). `offboard_mission.yaml` sets it to `false`; the mission manager owns the landing decision. Do not use the wrong config for a waypoint mission.

### 6.14 User-site NumPy breaks apt OpenCV

A NumPy 2.x in `~/.local/lib/python3.12/site-packages` shadows the apt NumPy 1.26 and breaks `cv2`/`cv_bridge` with an ABI error. `scripts/aeris_env.sh` exports `PYTHONNOUSERSITE=1`; always source it before perception.

### 6.15 Camera bridge topics are model-instance specific

The bridge config points at:

```text
/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image
```

If the PX4 instance changes, update `gz_camera_bridge.yaml`. Verify with `gz topic -l | grep imager`.

### 6.16 Localization is approximate

The Week 4 ground projection assumes a nadir camera and square pixels. The downward camera can see the neighbouring target at the frame edge, and the projection is not a calibrated extrinsic solution. Detection is robust; precise localization is a later refinement.

### 6.17 The lidar is 270°, but the target may not be forward

The Week 5 mission manager originally checked only the forward cone and missed an obstacle 90° off the heading. `aeris_navigation` now publishes 10° sector obstacles across the full scan, and the mission manager evaluates the corridor toward the target. Do not reintroduce a forward-only assumption.

### 6.18 The path-clearance threshold is 4 m

The simulated obstacle sits near 3.75 m at the check point. The mission manager uses a 4 m path-clearance threshold. Lowering it can suppress a valid replan.

### 6.19 The simulated battery drains over long sessions

PX4's battery simulator defaults to draining from 100% to 50% in 60 s (`SIM_BAT_DRAIN=60`, `SIM_BAT_MIN_PCT=50`). That can trigger PX4's own battery failsafe and look like a software failure. `scripts/launch_simulation.sh` now sets `PX4_PARAM_SIM_BAT_MIN_PCT=100` and `PX4_PARAM_SIM_BAT_DRAIN=86400` by default. Override them to test PX4's battery path explicitly.

### 6.20 PX4 local frame vs Gazebo world

Spawned models live in Gazebo world coordinates while the mission uses PX4 local NED. The origin offset is small but real; it can shift targets/obstacles by a few tenths of a metre. Use lidar/camera-relative checks or calibrate the spawn offset for precise map alignment.

### 6.21 The safety supervisor must wait for telemetry

At startup the safety node can match PX4 topics a second or two after the mission manager. Evaluating immediately produces a false `stale_pose` and aborts the mission. The supervisor gates evaluation on pose + vehicle-status readiness, and the mission manager only treats an unsafe status as an abort when `SafetyStatus.armed` is true.

### 6.22 Launch cleanup must kill the process group

Killing only the `ros2 launch` front-end can leave Python nodes alive. A stale mission manager with the same node name causes goal-sequence mismatches that look like flight-control bugs. `run_safety_fault_test.sh` starts each launch with `setsid` and signals the whole process group. Check for stale nodes with `ps -eo pid,args | grep -E 'mission_manager|offboard_controller'`.

### 6.23 RTL must not be overridden by the local abort path

When `~/rtl` is commanded, PX4 leaves offboard mode. The controller must enter its explicit `RTL` state, stop the offboard stream, and not treat the resulting offboard/failsafe transition as an abort. Otherwise it sends `NAV_LAND` and overrides the return-to-launch decision.

### 6.24 Grid packages must not depend on bringup

`aeris_bringup` launches `aeris_grid_gui`, so the GUI cannot also declare a
package dependency on `aeris_bringup`. The grid packages resolve the default
map through `AERIS_ROOT`; the launch files always pass the installed map path.

### 6.25 rclpy nodes have no `self.now()`

Use `self.get_clock().now().to_msg()` for message stamps. Calling
`self.now()` kills the node on its first status publication.

### 6.26 Allow offboard restart after DONE

A GUI demo must be able to select a second goal. The offboard controller
accepts `start` from `DONE`/`ABORTED` after readiness checks pass and
transitions back through `WAIT_FOR_READY`. Do not restore the old
`WAIT_FOR_READY`-only check.

### 6.27 Use `gz_x500` for the city grid

The camera airframe plus the large city world caused the Gazebo server to
crash during a long flight, which killed PX4 telemetry and triggered the
safety land path. The grid demo needs no camera; use `gz_x500`, which is
stable across multiple city missions.

---

## 7. Acceptance records

### Milestone 1 — telemetry foundation

| Check | Result |
|---|---|
| PX4 SITL boots | PASS |
| Gazebo shows X500 | PASS (GUI confirmed) |
| QGroundControl connects | PASS |
| DDS agent connects | PASS |
| `/fmu/*` topics visible | PASS |
| C++ telemetry node compiles/executes | PASS |
| Position telemetry continuous | PASS — 50.005 Hz |
| Estimator validity | PASS |

### Milestone 2 — offboard flight

| Check | Result |
|---|---|
| Offboard mode accepted | PASS |
| Arming accepted | PASS |
| Takeoff to 5 m | PASS — ~7.8 s |
| Position hold | PASS — 10 s, 0.01–0.03 m steady-state error |
| Landing accepted | PASS |
| Disarmed after landing | PASS |
| Post-landing preflight readiness | PASS — AUTO_LOITER, preflight true |
| Abnormal case: external land during HOLD | PASS — controlled landing, no crash |

### Milestone 3 — mission manager

| Check | Result |
|---|---|
| Mission YAML parser/validator | PASS — 11/11 tests |
| Two-waypoint mission | PASS — 2/2 |
| Inspection holds | PASS — 3 s each |
| RTL | PASS |
| Landing and disarm | PASS |
| Structured JSONL log | PASS |
| Final state | `COMPLETE`, success |

### Milestone 4 — perception

| Check | Result |
|---|---|
| Camera enabled | PASS — `gz_x500_mono_cam_down` |
| Camera bridge | PASS — 1280×960, ~12.8 Hz |
| Detector unit tests | PASS — 4/4 |
| Target detected at waypoints | PASS — 2/2 |
| Confidence | PASS — 1.00 |
| Detection in mission JSONL | PASS |
| Localization accuracy | APPROXIMATE — documented limitation |

### Milestone 5 — navigation and obstacle avoidance

| Check | Result |
|---|---|
| Lidar enabled | PASS — `gz_x500_lidar_2d`, 270°, ~30 Hz |
| Sector obstacle map | PASS |
| Navigation unit tests | PASS — 4/4 |
| Obstacle detected | PASS — 3.75 m, bearing −0.204 rad |
| Replan | PASS — detour `(1.54, 1.94, −5.0)` |
| Original waypoint reached | PASS |
| Mission | COMPLETE |

### Milestone 6 — safety, fault injection and failsafes

| Check | Result |
|---|---|
| Safety supervisor | PASS — 10 Hz `SafetyStatus` |
| Evaluator tests | PASS — 11/11 |
| Nominal flight | PASS — `safe=true` throughout, mission `COMPLETE` |
| `stale_pose` | PASS — `land`, `DONE`, disarmed |
| `low_battery` | PASS — `rtl`, PX4 RTL, disarmed |
| `geofence_breach` | PASS — `rtl`, PX4 RTL, disarmed |
| `mission_heartbeat_loss` | PASS — `rtl`, PX4 RTL, disarmed |
| JSONL `safety_abort` | PASS — every fault run |
| Workspace tests | PASS — 32/32 |

Evidence: `docs/experiments/week6_nominal_mission.jsonl`, `docs/experiments/week6_*_mission.jsonl`, `docs/experiments/week6_*_run.log`, `docs/experiments/week6_*_safety.log`.

### Milestone 7 — evaluation and demo packaging

| Check | Result |
|---|---|
| Evaluator | PASS — 8 runs, JSON + CSV + plots |
| Planned-completion success | PASS — 4/4 |
| Safe fault outcomes | PASS — 4/4 |
| Detection benchmark | PASS — 2/2, confidence 1.00 |
| Obstacle benchmark | PASS — 3.75 m detection, 1 replan |
| Safety decision latency | PASS — 41–98 ms |
| ROS package tests | PASS — 32/32 |
| Evaluator tests | PASS — 4/4 |
| Environment check | PASS — 18 ok |
| Report and demo runbook | PASS — `docs/evaluation.md`, `docs/demo.md` |

Evidence: `docs/experiments/benchmark.json`, `docs/experiments/benchmark.csv`, `docs/experiments/plots/`.

### Grid extension — city map, A* and GUI

| Check | Result |
|---|---|
| Grid map | PASS — 11×11, 4 m cells, 12 buildings, 44 occupied |
| A* planner | PASS — 9/9 tests, no corner cutting |
| GUI | PASS — click mapping test, live path/drone/progress |
| `(0,0)` mission | PASS — 11 cells / 40 m out and return |
| `(10,10)` mission | PASS — 11 cells / 40 m out and return |
| Repeat `(2,5)` from `DONE` | PASS — 4 cells / 12 m out and return |
| Path occupancy | PASS — zero occupied cells in any path |
| Building clearance | PASS — 2.269 m minimum, zero intrusions |
| Workspace tests | PASS — 42 ROS + 4 evaluator |
| Environment check | PASS — 20 ok |

Evidence: `docs/experiments/grid_mission.jsonl`, `docs/experiments/grid_repeat_goal.jsonl`, `docs/experiments/grid_repeat_short.jsonl`, `docs/experiments/grid_validation.txt`, `docs/experiments/plots/grid_gui*.png`.

---

## 8. Final state and future work

All seven planned weeks and the grid navigation extension are complete:

1. PX4/Gazebo/ROS 2 telemetry foundation.
2. Offboard arm, takeoff, hold and landing.
3. Mission manager, waypoints, RTL and JSONL logging.
4. Camera perception and OpenCV target detection.
5. Lidar obstacle detection and reactive replanning.
6. Safety supervision, fault injection and land/RTL failsafes.
7. Evaluation harness, benchmark report, plots and demo runbook.
8. City-grid occupancy, A* shortest-path navigation, return-to-start and a
   PyQt5 goal-selection GUI.

Future work, in priority order:

1. Repeat each scenario across many trials and add wind/obstacle randomization.
2. Validate the architecture on hardware or hardware-in-the-loop.
3. Calibrate camera extrinsics and replace the approximate ground projection.
4. Extend the grid planner with dynamic obstacles, 3D costs and path smoothing.
5. Add map editing and mission history to the GUI.
6. Record and publish the demo video using `docs/demo.md`.
7. Add CI for the build, ROS tests, evaluator tests and environment checks.

---

## 9. Copy-paste resume prompt

Paste this into a new AI session:

```text
You are resuming AERIS, a simulated industrial-inspection UAV project built on PX4 v1.17.0 and ROS 2 Jazzy.

Read these files first:
- claude.md (original brief)
- README.md
- docs/checkpoint.md
- docs/architecture.md
- docs/installation.md
- docs/flight_control.md
- docs/mission_design.md
- docs/perception.md
- docs/navigation.md
- docs/safety.md
- docs/evaluation.md
- docs/demo.md
- docs/grid_navigation.md
- docs/experiments/development_log.md
- docs/experiments/week3_mission.jsonl
- docs/experiments/week4_mission.jsonl
- docs/experiments/week5_mission.jsonl
- docs/experiments/week6_*_mission.jsonl
- docs/experiments/benchmark.json
- docs/experiments/grid_mission.jsonl
- docs/experiments/grid_validation.txt

Current state:
- All seven weeks and the city-grid navigation extension are complete.
- aeris_flight_control: 20 Hz offboard stream, arm, takeoff to 5 m, HOLD/TRANSIT/RTL/LAND, go_to/hold/land/rtl/abort, FlightStatus at 10 Hz.
- aeris_mission_manager: YAML validation, sequential waypoints, RTL, landing, obstacle replanning, safety aborts, JSONL logs; 11/11 tests pass.
- aeris_perception: downward mono camera, OpenCV red-panel detection, 4/4 tests; 2/2 detections at inspection waypoints, confidence 1.00.
- aeris_navigation: 270° lidar sector obstacle map and corridor check, 4/4 tests; obstacle mission logged a 3.75 m detection and detour to (1.54, 1.94).
- aeris_safety: pose/battery/geofence/heartbeat supervision, fault injection, land/RTL actions, 11/11 tests. All four injected faults were verified in SITL, with 41–98 ms decision latency.
- aeris_grid_navigation: 11×11 occupancy grid, 8-connected A* with no corner cutting, go-to-goal/return/land executor, 9/9 tests. Three SITL missions completed, including a repeat goal after DONE; minimum building clearance 2.269 m.
- aeris_grid_gui: PyQt5 click-to-goal, path, live drone and progress display, 1/1 headless test.
- Evaluation: scripts/evaluate_mission.py, 8 recorded runs, benchmark JSON/CSV, two plots, 4 evaluator tests.
- Workspace total: 42 ROS tests + 4 evaluator tests, 0 failures; environment check 20 ok.
- Perception uses gz_x500_mono_cam_down; navigation uses gz_x500_lidar_2d; the grid demo uses gz_x500 (camera + city world was unstable); ros_gz_bridge is installed.
- PYTHONNOUSERSITE=1 is required for apt OpenCV/NumPy; aeris_env.sh sets it.
- launch_simulation.sh keeps the SITL battery healthy with PX4_PARAM_SIM_BAT_MIN_PCT=100 and PX4_PARAM_SIM_BAT_DRAIN=86400.
- PX4 v1.17 uses _v1 topic suffixes for vehicle_local_position and vehicle_status.
- Always run `source scripts/aeris_env.sh` before ROS/PX4 commands; it keeps conda's Python 3.13 out of the ROS shell.
- Never pipe PX4's interactive console through tee; launch_simulation.sh uses `script` for PTY logging.

No further milestone is required. If extending the project, start with the future-work list in docs/checkpoint.md §8: repeated trials, hardware/HITL, calibrated perception, dynamic/3D grid planning, GUI map editing, demo video and CI.

Before changing code, run:
  source scripts/aeris_env.sh
  bash scripts/check_environment.sh
  bash scripts/build.sh

Do not upgrade PX4 or px4_msgs. Ask before changing the pinned versions.
```

---

## 10. Git and publishing

The repository is live at:

```text
git@github.com:Nikhil-URG/aeris-uav.git
https://github.com/Nikhil-URG/aeris-uav
```

Current branch: `main`. All checkpoint tags are pushed:

- `week-1-checkpoint` — Week 1 telemetry foundation (`0486c1e`)
- `week-2-checkpoint` — Week 2 offboard flight (`0666e09`)
- `week-3-checkpoint` — Week 3 mission manager (`ba262c9`)
- `week-4-checkpoint` — Week 4 perception (`8618b01`)
- `week-5-checkpoint` — Week 5 obstacle avoidance (`4237774`)
- `week-6-checkpoint` — Week 6 safety supervisor (`94e8f02`)
- `week-7-checkpoint` — Week 7 evaluation and demo (`b8c39df`)
- `grid-navigation-checkpoint` — city-grid A* and GUI (this checkpoint)

Push subsequent changes:

```bash
git push origin main
```

Create a future checkpoint:

```bash
git tag -a <milestone>-checkpoint -m "<description>"
git push origin <milestone>-checkpoint
```

The repository contains a submodule (`px4_msgs`). A fresh clone should use:

```bash
git clone --recursive git@github.com:Nikhil-URG/aeris-uav.git
```

If the clone was not recursive:

```bash
git submodule update --init --recursive
```

---

## 11. Quick health checklist after a break

```bash
cd /home/lucifer/Documents/Project_Aeris
source scripts/aeris_env.sh
bash scripts/check_environment.sh
git status
ros2 pkg list | grep -E '^(aeris_|px4_msgs)'
ros2 pkg executables aeris_flight_control
ros2 pkg executables aeris_mission_manager
ros2 pkg executables aeris_perception
ros2 pkg executables aeris_navigation
ros2 pkg executables aeris_safety
ros2 pkg executables aeris_grid_navigation
ros2 pkg executables aeris_grid_gui
python3 -m pytest tests/test_evaluate_mission.py -q
```

Expected:

- `20 ok, 0 warnings, 0 failures`
- clean working tree
- `aeris_bringup`, `aeris_interfaces`, `aeris_flight_control`, `aeris_mission_manager`, `aeris_perception`, `aeris_navigation`, `aeris_safety`, `aeris_grid_navigation`, `aeris_grid_gui`, `px4_msgs`
- executables: `telemetry_monitor`, `offboard_controller`, `mission_manager`, `perception`, `navigation_node`, `safety_node`, `grid_navigation`, `grid_gui`
- `ros_gz_bridge` installed
- `4 passed` for the evaluator tests

Then start the simulation and run the full mission demo as in §5, the city-grid demo as in §5.12, or follow `docs/demo.md`.
