# AERIS Checkpoint

**Checkpoint date:** 2026-11-09  
**Phase:** Week 3 complete — Week 4 (perception) is next  
**Repository:** `/home/lucifer/Documents/Project_Aeris`  
**Branch:** `main`  
**Week 1 tag:** `week-1-checkpoint` (commit `0486c1e`)  
**Week 2 tag:** `week-2-checkpoint` (commit `0666e09`)  
**Week 3 tag:** `week-3-checkpoint` (this checkpoint commit)  
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
| Workspace build | `px4_msgs`, `aeris_interfaces`, `aeris_flight_control`, `aeris_mission_manager`, `aeris_bringup` |
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
| QGC | Connected; PX4 logs `[mavlink] partner IP: 127.0.0.1` |
| Environment check | `14 ok, 0 warnings, 0 failures` |

Milestone 1, 2 and 3 acceptance are complete.

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
│   ├── checkpoint.md               # this file
│   └── experiments/
│       ├── development_log.md
│       ├── week2_offboard_run.log  # clean Week 2 flight log
│       ├── week3_mission.jsonl     # structured Week 3 mission events
│       └── week3_mission_run.log   # filtered Week 3 console evidence
├── px4/README.md                   # upstream pin and upgrade policy
├── ros2_ws/src/
│   ├── px4_msgs/                   # Git submodule, v1.17.0
│   ├── aeris_interfaces/           # AERIS msgs/srvs
│   ├── aeris_flight_control/       # C++ telemetry + offboard controller
│   ├── aeris_mission_manager/      # Python mission manager
│   └── aeris_bringup/              # launch + YAML + missions
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
│   ├── check_environment.sh
│   └── evaluate_mission.py         # Week 7 placeholder
├── simulation/                     # worlds/models/scenarios
├── tests/                          # integration/scenario tests
└── results/                        # logs/plots/reports (ignored)
```

Key Week 2–3 files:

- `ros2_ws/src/aeris_flight_control/src/offboard_controller.cpp`
- `ros2_ws/src/aeris_mission_manager/aeris_mission_manager/mission.py`
- `ros2_ws/src/aeris_mission_manager/aeris_mission_manager/node.py`
- `ros2_ws/src/aeris_bringup/config/offboard.yaml`
- `ros2_ws/src/aeris_bringup/config/offboard_mission.yaml`
- `ros2_ws/src/aeris_bringup/config/missions/inspection_alpha.yaml`
- `ros2_ws/src/aeris_bringup/launch/offboard.launch.py`
- `ros2_ws/src/aeris_bringup/launch/mission.launch.py`
- `docs/flight_control.md`
- `docs/experiments/week2_offboard_run.log`
- `docs/experiments/week3_mission.jsonl`

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

Expected: system Python 3.12, ROS Jazzy, Gazebo 8.x, PX4 `v1.17.0`, agent installed, workspace built, `12 ok, 0 warnings, 0 failures`.

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

Evidence and timings are in `docs/experiments/development_log.md`, `docs/experiments/week2_offboard_run.log` and `docs/experiments/week3_mission.jsonl`.

---

## 8. Week 4 plan — perception and target localization

### Goal

Integrate the simulated camera, detect an inspection target with OpenCV, estimate bearing/range, and feed detections into the mission log during INSPECT.

### Prerequisite

`ros-jazzy-ros-gz` is **not installed yet** and is required for the Gazebo image bridge. Install it before the SITL camera test:

```bash
sudo apt update
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-vision-msgs
```

`cv_bridge`, `image_transport`, `message_filters` and OpenCV development files are already installed.

### Deliverables

1. `aeris_interfaces`:
   - `Detection.msg` and `DetectionArray.msg`
   - target id/class, confidence, pixel bbox, bearing, range, vehicle-relative position.
2. `aeris_perception` Python package:
   - camera image subscriber using `cv_bridge`;
   - OpenCV color/contour detection (no YOLO);
   - optional depth-image range estimation;
   - detection publisher and optional CSV/JSONL output.
3. Simulation:
   - target model and spawn procedure (red target panel);
   - run PX4 with `gz_x500_mono_cam` or `gz_x500_depth`;
   - bridge Gazebo camera topics into ROS 2 with `ros_gz_bridge`.
4. Mission manager integration:
   - subscribe to detections during INSPECT and log `target_detected`.
5. Tests:
   - synthetic-image unit tests for the detector;
   - at least one SITL detection at a waypoint.

### Suggested detection approach

- Red/orange panel target with HSV thresholding and contour centroid.
- Bearing from camera intrinsics and pixel offset.
- Range from a depth image at the target centroid, or from known target size in monocular mode.
- Transform camera-frame estimate into local NED using the vehicle pose/yaw.
- If `cv2.aruco` is available, support ArUco as an alternative detector.

### Week 4 acceptance

- Camera stream visible through `ros_gz_bridge`.
- Target detected at 3–6 m with a confidence score.
- Detection published and recorded in the mission JSONL.
- Synthetic-image detector tests pass.

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
- docs/experiments/development_log.md
- docs/experiments/week2_offboard_run.log
- docs/experiments/week3_mission.jsonl

Current state:
- Weeks 1–3 are complete.
- aeris_flight_control: 20 Hz offboard stream, arm, takeoff to 5 m, HOLD/TRANSIT, go_to, land, abort, FlightStatus at 10 Hz.
- The offboard controller publishes a 2 Hz idle heartbeat, restores AUTO_LOITER after landing, and uses a 1 s telemetry freshness guard.
- aeris_mission_manager: YAML validation, sequential waypoints, RTL, landing, JSONL logs; 11/11 tests pass.
- Verified mission: inspection_alpha, 2/2 waypoints, 3 s inspection holds, RTL, land, COMPLETE in ~33.8 s.
- Flight evidence: docs/experiments/week3_mission.jsonl.
- PX4 v1.17 uses _v1 topic suffixes for vehicle_local_position and vehicle_status.
- Always run `source scripts/aeris_env.sh` before ROS/PX4 commands; it keeps conda's Python 3.13 out of the ROS shell.
- Never pipe PX4's interactive console through tee; launch_simulation.sh uses `script` for PTY logging.

Next task: implement Week 4, perception and target localization.
Install ros-jazzy-ros-gz and ros-jazzy-vision-msgs if not already present. Add Detection/DetectionArray interfaces, an aeris_perception OpenCV detector, a Gazebo camera bridge, a red target model, and mission-manager INSPECT integration. Add synthetic-image detector tests and one SITL detection run.

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
- `week-3-checkpoint` — Week 3 mission manager (this checkpoint)

Push subsequent changes:

```bash
git push origin main
```

Create a future checkpoint:

```bash
git tag -a week-4-checkpoint -m "Week 4 complete: perception verified"
git push origin week-4-checkpoint
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
```

Expected:

- `14 ok, 0 warnings, 0 failures`
- clean working tree
- `aeris_bringup`, `aeris_interfaces`, `aeris_flight_control`, `aeris_mission_manager`, `px4_msgs`
- executables: `telemetry_monitor`, `offboard_controller`, `mission_manager`

Then start the simulation and run the full mission demo as in §5.
