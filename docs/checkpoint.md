# AERIS Checkpoint

**Checkpoint date:** 2026-11-09  
**Phase:** Week 2 complete — Week 3 (mission manager) is next  
**Repository:** `/home/lucifer/Documents/Project_Aeris`  
**Branch:** `main`  
**Week 1 tag:** `week-1-checkpoint` (commit `0486c1e`)  
**Week 2 tag:** `week-2-checkpoint` (this checkpoint commit)  
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
| Workspace build | `px4_msgs`, `aeris_flight_control`, `aeris_bringup` |
| Telemetry rate | **50.005 Hz** on `/fmu/out/vehicle_local_position_v1` |
| Offboard stream | **20 Hz** (`OffboardControlMode` + `TrajectorySetpoint`) |
| Idle heartbeat | **2 Hz** while no mission is active |
| Clean flight | Arm → 5 m → 10 s hold → land → disarm |
| Takeoff time | ~7.8 s to 5 m |
| Steady-state hold error | 0.01–0.03 m |
| Landing to disarm | ~10.9 s |
| Final post-landing state | `AUTO_LOITER`, disarmed, `pre_flight_checks_pass=true` |
| Abort path | External QGC land during HOLD → controller sent `NAV_LAND` → safe landing |
| QGC | Connected; PX4 logs `[mavlink] partner IP: 127.0.0.1` |
| Environment check | `12 ok, 0 warnings, 0 failures` |

Milestone 1 and Milestone 2 acceptance are complete.

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
│       └── week2_offboard_run.log  # clean Week 2 flight log
├── px4/README.md                   # upstream pin and upgrade policy
├── ros2_ws/src/
│   ├── px4_msgs/                   # Git submodule, v1.17.0
│   ├── aeris_flight_control/       # C++ telemetry + offboard controller
│   └── aeris_bringup/              # launch + YAML config
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

Key Week 2 files:

- `ros2_ws/src/aeris_flight_control/src/offboard_controller.cpp`
- `ros2_ws/src/aeris_bringup/config/offboard.yaml`
- `ros2_ws/src/aeris_bringup/launch/offboard.launch.py`
- `docs/flight_control.md`
- `docs/experiments/week2_offboard_run.log`

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

### 5.6 Verify topics

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

Evidence and timings are in `docs/experiments/development_log.md` and `docs/experiments/week2_offboard_run.log`.

---

## 8. Week 3 plan — mission manager

### Goal

Add a Python mission layer that owns inspection goals and mission state, while `aeris_flight_control` remains the only component that talks to PX4.

### Suggested deliverables

1. `aeris_interfaces` package with:
   - a mission goal/waypoint message or service;
   - a flight-status message (state, armed, altitude, target, error);
   - optional mission-state message.
2. `aeris_mission_manager` Python package:
   - YAML mission parser and validator (see `docs/mission_design.md`);
   - explicit state machine: `IDLE → PREFLIGHT → TAKEOFF → TRANSIT → INSPECT → REPLAN/TRANSIT → RTL/LAND → COMPLETE/ABORT`;
   - sequential waypoint execution with hold times;
   - structured JSONL event log in `results/logs/<run_id>/`;
   - mission timeout and abort handling.
3. `aeris_flight_control` extension:
   - accept waypoint goals from the mission manager (service or topic);
   - continue the 20 Hz offboard stream and report goal reached / failed;
   - preserve the existing manual `start`/`abort` services for direct testing.
4. `aeris_bringup` launch/config for a full mission.
5. Tests:
   - mission validation unit tests;
   - state transition tests;
   - at least one 2-waypoint SITL mission.

### Interface recommendation

Keep PX4 command authority in `aeris_flight_control`. Use a goal interface (for example `aeris_interfaces/srv/GoTo` or `aeris_interfaces/msg/WaypointGoal`) plus a status topic. The mission manager should not publish `/fmu/in/*` directly.

### Week 3 acceptance

- Load a 2-waypoint inspection mission.
- Take off, transit to waypoint 1, hold, transit to waypoint 2, hold, return/land.
- Reject invalid missions (geofence, altitude, duplicate IDs, bad frame).
- Produce a structured run log and a short report.
- No regression in the Week 2 manual offboard demo.

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

Current state:
- Week 1 and Week 2 are complete.
- The C++ offboard controller in aeris_flight_control is verified: 20 Hz offboard stream, arm, takeoff to 5 m, 10 s hold, land, disarm.
- Clean run: takeoff ~7.8 s, steady hold error 0.01-0.03 m, landing to disarm ~10.9 s.
- The controller exposes /aeris_offboard_controller/start and /abort (std_srvs/srv/Trigger).
- It publishes a 2 Hz idle OffboardControlMode heartbeat and restores AUTO_LOITER after landing, so preflight checks pass for a repeat run.
- PX4 v1.17 uses _v1 topic suffixes for vehicle_local_position and vehicle_status.
- Always run `source scripts/aeris_env.sh` before ROS/PX4 commands; it keeps conda's Python 3.13 out of the ROS shell.
- Never pipe PX4's interactive console through tee; launch_simulation.sh uses `script` for PTY logging.

Next task: implement Week 3, the mission manager.
Create aeris_interfaces and aeris_mission_manager, implement the mission YAML schema/validator and state machine, and integrate sequential waypoint goals with the existing offboard controller. Add validation/state tests and produce structured mission logs. Do not put /fmu/in publishers in the mission manager; flight-control authority stays in aeris_flight_control.

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

Current branch: `main`. Both checkpoint tags are pushed:

- `week-1-checkpoint` — Week 1 telemetry foundation (`0486c1e`)
- `week-2-checkpoint` — Week 2 offboard flight (this checkpoint)

Push subsequent changes:

```bash
git push origin main
```

Create a future checkpoint:

```bash
git tag -a week-3-checkpoint -m "Week 3 complete: mission manager verified"
git push origin week-3-checkpoint
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
```

Expected:

- `12 ok, 0 warnings, 0 failures`
- clean working tree
- `aeris_bringup`, `aeris_flight_control`, `px4_msgs`
- executables: `telemetry_monitor`, `offboard_controller`

Then start the simulation and run the offboard demo as in §5.
