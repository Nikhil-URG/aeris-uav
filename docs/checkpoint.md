# AERIS Checkpoint

**Checkpoint date:** 2026-11-09  
**Phase:** Week 1 complete — Week 2 (offboard controller) is next  
**Repository:** `/home/lucifer/Documents/Project_Aeris`  
**Branch:** `main`  
**Current HEAD:** run `git log -1 --oneline` (this checkpoint is the latest commit)  
**Planned remote:** `git@github.com:Nikhil-URG/aeris-uav.git`

This document is the portable handoff. It records what is verified, how to reproduce the environment, what bit us along the way, and what to do next. Read this before touching the code after a break.

---

## 1. Project context

AERIS is a simulated industrial-inspection UAV built on **PX4 + ROS 2 Jazzy**. PX4 owns low-level flight stabilization (EKF2, position/attitude control, actuators). The ROS 2 application owns mission-level intelligence and talks to PX4 through **uXRCE-DDS**.

Week 1 established the simulation foundation and a read-only C++ telemetry monitor:

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
    └── telemetry_monitor
  aeris_bringup (Python)
    └── telemetry.launch.py
```

Week 2 adds the offboard controller: readiness checks, offboard heartbeat, mode request, arming, takeoff to 5 m, position hold and controlled landing.

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
| NVIDIA driver | `595.104.02` (after reboot; `nvidia-smi` works) |
| Workspace build | 3 packages: `px4_msgs`, `aeris_flight_control`, `aeris_bringup` |
| Telemetry rate | **50.005 Hz** on `/fmu/out/vehicle_local_position_v1` |
| Estimator validity | `PosValid=YES VelValid=YES` |
| GUI | User confirmed the X500 is visible in Gazebo |
| QGC | Connected; PX4 logs `[mavlink] partner IP: 127.0.0.1` and `[commander] Ready for takeoff!` |
| Environment check | `11 ok, 0 warnings, 0 failures` |
| Disk free | ~35 GiB at checkpoint |

Milestone 1 acceptance is complete.

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
│   ├── flight_control.md           # telemetry contract + Week 2 design
│   ├── mission_design.md           # Week 3 state machine and mission schema
│   ├── checkpoint.md               # this file
│   └── experiments/development_log.md
├── px4/README.md                   # upstream pin and upgrade policy
├── ros2_ws/src/
│   ├── px4_msgs/                   # Git submodule, v1.17.0
│   ├── aeris_flight_control/       # C++ telemetry node
│   └── aeris_bringup/              # launch + YAML config
├── scripts/
│   ├── setup.sh                    # one-command bootstrap
│   ├── aeris_env.sh                # conda-safe ROS shell
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

Key files for the next milestone:

- `ros2_ws/src/aeris_flight_control/src/telemetry_monitor.cpp`
- `ros2_ws/src/aeris_flight_control/CMakeLists.txt`
- `ros2_ws/src/aeris_bringup/config/telemetry.yaml`
- `ros2_ws/src/aeris_bringup/launch/telemetry.launch.py`
- `docs/flight_control.md` (offboard sequence and safety interlocks)

---

## 5. How to resume on any machine

### 5.1 Clone and initialize

```bash
git clone --recursive git@github.com:Nikhil-URG/aeris-uav.git
cd aeris-uav
# If cloned without --recursive:
# git submodule update --init --recursive
```

If the remote has not been created yet, see §10.

### 5.2 Environment

```bash
source scripts/aeris_env.sh
bash scripts/check_environment.sh
```

Expected: system Python 3.12, ROS Jazzy, Gazebo 8.x, PX4 `v1.17.0`, agent installed, workspace built, `11 ok, 0 warnings, 0 failures`.

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

### 5.4 Run

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

QGC (optional but verified):

```bash
scripts/run_qgroundcontrol.sh
```

### 5.5 Verify topics

```bash
source scripts/aeris_env.sh
ros2 topic list | grep '/fmu/'
ros2 topic hz /fmu/out/vehicle_local_position_v1
ros2 topic echo --once /fmu/out/vehicle_local_position_v1
```

Expected: ~50 Hz, `xy_valid: true`, `z_valid: true`, `v_xy_valid: true`, `v_z_valid: true`.

---

## 6. Hard-won gotchas

### 6.1 Conda leaks into CMake

Even with `PATH` cleaned, `CONDA_PREFIX` made CMake select `/home/lucifer/miniforge3/bin/python3` (3.13). The build then failed with `ModuleNotFoundError: No module named 'em'`.

`scripts/aeris_env.sh` now unsets `CONDA_PREFIX`, `CONDA_DEFAULT_ENV`, `CONDA_SHLVL`, `CONDA_PYTHON_EXE`, `CONDA_EXE`, `_CONDA_EXE`, `_CONDA_ROOT`, `_CE_CONDA`, and strips conda paths from `PATH`, `PYTHONPATH`, `LD_LIBRARY_PATH`, `CMAKE_PREFIX_PATH` and `PKG_CONFIG_PATH`.

If a build already cached the wrong interpreter:

```bash
rm -rf ros2_ws/build ros2_ws/install ros2_ws/log
bash scripts/build.sh
```

Check `_Python3_EXECUTABLE:INTERNAL` in `ros2_ws/build/<package>/CMakeCache.txt`; it must point to `/usr/bin/python3`.

### 6.2 ROS setup scripts are not `set -u` safe

`/opt/ros/jazzy/local_setup.sh` reads `AMENT_PYTHON_EXECUTABLE` without a default. Sourcing it under `set -u` aborts with `unbound variable`. `scripts/aeris_env.sh` disables nounset only while sourcing ROS, then restores the caller's setting.

### 6.3 PX4 v1.17 uses `_v1` DDS topic suffixes

The active local-position topic is:

```text
/fmu/out/vehicle_local_position_v1
```

not the unsuffixed name used in the original brief. The unsuffixed name can appear in the ROS graph if a subscriber creates it, but it has no publisher. This applies to other versioned messages such as `vehicle_status_v1`.

Always discover topic names after starting SITL:

```bash
ros2 topic list | grep '/fmu/out'
```

### 6.4 Never pipe PX4's interactive shell through `tee`

PX4 redraws its prompt continuously when stdout is a pipe. An early run produced two **7 GB** log files. `scripts/launch_simulation.sh` now runs PX4 under a PTY with:

```bash
script -q -e -f -c "make px4_sitl gz_x500" "$LOG_DIR/px4_sitl.log"
```

It also rotates a PX4 log larger than 200 MB to `px4_sitl.log.prev`. Do not revert to `| tee`.

### 6.5 QGC on Wayland

`scripts/run_qgroundcontrol.sh` forces `QT_QPA_PLATFORM=xcb`. If QGC misbehaves, use that wrapper rather than launching the AppImage directly. The AppImage may ignore SIGTERM; stop it with SIGKILL if needed.

### 6.6 NVIDIA driver mismatch

The first session reported `Failed to initialize NVML: Driver/library version mismatch`. A reboot resolved it; the driver now reports `595.104.02`. If the warning returns after a system update, reboot before diagnosing Gazebo rendering.

### 6.7 PX4 is external

PX4-Autopilot is **not** vendored. It lives at `~/aeris_dev/PX4-Autopilot` and is pinned to `v1.17.0` (commit `d6f12ad1c4`). The `px4_msgs` submodule is pinned to the matching `v1.17.0` commit. Do not upgrade either independently; follow `px4/README.md`.

---

## 7. Milestone 1 acceptance record

| Check | Result |
|---|---|
| PX4 SITL boots | PASS |
| Gazebo shows X500 | PASS (GUI confirmed) |
| QGroundControl connects | PASS |
| DDS agent connects | PASS |
| `/fmu/*` topics visible | PASS |
| C++ telemetry node compiles/executes | PASS |
| Position telemetry continuous | PASS — 50.005 Hz |
| Estimator validity | PASS — `PosXY/PosZ/Vel` valid |

Evidence and timings are in `docs/experiments/development_log.md`.

---

## 8. Week 2 plan — offboard controller

### Goal

Add a C++ node (working name `offboard_controller`) to `aeris_flight_control` that:

1. Waits for valid local position and vehicle status.
2. Streams `/fmu/in/offboard_control_mode` and `/fmu/in/trajectory_setpoint` continuously.
3. Requests offboard mode.
4. Arms the vehicle.
5. Takes off to 5 m (`z = -5.0` NED).
6. Holds position within a documented tolerance.
7. Commands a controlled landing.
8. Stops the setpoint stream on any fault and logs why.

### Topics to use

| Direction | Topic | Type |
|---|---|---|
| Subscribe | `/fmu/out/vehicle_local_position_v1` | `px4_msgs/msg/VehicleLocalPosition` |
| Subscribe | `/fmu/out/vehicle_status_v1` | `px4_msgs/msg/VehicleStatus` |
| Subscribe | `/fmu/out/vehicle_control_mode` | `px4_msgs/msg/VehicleControlMode` |
| Subscribe | `/fmu/out/failsafe_flags` | `px4_msgs/msg/FailsafeFlags` |
| Publish | `/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` |
| Publish | `/fmu/in/trajectory_setpoint` | `px4_msgs/msg/TrajectorySetpoint` |
| Publish | `/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` |

Verify exact names with `ros2 topic list`; do not assume they match older examples.

### Safety interlocks

- Never request offboard or arm without valid position/velocity.
- Stream setpoints for at least 1 s before requesting offboard.
- Stop the stream if validity is lost, the setpoint age exceeds a threshold, PX4 leaves offboard mode, or the mission supervisor heartbeat disappears (Week 6).
- Verify PX4's offboard-loss action (`COM_OF_LOSS_T` and related failsafe parameters) in simulation before autonomous missions.
- Log every mode request, arming transition and fault.

### Acceptance tests

- Offboard heartbeat accepted; mode switch succeeds.
- Arm, take off to 5 m, hold within tolerance, land.
- Kill the setpoint publisher mid-flight; verify PX4 failsafe behaviour.
- Invalid pose; verify the controller refuses to arm.
- Measure mode-switch latency, setpoint age and position-hold error.

### Suggested order

1. Add `offboard_controller.cpp` and a small state machine.
2. Parameterize altitude, rates, tolerances and timeouts in `aeris_bringup/config/offboard.yaml`.
3. Add `offboard.launch.py`.
4. Unit-test the state machine where possible.
5. Run the nominal flight, then the fault cases.
6. Record results in `docs/flight_control.md` and the development log.

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
- docs/experiments/development_log.md

Current state:
- Week 1 is complete: PX4 SITL, Gazebo Harmonic 8.15, QGroundControl, uXRCE-DDS Agent v2.4.3, and the C++ telemetry monitor are verified.
- Telemetry: /fmu/out/vehicle_local_position_v1 at ~50 Hz, PosValid=YES, VelValid=YES.
- The working tree is clean; the repo is /home/lucifer/Documents/Project_Aeris on branch main.
- PX4 lives outside the repo at ~/aeris_dev/PX4-Autopilot (pinned v1.17.0).
- Always run `source scripts/aeris_env.sh` before ROS/PX4 commands; it keeps conda's Python 3.13 out of the ROS shell.
- PX4 v1.17 appends `_v1` to versioned DDS topics. Never assume unsuffixed names.
- Never pipe PX4's interactive console through tee; launch_simulation.sh uses `script` for PTY logging.

Next task: implement the Week 2 offboard controller in aeris_flight_control:
readiness checks, offboard control mode + trajectory setpoint heartbeat, offboard mode request, arming, takeoff to 5 m (NED z=-5.0), position hold, and controlled landing. Add parameters/launch config and tests for invalid pose and setpoint-stream loss. Update docs and the development log with measured results.

Before changing code, run:
  source scripts/aeris_env.sh
  bash scripts/check_environment.sh
  bash scripts/build.sh

Do not upgrade PX4 or px4_msgs. Ask before changing the pinned versions.
```

---

## 10. Git and publishing

SSH authentication to GitHub is configured and works as `Nikhil-URG`. The planned repository is:

```text
git@github.com:Nikhil-URG/aeris-uav.git
```

Current branch: `main`.

When the empty remote repository exists:

```bash
git remote add origin git@github.com:Nikhil-URG/aeris-uav.git
git push -u origin main
```

The repository contains a submodule (`px4_msgs`). A fresh clone should use:

```bash
git clone --recursive git@github.com:Nikhil-URG/aeris-uav.git
```

If the clone was not recursive:

```bash
git submodule update --init --recursive
```

### Creating the remote

GitHub does not create repositories over SSH. The remote must be created through the GitHub web UI or an authenticated API token / GitHub CLI. If `gh` is available:

```bash
gh auth login
gh repo create Nikhil-URG/aeris-uav --private --source=. --remote=origin --push
```

Otherwise create an empty repository (no README, no `.gitignore`, no license) at:

<https://github.com/new?name=aeris-uav>

then add the remote and push.

---

## 11. Quick health checklist after a break

```bash
cd /home/lucifer/Documents/Project_Aeris
source scripts/aeris_env.sh
bash scripts/check_environment.sh
git status
ros2 pkg list | grep -E '^(aeris_|px4_msgs)'
```

Expected:

- `11 ok, 0 warnings, 0 failures`
- clean working tree
- `aeris_bringup`, `aeris_flight_control`, `px4_msgs`

Then start the simulation and telemetry as in §5.
