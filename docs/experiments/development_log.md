# AERIS Development Log

This is an engineering journal, not a changelog. Record what was attempted, what worked, what failed, why decisions were made, and what was measured.

---

## 2026-11-02 — Environment audit and Week 1 bootstrap

### Goal

Establish the machine state before installing anything, then create a reproducible Week 1 scaffold for the simulation foundation and telemetry monitor.

### Environment observed

| Item | Value |
|---|---|
| OS | Pop!_OS 24.04 LTS (Ubuntu 24.04 `noble` base) |
| Kernel | `7.1.5-76070105-generic` |
| CPU | Intel Core i5-13420H, 8 cores / 12 threads |
| RAM | 15 GiB total, 9.2 GiB available, 19 GiB swap |
| GPU | NVIDIA RTX 4050 Laptop, 6 GiB, driver 580.173.02 |
| Disk | 54 GiB free on `/` (88% used) |
| Session | Wayland / COSMIC, XWayland display `:1` |
| Python | System 3.12.3; Miniforge 3.13.13 first in `PATH` |
| ROS 2 | not installed |
| Gazebo | not installed |
| Build tools | `git`, `gcc`, `g++`, `make`, `terminator` present; `cmake`, `ninja`, `colcon`, `vcstool`, `rosdep` missing |
| Sudo | password required |

### Findings that changed the plan

1. **Conda shadows the system Python.**
   - `command -v python3` resolves to `/home/lucifer/miniforge3/bin/python3` (3.13.13).
   - ROS 2 Jazzy and PX4 SITL require system Python 3.12.
   - PX4's `Tools/setup/ubuntu.sh` would install its Python requirements into the conda base environment if run unchanged.
   - Decision: add `scripts/aeris_env.sh` to sanitize `PATH`, `PYTHONPATH` and `LD_LIBRARY_PATH`; make the PX4 bootstrap script refuse to run with the wrong interpreter. Do not modify the user's global conda configuration.

2. **ROS apt HTTPS certificate mismatch.**
   - `https://packages.ros.org` resolves to an OSUOSL mirror with a certificate for `*.osuosl.org`, so TLS validation fails.
   - `http://packages.ros.org` works, and the official `ros2-apt-source` 1.3.0 package configures HTTP.
   - Decision: use the official apt source package, pinned to 1.3.0; document the observation.

3. **Disk is adequate but not generous.**
   - 54 GiB free; PX4, Gazebo, ROS desktop and build artifacts will consume a significant fraction.
   - Decision: skip the blanket `apt upgrade`, install PX4 without the NuttX toolchain by default, and keep result artifacts out of Git.

4. **Wayland/COSMIC.**
   - Gazebo and QGC should run through XWayland (`QT_QPA_PLATFORM=xcb` fallback).
   - Headless PX4/Gazebo runs will be used for automated checks.

### Decisions

| ID | Decision | Rationale |
|---|---|---|
| D-001 | Repository root is `/home/lucifer/Documents/Project_Aeris` | It is the active workspace; the Git repository can still be named `aeris-uav` |
| D-002 | PX4 remains external at `~/aeris_dev/PX4-Autopilot` | Keeps this repository reviewable and pins upstream explicitly |
| D-003 | Pin PX4 `v1.17.0` (commit `d6f12ad`), `px4_msgs` `v1.17.0` (commit `86d8239`), Micro XRCE-DDS Agent `v2.4.3` (commit `7362281`) | Reproducibility |
| D-004 | Use a `px4_msgs` Git submodule at `ros2_ws/src/px4_msgs` | Records the exact message revision in the superproject |
| D-005 | Keep `aeris_flight_control` and `aeris_bringup` as the only active packages in Week 1 | Avoid building architecture that has no milestone yet |
| D-006 | Default PX4 setup without NuttX; opt in with `AERIS_INSTALL_NUTTX=1` | SITL does not need the embedded toolchain; saves disk/time |
| D-007 | Use best-effort QoS on `/fmu/...` topics | Matches PX4 uXRCE-DDS publications |

### Implemented

- Repository scaffold for Week 1.
- `aeris_flight_control` C++ package with the read-only `telemetry_monitor` node.
- `aeris_bringup` Python package with a telemetry launch file and YAML parameters.
- `px4_msgs` submodule pinned to `v1.17.0`.
- Setup scripts for system/ROS, PX4/Gazebo, Micro XRCE-DDS Agent, QGC, build, simulation launch, environment checks, and a one-command orchestrator (`scripts/setup.sh`).
- Architecture, installation, flight-control and mission-design documentation.

### Commands run (audit only, no installs yet)

```bash
cat /etc/os-release
uname -r
python3 --version
which -a python3
nvidia-smi
df -h ~
git ls-remote --tags https://github.com/PX4/PX4-Autopilot.git refs/tags/v1.17.0
git ls-remote --tags https://github.com/PX4/px4_msgs.git refs/tags/v1.17.0
```

### Pending

- Run `scripts/install_dependencies.sh`.
- Run `scripts/bootstrap_px4.sh`.
- Run `scripts/install_microxrce_agent.sh`.
- Build the workspace and run the first telemetry session.
- Record the Milestone 1 acceptance results below.

### Milestone 1 run — completed 2026-11-09

| Check | Result | Evidence |
|---|---|---|
| PX4 SITL boots | PASS | `pxh>` prompt; `INFO [init] Gazebo world is ready` |
| Gazebo shows X500 | PASS | GUI confirmed by user after reboot |
| QGroundControl connects | PASS | QGC running; PX4 log: `[mavlink] partner IP: 127.0.0.1` and `[commander] Ready for takeoff!` |
| DDS agent connects | PASS | `/fmu/in/*` and `/fmu/out/*` topics present |
| ROS 2 topics | PASS | `/fmu/out/vehicle_local_position_v1`, `/fmu/out/vehicle_status_v1`, ... |
| C++ telemetry node builds | PASS | `colcon build`: 3 packages finished |
| Position telemetry continuous | PASS | 50.005 Hz on `/fmu/out/vehicle_local_position_v1` |
| Estimator validity | PASS | `PosValid=YES VelValid=YES` at rest |

Measured message rate: **50.005 Hz** on `/fmu/out/vehicle_local_position_v1` (min interval 0.016 s, max 0.024 s, std dev 0.0004 s).

First telemetry output:

```text
[aeris_telemetry_monitor]: Position [NED] X=0.02 Y=-0.00 Z=0.03 | Velocity [NED] VX=0.00 VY=-0.00 VZ=-0.00 | Speed=0.00 m/s | PosValid=YES VelValid=YES | Moving=NO
```

---

## 2026-11-09 — Workspace build and environment fixes

### Goal

Complete the first workspace build and start the headless simulation after the initial setup run.

### What had already succeeded

- ROS 2 Jazzy installed.
- PX4-Autopilot `v1.17.0` at `~/aeris_dev/PX4-Autopilot` (commit `d6f12ad1c4`).
- Gazebo Harmonic `8.15.0` installed.
- Micro XRCE-DDS Agent `v2.4.3` installed at `/usr/local/bin/MicroXRCEAgent`.
- QGroundControl `v5.1.5` AppImage installed.
- `px4_msgs` submodule at `86d8239` (`v1.17.0`).

### Failure 1 — `AMENT_PYTHON_EXECUTABLE: unbound variable`

Symptom:

```text
/opt/ros/jazzy/local_setup.sh: line 23: AMENT_PYTHON_EXECUTABLE: unbound variable
```

Cause: `set -u` (nounset) in the AERIS scripts was inherited by ROS 2's generated setup scripts. `local_setup.sh` reads optional variables such as `AMENT_PYTHON_EXECUTABLE` without defaults. ROS setup scripts are not nounset-safe.

Fix: `scripts/aeris_env.sh` now records whether nounset is enabled, disables it while sourcing `/opt/ros/jazzy/setup.bash` and the workspace setup, then restores the caller's setting.

Verification:

```bash
bash -c 'set -euo pipefail; source scripts/aeris_env.sh; echo "flags=$-"'
```

### Failure 2 — CMake selected conda Python

Symptom: the `px4_msgs` build failed inside `rosidl_adapter`:

```text
execute_process(/home/lucifer/miniforge3/bin/python3 -m rosidl_adapter ...)
ModuleNotFoundError: No module named 'em'
```

Cause: cleaning `PATH` was not sufficient. `CONDA_PREFIX` remained set, and CMake's `FindPython3` treats an active conda prefix as a virtual environment. It selected Python 3.13, which does not have ROS 2's `em` module. The generated cache recorded `_Python3_EXECUTABLE:INTERNAL=/home/lucifer/miniforge3/bin/python3`.

Fix: `scripts/aeris_env.sh` now:

- calls `conda deactivate` when the conda shell function is available;
- strips conda entries from `PATH`, `PYTHONPATH`, `LD_LIBRARY_PATH`, `CMAKE_PREFIX_PATH` and `PKG_CONFIG_PATH`;
- unsets `CONDA_PREFIX`, `CONDA_DEFAULT_ENV`, `CONDA_SHLVL`, `CONDA_PYTHON_EXE`, `CONDA_EXE`, `_CONDA_EXE`, `_CONDA_ROOT` and `_CE_CONDA`.

The stale build tree was removed (`ros2_ws/build`, `ros2_ws/install`, `ros2_ws/log`) and the clean build succeeded:

```text
Finished <<< px4_msgs [4min 10s]
Finished <<< aeris_flight_control [11.5s]
Finished <<< aeris_bringup [1.24s]
Summary: 3 packages finished [4min 23s]
```

### Finding — PX4 v1.17 uses versioned DDS topic names

The brief and the initial node used `/fmu/out/vehicle_local_position`. In PX4 v1.17 the active publisher is `/fmu/out/vehicle_local_position_v1`, because the `px4_msgs` message declares `MESSAGE_VERSION = 1`. The unsuffixed topic appeared in the ROS graph only because the node had created a subscription to it; it had no publisher.

Verified:

```text
$ ros2 topic info /fmu/out/vehicle_local_position_v1
Type: px4_msgs/msg/VehicleLocalPosition
Publisher count: 1
```

Updated the node default, `aeris_bringup/config/telemetry.yaml`, and the architecture/installation/flight-control docs. `claude.md` is left unchanged as the original brief. Other versioned topics (for example `vehicle_status_v1`) follow the same suffix convention; Week 2 offboard work must use the exact discovered topic names rather than assuming them.

### Finding — PX4 shell prompt can flood piped logs

The first headless run piped `make px4_sitl gz_x500` through `tee`. PX4's interactive shell continuously redrew its prompt because stdout was a pipe, producing `results/logs/px4_sitl.log` and `results/logs/launch_sim_console.log` at **7.0 GB each** in a short session.

Fix:

- run PX4 under a PTY with `script -q -e -f -c "make px4_sitl gz_x500" ...`;
- do not pipe the interactive console through `tee`;
- rotate the PX4 log if it exceeds 200 MB.

Verification: a later short headless session produced a **6.3 KB** `px4_sitl.log`, and the telemetry monitor still received valid 50 Hz data.

### Observation — NVIDIA NVML driver/library mismatch

`nvidia-smi` and `scripts/check_environment.sh` report:

```text
Failed to initialize NVML: Driver/library version mismatch
```

The userspace driver and the loaded kernel module were out of sync after a package update. A reboot resolved it: the driver now reports `595.104.02` and the GUI works. Headless simulation remained the reliable first validation path.

### GUI and QGC verification

After the reboot, the user confirmed:

- Gazebo GUI displays the X500.
- QGC connects to PX4.
- PX4 logs `[mavlink] partner IP: 127.0.0.1` and `[commander] Ready for takeoff!`.
- The PX4 SITL log remains small (7.7 KB) under the `script` PTY wrapper.

**Week 1 acceptance is complete.** Next: Week 2 offboard controller.

---

## 2026-11-09 — Week 2: offboard controller and first autonomous flight

### Goal

Implement the Week 2 C++ offboard controller: readiness checks, offboard heartbeat, mode request, arming, takeoff to 5 m, position hold and controlled landing.

### Implemented

- `aeris_flight_control/src/offboard_controller.cpp`
- `aeris_bringup/config/offboard.yaml` and `launch/offboard.launch.py`
- State machine: `WAIT_FOR_READY → PRESTREAM → REQUEST_OFFBOARD → ARM → TAKEOFF → HOLD → LAND → DONE`, with `ABORT_LAND → ABORTED` on fault.
- Services `~/start` and `~/abort` (`std_srvs/srv/Trigger`).
- 20 Hz `OffboardControlMode` + `TrajectorySetpoint` stream (PX4 `COM_OF_LOSS_T` default 1.0 s).
- Safety checks: pose/velocity validity, status freshness, preflight checks, disarmed state, failsafe, per-state timeouts, horizontal/altitude limits, hold-error limit.
- PX4-time timestamps taken from `VehicleLocalPosition`.

### Flight test — clean final run

Artifact: `docs/experiments/week2_offboard_run.log`

| Event | Result |
|---|---|
| `DO_SET_MODE` (offboard) | accepted |
| `ARM_DISARM` | accepted |
| `NAV_LAND` | accepted |
| Takeoff to 5 m | ~7.8 s |
| Hold duration | 10 s |
| Steady-state hold error | 0.01–0.03 m |
| Landing to disarm | ~10.9 s |
| Final state | disarmed, `AUTO_LOITER`, `pre_flight_checks_pass=true` |
| PX4 events | Takeoff detected, Landing detected, Disarmed by landing |

### Abort-path test

During a separate run, an external QGC land command arrived while the controller was in `HOLD`. The controller detected that PX4 had left offboard mode, sent its own `NAV_LAND`, stopped the stream, and the vehicle landed and disarmed safely. This validated the abort path even though the run ended in `ABORTED` rather than `DONE`.

### Issues and decisions

1. **Offboard signal loss blocks preflight after landing.**
   - PX4's `offboard_control_signal_lost` remains true after the node stops streaming and keeps `pre_flight_checks_pass` false.
   - Fix: publish a 2 Hz idle `OffboardControlMode` heartbeat while no mission is streaming.

2. **POSCTL is not a good post-landing mode for this SITL setup.**
   - POSCTL requires a manual-control link; this GCS-only setup has none.
   - Fix: restore `AUTO_LOITER` after landing, which allows preflight checks to pass for a subsequent run.

3. **External land during HOLD looked like a failure.**
   - Fix: if offboard is lost and `nav_state == AUTO_LAND`, transition to `LAND` rather than `ABORT_LAND`. The abort path remains for other faults.

4. **Stale offboard launch processes interfered with a test.**
   - Two `ros2 launch` instances from earlier experiments were still alive and both published heartbeats.
   - Operational note: verify the process list before a flight; stop old `offboard_controller`/`ros2 launch` instances.

5. **PX4 v1.17 `_v1` topic suffix applies to `vehicle_status_v1` as well.**
   - The controller uses the discovered names, not older examples.

### Measurements

- Setpoint rate: 20 Hz.
- Takeoff ramp: target 1.0 m/s, vehicle reached 5 m in ~7.8 s.
- Hold error after convergence: ≤0.03 m (target tolerance 0.3 m).
- Landing: ~10.9 s from `NAV_LAND` to disarm.
- Command acknowledgements: all accepted (result 0).

### Pending / next

- Week 3: mission manager state machine, waypoint missions, validation and structured logging.
- Week 6: scripted fault injection for setpoint-stream loss, invalid pose and external mode changes.
- Add an automated offboard integration test once the mission manager can drive missions.

---

## 2026-11-09 — Week 3: mission manager and two-waypoint inspection mission

### Goal

Add a mission layer that owns inspection goals and state, while `aeris_flight_control` remains the only component that commands PX4.

### Implemented

- `aeris_interfaces`:
  - `msg/FlightStatus`
  - `msg/MissionStatus`
  - `srv/GoTo` (NED target, tolerance, timeout, goal sequence)
- `aeris_flight_control` extensions:
  - `TRANSIT` state and `~/go_to` service;
  - `~/land` service;
  - `FlightStatus` publisher at 10 Hz;
  - goal sequence numbers to eliminate stale `goal_reached` races;
  - `auto_land_after_hold` parameter (true for the manual demo, false for missions).
- `aeris_mission_manager` Python package:
  - mission YAML parser and validator;
  - mission state machine (`PREFLIGHT → TAKEOFF → TRANSIT → INSPECT → … → LAND → COMPLETE`);
  - sequential waypoint execution, RTL, landing and abort;
  - JSONL event log per run;
  - `MissionStatus` publisher;
  - `~/start` and `~/abort` services.
- `aeris_bringup`:
  - `mission.launch.py` (telemetry + offboard + mission manager);
  - `offboard_mission.yaml`;
  - `config/missions/inspection_alpha.yaml`.

### Mission result — `inspection_alpha`

Artifacts:

- `docs/experiments/week3_mission.jsonl`
- `docs/experiments/week3_mission_run.log`

| Metric | Result |
|---|---|
| Waypoints | 2 (`wp_01`, `wp_02`) |
| Takeoff to HOLD | 10.3 s after mission start |
| `wp_01` transit | 2.08 s |
| `wp_02` transit | 2.03 s |
| Inspection holds | 3 s each |
| RTL transit | 2.18 s |
| Landing to complete | 11.2 s |
| Total mission | ~33.8 s |
| Final state | `COMPLETE`, `success=true`, 2/2 waypoints |
| Vehicle | disarmed, `AUTO_LOITER`, preflight checks passing |

The JSONL log records `mission_loaded`, `mission_start`, every state transition, every waypoint transit/reach/inspection, RTL and `mission_complete`.

### Tests

```text
colcon test --packages-select aeris_mission_manager
11 passed, 0 errors, 0 failures
```

Validation covers duplicate IDs, frame, altitude, geofence, actions, hold times and missing files.

### Issues and fixes

1. **Stale `goal_reached` latch.** The mission manager could mistake the previous waypoint's `goal_reached` for the new goal.
   - Fix: add a monotonically increasing `goal_sequence` to `GoTo` responses and `FlightStatus`; the mission manager waits for the exact sequence.

2. **Offboard controller auto-landed after HOLD.** The Week 2 demo behavior was wrong for multi-waypoint missions.
   - Fix: `auto_land_after_hold` parameter; `offboard_mission.yaml` disables it and the mission manager owns the landing decision.

3. **Mission manager service calls from a thread.** Service futures are completed by the main rclpy executor while the mission thread waits.
   - The mission thread never calls `spin_until_future_complete`; it polls futures while the main thread spins.

### Next

Week 4: camera integration, OpenCV target detection and target localization. This requires the Gazebo–ROS image bridge (`ros-jazzy-ros-gz`), which is not installed yet.

---

## 2026-11-09 — Week 4: camera, OpenCV detection and mission-log integration

### Goal

Integrate the simulated downward camera, detect the inspection target with OpenCV, publish detections, and record them during mission INSPECT phases.

### Implemented

- `aeris_interfaces`: `Detection` and `DetectionArray`.
- `aeris_perception` Python package:
  - `detector.py`: HSV red thresholding, contour filtering, bearing, slant range and nadir ground projection;
  - `node.py`: image/camera-info/position subscriptions and detection publishing;
  - 4 synthetic-image unit tests.
- `aeris_bringup`:
  - `config/gz_camera_bridge.yaml`;
  - `launch/perception.launch.py`;
  - `ros_gz_bridge` and `aeris_perception` dependencies.
- Simulation:
  - `simulation/models/red_target/model.sdf`;
  - `scripts/spawn_targets.sh`;
  - `launch_simulation.sh` supports `PX4_SIM_MODEL` and `PX4_GZ_WORLD`.
- Mission manager:
  - subscribes to `/aeris/perception/detections`;
  - logs `target_detected` / `target_not_detected` during INSPECT.

### Camera and bridge

- Model: `gz_x500_mono_cam_down`.
- Camera: 1280 × 960, `fx=fy=539.9`, `cx=640`, `cy=480`.
- ROS image rate: ~12.8 Hz.
- Bridge topics: `/aeris/camera/image_raw`, `/aeris/camera/camera_info`.

### Mission result — perception-backed `inspection_alpha`

| Metric | Result |
|---|---|
| Detector tests | 4/4 passing |
| Mission | 2/2 waypoints, RTL, land, `COMPLETE` |
| `target_detected` events | 2/2 inspection waypoints |
| Confidence | 1.00 at both |
| Slant range | 6.27 m / 6.28 m |
| Detection class | `red_target` |

Artifacts:

- `docs/experiments/week4_mission.jsonl`
- `docs/experiments/week4_perception.log`
- `docs/experiments/week4_mission_run.log`
- `docs/perception.md`

### Issues and fixes

1. **NumPy 2 in user site broke apt OpenCV.**
   - `~/.local` NumPy 2.5.4 shadowed system NumPy 1.26 and `cv2`/`cv_bridge` failed to import.
   - Fix: `scripts/aeris_env.sh` exports `PYTHONNOUSERSITE=1`, selecting the apt OpenCV 4.6 + NumPy 1.26 stack.

2. **Camera-info callback logged at 30 Hz.**
   - Fix: log camera intrinsics only on the first valid message.

3. **The downward camera can see the neighbouring target.**
   - The red panel directly under the vehicle is partly occluded by the airframe; the other panel appears at the edge of the frame.
   - Detection is robust, but ground localization remains approximate. Documented in `docs/perception.md`.

### Next

Week 5: depth/lidar perception, obstacle detection, local planning and replanning. The Week 4 camera provides the inspection view; Week 5 adds obstacle-aware navigation.

---

## 2026-11-09 — Week 5: lidar obstacle detection and reactive replanning

### Goal

Detect an obstacle on the inspection path and replan around it without leaving the offboard flight architecture.

### Implemented

- `aeris_interfaces`: `Obstacle` and `ObstacleArray`.
- `aeris_navigation` C++ package:
  - forward-cone minimum-range analysis;
  - 10° sector obstacle map across the 270° scan;
  - `ObstacleArray` publisher;
  - 4 gtest cases for scan analysis.
- `aeris_bringup`:
  - `config/gz_lidar_bridge.yaml`;
  - `launch/navigation.launch.py`.
- Simulation:
  - `simulation/models/tall_obstacle` (1 × 1 × 6 m);
  - `scripts/spawn_obstacle.sh`.
- Mission manager:
  - subscribes to `/aeris/navigation/obstacles`;
  - checks the corridor to the target;
  - logs `obstacle_detected`, computes a lateral midpoint detour and logs `replan`;
  - sends the detour goal, then the original waypoint.

### Result — obstacle mission

| Metric | Result |
|---|---|
| Lidar | 270°, 1080 samples, ~30 Hz |
| Forward obstacle | 3.75 m, bearing −0.204 rad |
| Replan detour | `(1.54, 1.94, −5.0)` |
| Original wp_02 | reached after detour |
| Mission | `COMPLETE` |
| Navigation tests | 4/4 passing |

Evidence:

- `docs/experiments/week5_mission.jsonl`
- `docs/experiments/week5_navigation.log`
- `docs/experiments/week5_mission_run.log`
- `docs/navigation.md`

### Issues and decisions

1. **The target direction was not aligned with the lidar's forward cone.**
   - The lidar is 270° wide; the target can be 90° off the vehicle heading.
   - Fix: publish sector obstacles across the whole scan and have the mission manager evaluate the corridor to the target, not just the forward cone.

2. **The obstacle sat just outside the initial 3 m threshold.**
   - The corridor range at the check was 3.75 m.
   - Fix: use a 4 m path-clearance threshold. The replan then triggered deterministically.

3. **The simulated battery drains across long sessions.**
   - A stale sim reported `battery_unhealthy` and blocked preflight.
   - Fix: restart the SITL session for a clean battery before a test campaign.

4. **PX4 local origin vs Gazebo world origin is not exact.**
   - Spawned models are offset slightly from the nominal local-frame path.
   - The lidar-relative corridor check is robust to this for the demo; precise map alignment is a future refinement.

### Next

Week 6: safety supervision, fault injection, failsafe/return-to-launch behaviour and automated tests.

---

## 2026-11-09 — Week 6: safety supervisor, fault injection and failsafes

### Goal

Add an independent safety layer that detects unhealthy flight conditions, injects faults on demand, and commands a deterministic safe action through the existing flight-control abstraction.

### Implemented

- `aeris_interfaces`:
  - `SafetyStatus.msg`;
  - `InjectFault.srv`.
- `aeris_flight_control`:
  - `~/rtl` service using `VEHICLE_CMD_NAV_RETURN_TO_LAUNCH`;
  - `~/hold` service;
  - `RTL` state that stops the offboard stream, waits for landing/disarm and restores `AUTO_LOITER`;
  - PX4 failsafe is not treated as a local abort while RTL is active.
- `aeris_safety` C++ package:
  - pure evaluator in `include/aeris_safety/safety_evaluator.hpp`;
  - 10 Hz `SafetyStatus` publisher;
  - pose validity/freshness, battery, geofence and mission-heartbeat monitors;
  - injectable `stale_pose`, `low_battery`, `geofence_breach`, `mission_heartbeat_loss`;
  - safe actions via `land`, `rtl` and `hold` services;
  - startup telemetry gate and armed-only action dispatch.
- `aeris_mission_manager`:
  - subscribes to `SafetyStatus`;
  - aborts sequencing with `safety_abort` when armed and unsafe;
  - does not override the supervisor's RTL with a local abort/land;
  - aborts if the safety supervisor heartbeat disappears.
- `aeris_bringup`:
  - `config/safety.yaml`;
  - `launch/safety.launch.py`;
  - `config/missions/safety_test.yaml`.
- `scripts/run_safety_fault_test.sh`: one-command fault scenario runner.
- `scripts/launch_simulation.sh`: keeps the SITL battery healthy by default with `PX4_PARAM_SIM_BAT_MIN_PCT=100` and `PX4_PARAM_SIM_BAT_DRAIN=86400`.

### Results

| Scenario | Fault | Action | Outcome |
|---|---|---|---|
| Nominal | none | none | mission `COMPLETE`, safety `safe=true` throughout |
| Stale pose | `stale_pose` | `land` | `LAND → DONE`, disarmed |
| Low battery | `low_battery` | `rtl` | `Returning to launch` → landed → disarmed |
| Geofence breach | `geofence_breach` | `rtl` | `Returning to launch` → landed → disarmed |
| Mission heartbeat loss | `mission_heartbeat_loss` | `rtl` | `Returning to launch` → landed → disarmed |

All fault runs recorded `safety_abort` in the mission JSONL and the offboard controller reached `DONE`.

Tests:

```text
aeris_safety:           11/11
aeris_navigation:        4/4
aeris_mission_manager:  11/11
aeris_perception:        4/4
Total:                  32 tests, 0 failures
```

Environment check: `18 ok, 0 warnings, 0 failures`.

Evidence:

- `docs/experiments/week6_nominal_mission.jsonl`
- `docs/experiments/week6_stale_pose_mission.jsonl`
- `docs/experiments/week6_low_battery_mission.jsonl`
- `docs/experiments/week6_geofence_breach_mission.jsonl`
- `docs/experiments/week6_mission_heartbeat_loss_mission.jsonl`
- `docs/experiments/week6_*_run.log`
- `docs/experiments/week6_*_safety.log`
- `docs/safety.md`

### Issues and fixes

1. **Safety status was unsafe before DDS discovery completed.**
   - The supervisor started before PX4 position topics were matched and reported `stale_pose`; the mission manager aborted at takeoff.
   - Fix: gate evaluation on pose and vehicle-status telemetry readiness, and require `SafetyStatus.armed` before the mission manager treats a fault as an abort.

2. **Stale mission-manager processes survived a failed test.**
   - Killing only the `ros2 launch` front-end left the Python node alive; a second mission manager caused goal-sequence mismatches.
   - Fix: `run_safety_fault_test.sh` starts each launch in its own process group and signals the whole group on cleanup.

3. **PX4's simulated battery failsafe interrupted long campaigns.**
   - `SIM_BAT_DRAIN` defaults to 60 s from 100% to `SIM_BAT_MIN_PCT` (50%), so PX4 triggered its own RTL.
   - Fix: keep the SITL battery healthy by default; AERIS battery faults are injected in ROS, independently of PX4's power model.

4. **RTL and the offboard controller's failsafe handling could conflict.**
   - The controller would abort on PX4 failsafe while already returning to launch.
   - Fix: add an explicit `RTL` state, stop the offboard stream once PX4 leaves offboard mode, and skip the local failsafe abort while RTL is active.

### Next

Week 7: evaluation harness, benchmark metrics, plots/report and demo packaging.

---

## 2026-11-09 — Week 7: evaluation, benchmarks and demo packaging

### Goal

Turn the Week 3–6 flight evidence into reproducible metrics, plots and a final
report, and package the demo so another engineer can reproduce it.

### Implemented

- `scripts/evaluate_mission.py`:
  - parses mission JSONL files or run directories;
  - computes duration, takeoff, transit, hold, RTL and landing times;
  - counts detections, detection rate, confidence and range;
  - counts obstacle detections, replans and detour coordinates;
  - records safety faults/actions and state transitions;
  - writes JSON, CSV and matplotlib plots.
- `tests/test_evaluate_mission.py`: 4 evaluator tests.
- `docs/evaluation.md`: methodology, scenario tables, aggregate metrics,
  plots, limitations and conclusion.
- `docs/demo.md`: terminal-by-terminal demo runbook and video storyboard.
- Checked-in benchmark evidence:
  - `docs/experiments/benchmark.json`
  - `docs/experiments/benchmark.csv`
  - `docs/experiments/plots/mission_durations.png`
  - `docs/experiments/plots/phase_breakdown.png`

### Results

| Metric | Result |
|---|---|
| Recorded runs | 8 |
| Completed missions | 4 |
| Planned safety aborts | 4 |
| Planned-completion success | 4/4 |
| Safe fault outcomes | 4/4 |
| Completed duration | mean 40.3 s, range 33.8–55.1 s |
| Detections | 2, confidence 1.00, rate 100% |
| Obstacle detections / replans | 1 / 1 |
| Safety dispatch latency | 41–98 ms |
| ROS tests | 32/32 |
| Evaluator tests | 4/4 |
| Environment check | 18 ok |

### Notes

- The benchmark set uses the authoritative Week 3–6 logs rather than
  re-flying every scenario. This is documented as a validity limitation.
- Safety-run durations in the JSONL end at `safety_abort`, not at landing; the
  safety report uses the offboard/PX4 logs for the post-action outcome.
- Demo video capture remains an operator step; the storyboard and expected
  outputs are checked in.

### Next

Project complete. Future work is listed in `docs/checkpoint.md` §8.

---

## Entry template

### YYYY-MM-DD — Title

**Goal:**

**Environment / versions:**

**What was implemented:**

**What worked:**

**What failed:**

**Debugging decisions:**

**Measurements:**

**Next steps:**
