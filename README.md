# AERIS — Autonomous UAV Inspection & Reliability System

**Repository:** <https://github.com/Nikhil-URG/aeris-uav>

AERIS is a modular UAV autonomy platform built on **PX4** and **ROS 2 Jazzy**. It is developed as a reproducible robotics software product: pinned dependencies, documented interfaces, scripted setup, simulation scenarios, automated checks, and measured results.

**Current status:** Week 7 complete — full stack evaluated, benchmarked, documented and packaged for demo. 36 automated tests pass; four safety faults verified in SITL.

## Mission

The simulated vehicle will eventually:

1. Arm and take off.
2. Navigate to inspection locations.
3. Maintain stable position and altitude.
4. Observe inspection targets with its camera.
5. Detect and avoid obstacles.
6. Replan its trajectory when necessary.
7. Return to its launch position.
8. Land safely.
9. Report mission results and failures.

PX4 remains responsible for low-level flight stabilization. The ROS 2 application owns mission-level intelligence and sends appropriate setpoints to PX4 over uXRCE-DDS.

## Stack

| Layer | Technology |
|---|---|
| Flight controller | PX4-Autopilot v1.17.0 |
| Autonomy | ROS 2 Jazzy (C++ and Python) |
| PX4 ↔ ROS 2 bridge | uXRCE-DDS + Micro XRCE-DDS Agent v2.4.3 |
| Simulation | Gazebo Harmonic, X500 airframe |
| Ground station | QGroundControl v5.1.5 (optional) |
| OS target | Ubuntu 24.04 / Pop!_OS 24.04 (noble) |

## Architecture

```text
 Gazebo Harmonic / X500
          │ sensors
          ▼
 PX4 Autopilot
   EKF2 → position controller → attitude controller → actuators
          │ uXRCE-DDS (DDS topics)
          ▼
 ROS 2 Jazzy
   ├── aeris_flight_control   (C++)   PX4 interface, offboard control
   ├── aeris_mission_manager  (Py)    mission state machine
   ├── aeris_navigation       (C++)   trajectory and obstacle planning
   ├── aeris_perception       (Py)    camera, OpenCV, target localization
   ├── aeris_safety           (C++)   health checks and failsafes
   └── aeris_bringup          (Py)    launch files and configuration
```

Design rules:

1. PX4 is the only component that commands actuators.
2. ROS 2 sends high-level setpoints and commands, never motor outputs.
3. Every external dependency is version-pinned and documented.
4. Every milestone has a scripted acceptance check.
5. Logs and experiment results are first-class project artifacts.

See [docs/architecture.md](docs/architecture.md) for the full design.

## Repository layout

```text
.
├── README.md
├── LICENSE
├── .gitignore
├── .gitmodules                  # px4_msgs, pinned to v1.17.0
├── claude.md                    # Original project brief
├── docs/
│   ├── architecture.md
│   ├── installation.md
│   ├── flight_control.md
│   ├── mission_design.md
│   ├── perception.md
│   ├── navigation.md
│   ├── safety.md
│   ├── evaluation.md
│   ├── demo.md
│   ├── checkpoint.md
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
│       └── plots/
├── px4/
│   └── README.md                # Upstream PX4 pin and setup notes
├── ros2_ws/
│   └── src/
│       ├── px4_msgs/            # Git submodule (PX4 v1.17.0 messages)
│       ├── aeris_interfaces/    # AERIS messages and services
│       ├── aeris_flight_control/
│       ├── aeris_mission_manager/
│       ├── aeris_perception/
│       ├── aeris_navigation/
│       ├── aeris_safety/
│       └── aeris_bringup/
├── simulation/
│   ├── worlds/
│   ├── models/
│   └── scenarios/
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
│   ├── run_safety_fault_test.sh
│   ├── check_environment.sh
│   └── evaluate_mission.py      # JSONL metrics, CSV and plots
├── tests/
│   ├── test_evaluate_mission.py
│   ├── integration/
│   └── scenarios/
└── results/
    ├── logs/
    ├── plots/
    └── reports/
```

## Version pins

| Component | Pin | Upstream commit |
|---|---|---|
| PX4-Autopilot | `v1.17.0` | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| px4_msgs | `v1.17.0` / `release/1.17` | `86d8239e962f6939e05c3737784f60c02fa884db` |
| Micro XRCE-DDS Agent | `v2.4.3` | `73622810d984349b80bbac0ef55fc0b694d62222` |
| QGroundControl | `v5.1.5` | — |
| ROS 2 | Jazzy | Ubuntu `noble` packages |

PX4-Autopilot lives outside this repository at `~/aeris_dev/PX4-Autopilot` by default. The exact upstream commit is recorded in [px4/README.md](px4/README.md).

## Quick start

Run the one-time setup as your **normal user** (the scripts request `sudo` internally):

```bash
bash scripts/setup.sh
```

Or run the stages individually:

```bash
bash scripts/install_dependencies.sh   # system tools + ROS 2 Jazzy
bash scripts/bootstrap_px4.sh           # PX4 v1.17.0 + Gazebo Harmonic
bash scripts/install_microxrce_agent.sh
bash scripts/build.sh                   # builds px4_msgs + AERIS packages
```

Run the simulation in one terminal:

```bash
bash scripts/launch_simulation.sh
```

Run the telemetry monitor in another:

```bash
source scripts/aeris_env.sh
ros2 launch aeris_bringup telemetry.launch.py
```

Optional ground station:

```bash
bash scripts/install_qgroundcontrol.sh
scripts/run_qgroundcontrol.sh
```

## Offboard flight demo

With the simulation and QGC running:

```bash
source scripts/aeris_env.sh
ros2 launch aeris_bringup offboard.launch.py
```

In another terminal, trigger the mission:

```bash
ros2 service call /aeris_offboard_controller/start std_srvs/srv/Trigger
```

The controller waits for a valid pose and preflight checks, streams offboard setpoints at 20 Hz, requests offboard mode, arms, climbs to 5 m, holds for 10 s, lands and disarms. A controlled abort is available:

```bash
ros2 service call /aeris_offboard_controller/abort std_srvs/srv/Trigger
```

Do not interact with QGC during an automated run unless you intend to exercise the external-land/abort path. Flight evidence is in [docs/experiments/week2_offboard_run.log](docs/experiments/week2_offboard_run.log).

## Full mission demo

The mission launch starts telemetry, the offboard controller and the Python mission manager:

```bash
source scripts/aeris_env.sh
ros2 launch aeris_bringup mission.launch.py
```

In another terminal:

```bash
ros2 service call /aeris_mission_manager/start std_srvs/srv/Trigger
```

The default mission (`inspection_alpha`) takes off, visits two inspection waypoints, holds at each, returns to launch and lands. Each run writes a structured event log to `results/logs/<run_id>/mission.jsonl`.

Mission evidence: [docs/experiments/week3_mission.jsonl](docs/experiments/week3_mission.jsonl).

## Perception demo

Use the camera-enabled airframe:

```bash
PX4_SIM_MODEL=gz_x500_mono_cam_down bash scripts/launch_simulation.sh
```

Spawn the two red inspection targets:

```bash
bash scripts/spawn_targets.sh
```

Start the Gazebo camera bridge and the OpenCV perception node:

```bash
source scripts/aeris_env.sh
ros2 launch aeris_bringup perception.launch.py
```

The perception stack publishes `/aeris/perception/detections`; the mission manager records `target_detected` events during INSPECT. See [docs/perception.md](docs/perception.md) and [docs/experiments/week4_mission.jsonl](docs/experiments/week4_mission.jsonl).

## Obstacle-avoidance demo

Use the lidar airframe:

```bash
PX4_SIM_MODEL=gz_x500_lidar_2d bash scripts/launch_simulation.sh
```

Spawn the tall obstacle on the wp_01 → wp_02 path:

```bash
bash scripts/spawn_obstacle.sh
```

Start the lidar bridge and navigation node:

```bash
source scripts/aeris_env.sh
ros2 launch aeris_bringup navigation.launch.py
```

Run the mission as above. The mission manager logs `obstacle_detected` and `replan`, flies a lateral detour, then continues to the original waypoint. See [docs/navigation.md](docs/navigation.md) and [docs/experiments/week5_mission.jsonl](docs/experiments/week5_mission.jsonl).

## Safety and fault injection

Start the mission stack and the safety supervisor:

```bash
source scripts/aeris_env.sh
MISSION_FILE=$(ros2 pkg prefix aeris_bringup)/share/aeris_bringup/config/missions/safety_test.yaml
ros2 launch aeris_bringup mission.launch.py mission_file:="$MISSION_FILE"
ros2 launch aeris_bringup safety.launch.py
```

Start the mission, then inject a fault during the inspection hold:

```bash
ros2 service call /aeris_mission_manager/start std_srvs/srv/Trigger
ros2 service call /aeris_safety/inject_fault aeris_interfaces/srv/InjectFault \
  "{fault: low_battery, enable: true}"
```

The supervisor publishes `/aeris/safety/status` and commands `land` for stale pose or `rtl` for low battery, geofence breach and mission-heartbeat loss. The mission manager records `safety_abort` and stops sequencing. A one-command scenario runner is available:

```bash
bash scripts/run_safety_fault_test.sh stale_pose
```

See [docs/safety.md](docs/safety.md) and the `docs/experiments/week6_*` evidence.

## Evaluation

The evaluator turns mission JSONL logs into benchmark metrics, a CSV and plots:

```bash
source scripts/aeris_env.sh
python3 scripts/evaluate_mission.py \
  --json docs/experiments/benchmark.json \
  --csv docs/experiments/benchmark.csv \
  --plot-dir docs/experiments/plots
python3 -m pytest tests/test_evaluate_mission.py -q
```

Recorded benchmark results:

- 8 authoritative run logs (nominal, perception, obstacle, safety nominal,
  four fault scenarios);
- 4/4 completed missions and 4/4 safe fault outcomes;
- 100% detection rate at both inspection waypoints, mean confidence 1.00;
- obstacle detected at 3.75 m, one lateral replan, original waypoint reached;
- safety actions dispatched 41–98 ms after fault injection;
- 32 ROS tests + 4 evaluator tests, 0 failures.

Read the full report in [docs/evaluation.md](docs/evaluation.md), the plots in
`docs/experiments/plots/`, and the reproducible demo runbook in
[docs/demo.md](docs/demo.md).

Full instructions, verification steps and troubleshooting: [docs/installation.md](docs/installation.md).

## Resuming the project

Read [docs/checkpoint.md](docs/checkpoint.md) for the current state, verified results, environment details, and a copy-paste resume prompt. It is written so a new session — on any machine — can continue without reconstructing context from chat history.

## Development workflow

```bash
source scripts/aeris_env.sh   # keeps conda out of the ROS/PX4 shell
bash scripts/build.sh
bash scripts/check_environment.sh
```

The environment script is required because this machine has Miniforge first in `PATH`. ROS 2 Jazzy and PX4 SITL must use the system Python 3.12.

## Roadmap

| Week | Deliverable | Status |
|---|---|---|
| 1 | PX4 SITL + Gazebo + QGC + uXRCE-DDS + telemetry monitor | Complete |
| 2 | C++ offboard controller: readiness, arm, takeoff, hold, land | Complete |
| 3 | Mission manager: state machine, waypoints, validation, logging | Complete |
| 4 | Perception: camera, OpenCV, target detection and localization | Complete |
| 5 | Navigation: depth camera, obstacle detection, local replanning | Complete |
| 6 | Safety: fault injection, failsafes, RTL, automated tests | Complete |
| 7 | Evaluation: benchmarks, experiment reports, demo | Complete |

## License

Apache-2.0. See [LICENSE](LICENSE).
