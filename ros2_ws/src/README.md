# ROS 2 Workspace

Build with:

```bash
source ../scripts/aeris_env.sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

or, from the repository root:

```bash
bash scripts/build.sh
```

## Active packages (grid extension)

| Package | Language | Purpose |
|---|---|---|
| `aeris_interfaces` | ROS 2 msgs/srvs | Flight/mission/safety/grid status, detections, obstacles, goals, fault injection |
| `aeris_flight_control` | C++ | PX4 telemetry and offboard control |
| `aeris_mission_manager` | Python | Mission validation, sequencing, replanning, safety aborts and JSONL logging |
| `aeris_perception` | Python | Camera bridge consumer, OpenCV detection and localization |
| `aeris_navigation` | C++ | Lidar obstacle detection and corridor analysis |
| `aeris_safety` | C++ | Health monitoring, fault injection and failsafe actions |
| `aeris_grid_navigation` | Python | Occupancy grid, A* shortest path and go-to-goal/return execution |
| `aeris_grid_gui` | Python + PyQt5 | Goal selection and live progress display |
| `aeris_bringup` | Python | Launch files and configuration |

`px4_msgs` is an external Git submodule pinned to PX4 `v1.17.0`. It provides the DDS message definitions that match the simulated firmware.

## Planned packages

All planned flight packages are implemented. Week 7 evaluation tooling lives under `scripts/` and `docs/experiments/`.

Packages are added when their milestone starts. This keeps the build small and ensures every package has an owner and an acceptance test.

## Conventions

- C++ packages use `ament_cmake`; Python packages use `ament_python`.
- Every package declares its dependencies in `package.xml`.
- Parameters belong in `aeris_bringup/config/`, not hard-coded in nodes.
- Launch files belong in `aeris_bringup/launch/`.
- Unit tests live next to their package; integration tests live in `tests/integration/`.
