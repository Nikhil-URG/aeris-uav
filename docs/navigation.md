# Navigation and Obstacle Avoidance

This document describes the Week 5 lidar, obstacle-detection and reactive replanning stack.

## Lidar model

AERIS uses the PX4 Gazebo model `gz_x500_lidar_2d`:

- 2D `gpu_lidar` named `lidar_2d_v2`;
- 270° field of view (`-2.356` to `2.356` rad);
- 1080 samples;
- scan rate ~30 Hz;
- mounted above the X500 frame.

Start the lidar simulation:

```bash
PX4_SIM_MODEL=gz_x500_lidar_2d bash scripts/launch_simulation.sh
```

Gazebo topic:

```text
/world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan
```

Bridge configuration: `ros2_ws/src/aeris_bringup/config/gz_lidar_bridge.yaml`

ROS topic:

```text
/aeris/lidar/scan    sensor_msgs/msg/LaserScan
```

## Obstacle model

```text
simulation/models/tall_obstacle/model.sdf
```

A static 1 × 1 × 6 m column. Spawn it on the wp_01 → wp_02 path:

```bash
bash scripts/spawn_obstacle.sh
```

Spawn position: `(4.0, 2.0, 3.0)` in the Gazebo world.

## Navigation node

Package: `aeris_navigation`  
Executable: `navigation_node`  
Launch: `ros2 launch aeris_bringup navigation.launch.py`

Inputs:

- `/aeris/lidar/scan` (`sensor_msgs/msg/LaserScan`)

Output:

- `/aeris/navigation/obstacles` (`aeris_interfaces/msg/ObstacleArray`)

The node:

1. analyzes a forward cone for a quick `blocked`/minimum-range status;
2. divides the full 270° scan into 10° sectors and publishes the minimum range per sector;
3. includes bearing/range for every sector obstacle within 10 m.

`ObstacleArray` fields:

| Field | Meaning |
|---|---|
| `blocked` | obstacle inside the forward 70° cone below 4 m |
| `forward_min_range_m` | closest forward range |
| `forward_min_bearing_rad` | bearing of that range |
| `obstacles[]` | sector obstacles with bearing/range |

## Mission-manager replanning

Before sending each waypoint goal, the mission manager:

1. reads the latest `ObstacleArray`;
2. computes the bearing from the current vehicle position to the waypoint;
3. checks every sector obstacle within ±30° of that bearing;
4. if the closest is under 4 m, logs `obstacle_detected`;
5. computes a lateral detour at the path midpoint, 2.5 m opposite the obstacle;
6. logs `replan`, sends the detour goal, waits for it, then sends the original waypoint.

The detour is a reactive local plan, not a global planner. It is intentionally simple and testable.

## Run

Terminal 1 — lidar simulation:

```bash
PX4_SIM_MODEL=gz_x500_lidar_2d bash scripts/launch_simulation.sh
```

Terminal 2 — obstacle:

```bash
bash scripts/spawn_obstacle.sh
```

Terminal 3 — bridge + navigation:

```bash
source scripts/aeris_env.sh
ros2 launch aeris_bringup navigation.launch.py
```

Terminal 4 — mission:

```bash
source scripts/aeris_env.sh
ros2 launch aeris_bringup mission.launch.py
ros2 service call /aeris_mission_manager/start std_srvs/srv/Trigger
```

## Results — 2026-11-09

| Metric | Result |
|---|---|
| Lidar rate | ~30 Hz |
| Navigation tests | 4/4 passing |
| Obstacle detected | PASS — 3.75 m, bearing −0.20 rad |
| Replan | PASS — detour `(1.54, 1.94, −5.0)` |
| Original waypoint reached | PASS |
| Mission | COMPLETE |
| Detour clearance | PASS — path moved 2.5 m laterally |

JSONL events:

```text
obstacle_detected: range_m=3.75, bearing_rad=-0.204
replan: detour_x=1.54, detour_y=1.94, detour_z=-5.0
waypoint_reached: wp_02
mission_complete
```

Evidence:

- `docs/experiments/week5_mission.jsonl`
- `docs/experiments/week5_navigation.log`
- `docs/experiments/week5_mission_run.log`

## Limitations

- The PX4 local frame origin and the Gazebo world origin are not perfectly aligned, so spawned obstacles are slightly offset from the nominal mission path. The corridor check uses lidar-relative ranges and tolerated the offset with a 4 m threshold.
- Sector obstacles are coarse (10°). A real planner would cluster beams and maintain an occupancy grid.
- The detour is a single lateral midpoint offset. There is no global path search or dynamic re-planning during the detour.
- The lidar is 2D; low obstacles below the scan plane are not seen. Week 4's camera handles target inspection separately.

These simplifications are deliberate for Week 5; Week 6 adds safety/fault handling and Week 7 evaluates the complete stack.
