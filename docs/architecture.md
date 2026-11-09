# AERIS Architecture

## 1. Scope

AERIS is a simulated industrial-inspection UAV. The architecture separates three concerns:

- **Simulation:** Gazebo Harmonic models the X500 airframe, physics, sensors and environment.
- **Flight control:** PX4 runs EKF2, position control, attitude control and actuator output.
- **Mission autonomy:** ROS 2 runs mission state, perception, planning, safety supervision and logging.

The split is deliberate. PX4 is a mature, safety-critical flight stack; the ROS 2 layer is where inspection behaviour evolves quickly. Keeping actuator authority in PX4 means the autonomy layer can fail without directly writing motor commands, and it leaves the door open to a different autopilot later.

## 2. Design principles

1. **PX4 owns stabilization.** No AERIS node publishes actuator commands.
2. **The ROS 2 layer owns mission intent.** It publishes position/trajectory setpoints and high-level vehicle commands.
3. **Interfaces are explicit.** Topics, message types, QoS, frames and units are documented before code depends on them.
4. **Dependencies are pinned.** PX4, `px4_msgs`, the DDS agent and QGC have recorded versions/commits.
5. **Every claim is testable.** Milestones have acceptance criteria and logs.
6. **Safety is a layer, not an afterthought.** Failsafes, watchdogs and return-to-launch are designed alongside autonomy.

## 3. Runtime layers

```text
┌─────────────────────────────────────────────────────────────┐
│ Mission Planning                                            │
│   inspection goals · waypoint missions · state machine       │
├─────────────────────────────────────────────────────────────┤
│ Perception                 Navigation                       │
│   camera · OpenCV          planner · trajectory · avoidance  │
│   target detection                                          │
├─────────────────────────────────────────────────────────────┤
│ Flight Interface                                            │
│   ROS 2 nodes · PX4 messages · uXRCE-DDS                    │
├─────────────────────────────────────────────────────────────┤
│ PX4 Autopilot                                               │
│   EKF2 · position controller · attitude controller · outputs │
├─────────────────────────────────────────────────────────────┤
│ Gazebo Simulation                                           │
│   quadrotor · physics · sensors · inspection environment     │
└─────────────────────────────────────────────────────────────┘

Reliability & Observability spans every layer:
  logging · health monitoring · fault injection · evaluation
```

## 4. Data flow

### Telemetry path (Week 1)

```text
Gazebo sensors
  → PX4 uORB topics
  → PX4 uXRCE-DDS client
  → MicroXRCEAgent (UDP 8888)
  → ROS 2 DDS topics (/fmu/out/...)
  → aeris_telemetry_monitor
```

### Command path (Week 2+)

```text
aeris_flight_control
  → /fmu/in/offboard_control_mode
  → /fmu/in/trajectory_setpoint
  → /fmu/in/vehicle_command
  → PX4 offboard/commander
  → position + attitude controllers
  → actuators
```

The offboard stream is a heartbeat. PX4 must receive setpoints continuously before and during offboard mode; the exact rate and timeout will be verified experimentally in Week 2 and recorded in [flight_control.md](flight_control.md).

## 5. Coordinate frames and units

PX4 uses **NED** (North-East-Down) and **FRD** (Forward-Right-Down). ROS conventionally uses **ENU** (East-North-Up) and **FLU** (Forward-Left-Up). AERIS handles the conversion at the boundary rather than mixing conventions.

For a local position `(x, y, z)` in PX4 NED:

```text
east  =  y
north =  x
up    = -z
```

Therefore a PX4 altitude of 5 m is `z = -5.0` in `VehicleLocalPosition`. The telemetry monitor prints PX4 NED values verbatim and labels them as NED.

Units:

- position: metres
- velocity: metres/second
- heading/yaw: radians
- timestamps: microseconds since boot (PX4), unless noted

## 6. Initial interface contract

| Topic | Type | Direction | QoS | Purpose |
|---|---|---|---|---|
| `/fmu/out/vehicle_local_position_v1` | `px4_msgs/msg/VehicleLocalPosition` | PX4 → ROS 2 | best effort | Local NED position, velocity, validity |
| `/fmu/out/vehicle_status` | `px4_msgs/msg/VehicleStatus` | PX4 → ROS 2 | best effort | Arming, nav state, failsafes |
| `/fmu/out/vehicle_attitude` | `px4_msgs/msg/VehicleAttitude` | PX4 → ROS 2 | best effort | Attitude quaternion |
| `/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` | ROS 2 → PX4 | best effort | Offboard heartbeat/control selection |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/msg/TrajectorySetpoint` | ROS 2 → PX4 | best effort | Position/velocity setpoints (NED) |
| `/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` | ROS 2 → PX4 | best effort | Arm, mode, takeoff, land, RTL |

PX4 v1.17 appends a `_v1` suffix to versioned DDS messages (for example `vehicle_local_position_v1` and `vehicle_status_v1`). The exact topic set is firmware-configuration dependent; it is verified with `ros2 topic list` during Milestone 1 and must not be assumed from older examples.

## 7. Package responsibilities

| Package | Language | Status | Responsibility |
|---|---|---|---|
| `aeris_interfaces` | ROS 2 msgs/srvs | Planned (Week 3) | AERIS-specific mission messages |
| `aeris_bringup` | Python | Active | Launch files and configuration |
| `aeris_flight_control` | C++ | Active | PX4 communication, offboard control |
| `aeris_mission_manager` | Python | Planned (Week 3) | Mission state machine |
| `aeris_navigation` | C++ | Planned (Week 5) | Trajectory and obstacle planning |
| `aeris_perception` | Python | Planned (Week 4) | Image processing and detection |
| `aeris_safety` | C++ | Planned (Week 6) | Health checks and safety supervision |

Only `aeris_flight_control` and `aeris_bringup` are built in Week 1. The remaining packages are created when their milestone begins, so the build stays small and every directory has a purpose.

## 8. Safety architecture

Layers, from lowest to highest latency:

1. **PX4 failsafes:** data-link loss, RC loss, battery, geofence, offboard-loss behaviour.
2. **Offboard watchdog:** `aeris_flight_control` stops streaming if its own health checks fail; PX4 then executes its configured offboard-loss action.
3. **Mission supervisor:** `aeris_safety` (Week 6) monitors pose validity, estimator health, battery, mission timeout and geofence, and can command RTL or land.
4. **Operator:** QGC remains the manual override and monitoring surface.

Safety rules:

- Never enter offboard mode without a valid local position and a live setpoint stream.
- Never arm without a completed preflight check.
- Treat stale telemetry as invalid, not as zero.
- Record every mode change, arming event and failsafe in the mission log.

## 9. Reliability and observability

- **Structured logs:** mission events as JSON Lines in `results/logs/`.
- **rosbag2:** record selected topics for every scenario run.
- **Health topics:** validity flags from PX4, node heartbeats, setpoint age.
- **Experiment records:** commands, versions, parameters and metrics in `docs/experiments/`.
- **Fault injection:** Week 6 adds deterministic faults (dropped setpoints, invalid pose, simulated link loss).

## 10. Architectural decisions

### ADR-001: PX4 remains the flight controller

**Decision:** Use PX4 for stabilization and low-level control; do not implement a custom flight stack.

**Rationale:** Mature estimator/controller, realistic architecture, safety features, and a clean boundary between flight control and mission autonomy. It also allows a future autopilot swap behind the same ROS 2 interface.

### ADR-002: uXRCE-DDS instead of MAVLink for autonomy

**Decision:** Use PX4's uXRCE-DDS bridge and `px4_msgs` for the ROS 2 interface.

**Rationale:** Native ROS 2 messages, typed interfaces and lower integration friction than parsing MAVLink. QGC keeps using MAVLink for ground-station functions.

### ADR-003: C++ for flight control, Python for mission logic

**Decision:** `aeris_flight_control` and `aeris_navigation` are C++; `aeris_mission_manager`, `aeris_perception` and `aeris_bringup` are Python.

**Rationale:** Deterministic, latency-sensitive loops stay in C++. Mission logic and perception iterate faster in Python. The boundary is a ROS 2 topic, so language choice is local.

### ADR-004: PX4 is an external, pinned dependency

**Decision:** Keep PX4-Autopilot outside this Git repository and pin `v1.17.0` in scripts and docs.

**Rationale:** Avoid vendoring a large upstream tree, keep the repository reviewable, and make upgrades explicit.

### ADR-005: best-effort QoS for PX4 topics

**Decision:** Use best-effort QoS for `/fmu/...` topics unless a specific interface requires otherwise.

**Rationale:** Matches PX4's uXRCE-DDS publications and avoids incompatible QoS warnings. Reliability is handled at the application layer (watchdogs, validity checks), not by forcing reliable DDS delivery.

## 11. Open questions

- Exact PX4 offboard setpoint rate and timeout under Gazebo load (measure in Week 2).
- Camera and depth-sensor bridge design (`ros_gz_bridge` vs. PX4 camera topics) for Weeks 4–5.
- Whether the inspection world should be a custom Gazebo world or a composition of existing models.
- Mission log schema and evaluation metrics (Week 7).
