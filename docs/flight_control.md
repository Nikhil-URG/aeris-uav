# Flight Control Interface

`aeris_flight_control` is the boundary between AERIS mission logic and PX4. It is deliberately narrow: it translates mission intent into PX4 offboard interfaces and translates PX4 telemetry into a stable internal contract. It never commands motors directly.

## 1. Milestone 1 — read-only telemetry

The first executable is `aeris_telemetry_monitor`. It subscribes to PX4's local position estimate and reports:

- position in PX4 NED coordinates;
- velocity in PX4 NED coordinates;
- speed magnitude;
- position and velocity validity flags;
- whether the vehicle is moving above a configurable threshold.

### Node

| Item | Value |
|---|---|
| Node name | `aeris_telemetry_monitor` |
| Executable | `telemetry_monitor` |
| Package | `aeris_flight_control` |
| Topic | `/fmu/out/vehicle_local_position_v1` |
| Message | `px4_msgs/msg/VehicleLocalPosition` |
| QoS | best effort, keep last 10 |

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `position_topic` | `/fmu/out/vehicle_local_position_v1` | PX4 local position topic |
| `log_every_n` | `50` | Print one line per N messages |
| `moving_speed_threshold` | `0.1` | Speed threshold in m/s for `Moving=YES` |

The defaults live in `ros2_ws/src/aeris_bringup/config/telemetry.yaml`; the launch file passes them to the node.

> **PX4 v1.17 topic suffix:** versioned DDS messages carry a `_v1` suffix. The local position publisher is `/fmu/out/vehicle_local_position_v1`, not the unsuffixed name used in early examples. The unsuffixed name can still appear in the ROS graph if a subscriber creates it, but it has no publisher. Always confirm the active name with `ros2 topic list` after starting SITL.

### Run

```bash
source scripts/aeris_env.sh
ros2 launch aeris_bringup telemetry.launch.py
```

or directly:

```bash
ros2 run aeris_flight_control telemetry_monitor --ros-args -p log_every_n:=50
```

### Expected output

```text
[aeris_telemetry_monitor]: AERIS telemetry monitor initialized (topic='/fmu/out/vehicle_local_position_v1', log_every_n=50).
[aeris_telemetry_monitor]: Position [NED] X=0.02 Y=-0.01 Z=-0.04 | Velocity [NED] VX=0.00 VY=0.01 VZ=0.00 | Speed=0.01 m/s | PosValid=YES VelValid=YES | Moving=NO
```

Values are illustrative; simulator noise is expected.

## 2. PX4 message semantics

`VehicleLocalPosition` is expressed in the local NED frame:

| Field | Meaning | Unit |
|---|---|---|
| `x`, `y`, `z` | local position north, east, down | m |
| `vx`, `vy`, `vz` | local velocity north, east, down | m/s |
| `xy_valid`, `z_valid` | position estimate validity | bool |
| `v_xy_valid`, `v_z_valid` | velocity estimate validity | bool |
| `heading` | yaw estimate | rad |
| `eph`, `epv` | horizontal/vertical position uncertainty | m |
| `dead_reckoning` | estimator is dead-reckoning | bool |
| `timestamp` | PX4 timestamp | µs since boot |

Consequences:

- A vehicle at 5 m altitude has `z = -5.0`, not `+5.0`.
- "Up" is `-z`; use `-msg.z` when producing an ENU altitude.
- Never treat invalid or stale telemetry as zero. Gate mission logic on `xy_valid && z_valid` (position) and `v_xy_valid && v_z_valid` (velocity).

The telemetry monitor prints PX4 values verbatim and labels the frame as NED to avoid silent frame confusion.

## 3. Milestone 2 — offboard controller (Week 2) — implemented and flight-tested

The executable is `offboard_controller` in `aeris_flight_control`.

```bash
ros2 launch aeris_bringup offboard.launch.py
ros2 service call /aeris_offboard_controller/start std_srvs/srv/Trigger
```

Configuration lives in `ros2_ws/src/aeris_bringup/config/offboard.yaml`.

### Services

| Service | Type | Purpose |
|---|---|---|
| `~/start` | `std_srvs/srv/Trigger` | Start after readiness checks pass |
| `~/go_to` | `aeris_interfaces/srv/GoTo` | Fly to an NED target and hold |
| `~/hold` | `std_srvs/srv/Trigger` | Cancel the current goal and hold position |
| `~/land` | `std_srvs/srv/Trigger` | Land at the current position |
| `~/rtl` | `std_srvs/srv/Trigger` | Switch PX4 to AUTO_RTL |
| `~/abort` | `std_srvs/srv/Trigger` | Abort and land at the current position |

### State machine

```text
WAIT_FOR_READY
  -> PRESTREAM            (2 s heartbeat + hold setpoint)
  -> REQUEST_OFFBOARD     (DO_SET_MODE, custom main mode 6)
  -> ARM                  (ARM_DISARM, param1=1)
  -> TAKEOFF              (ramp z from current to -5.0 NED)
  -> HOLD                 (wait for a goal, land, or standalone auto-land)
  -> TRANSIT              (go_to target; new goals update in flight)
  -> HOLD                 (goal reached, or goal timeout)
  -> RTL                  (AUTO_RTL; stop offboard stream when PX4 leaves offboard)
  -> LAND                 (NAV_LAND at current position)
  -> DONE                 (disarmed; restore AUTO_LOITER)

Any fault -> ABORT_LAND   (send NAV_LAND, keep signal briefly)
          -> ABORTED
```

### Interface contract

| Direction | Topic | Purpose |
|---|---|---|
| Subscribe | `/fmu/out/vehicle_local_position_v1` | Pose, velocity, validity, PX4 time |
| Subscribe | `/fmu/out/vehicle_status_v1` | Arming, nav state, failsafe, preflight |
| Subscribe | `/fmu/out/vehicle_control_mode` | Offboard flag confirmation |
| Subscribe | `/fmu/out/vehicle_command_ack` | Command acknowledgement logging |
| Publish | `/fmu/in/offboard_control_mode` | Position-control heartbeat |
| Publish | `/fmu/in/trajectory_setpoint` | Position + yaw setpoint (NED) |
| Publish | `/fmu/in/vehicle_command` | Mode, arm, land commands |
| Publish | `/aeris_offboard_controller/flight_status` | `aeris_interfaces/msg/FlightStatus` at 10 Hz |

### Mission interface (Week 3)

The mission manager never publishes PX4 topics. It calls the offboard controller:

- `GoTo` carries a target in local NED, tolerance and timeout. The controller assigns a monotonically increasing **goal sequence** and reports it in `FlightStatus`, eliminating stale `goal_reached` races.
- `FlightStatus` reports controller state, armed/offboard flags, current and target position, position error and goal status.
- `auto_land_after_hold` is `true` for the standalone Week 2 demo and `false` for `mission.launch.py`; the mission manager owns the landing decision.
- `offboard_mission.yaml` is the mission-mode config.

The Python mission manager loads and validates a YAML mission, calls `start`, sequences `go_to` for each waypoint, waits for each exact goal sequence, holds for the inspection time, performs RTL, calls `land`, and writes a JSONL event log.

### Safety interface (Week 6)

`aeris_safety` monitors PX4 telemetry and the mission heartbeat and commands the controller through the same services:

- stale pose → `~/land`
- low battery, geofence breach, mission-heartbeat loss → `~/rtl`
- optional hold action → `~/hold`

The controller treats `~/rtl` as an explicit `RTL` state: it requests `VEHICLE_CMD_NAV_RETURN_TO_LAUNCH`, stops the offboard stream after PX4 leaves offboard mode, waits for landing/disarm, then restores `AUTO_LOITER`. PX4's own failsafe does not trigger the local abort path while `RTL` is active, so a commanded return is not overridden by a landing command. See [safety.md](safety.md).

### Safety checks

- Position and velocity validity (`xy_valid`, `z_valid`, `v_xy_valid`, `v_z_valid`).
- Status and control-mode freshness (1 s timeout).
- `pre_flight_checks_pass`, disarmed state and no PX4 failsafe before start.
- Timeouts for mode request, arming, takeoff and landing.
- Horizontal distance and altitude limits.
- Position-hold error limit (1 m, sustained for 2 s).
- Abort sends `NAV_LAND` and keeps the offboard signal briefly so PX4 accepts the landing command before the stream stops.
- An external `AUTO_LAND` during HOLD is treated as a landing, not a failure.

### Measured results — 2026-11-09

Clean final run (`docs/experiments/week2_offboard_run.log`):

| Metric | Result |
|---|---|
| `DO_SET_MODE` / `ARM_DISARM` / `NAV_LAND` acks | accepted (result 0) |
| Takeoff to 5 m | ~7.8 s (1.0 m/s target ramp) |
| Hold duration | 10 s |
| Steady-state hold error | 0.01–0.03 m (initial convergence 0.27 m) |
| Landing to disarm | ~10.9 s |
| Final state | disarmed, `AUTO_LOITER`, `pre_flight_checks_pass=true` |
| PX4 events | Takeoff detected, Landing detected, Disarmed by landing |

An abort-path test was also performed: an external QGC land command arrived during HOLD, the controller detected the loss of offboard mode, sent its own `NAV_LAND`, stopped the stream and the vehicle landed and disarmed safely.

Week 3 two-waypoint mission (`docs/experiments/week3_mission.jsonl`):

| Metric | Result |
|---|---|
| Waypoints | 2/2 completed |
| `wp_01` / `wp_02` transit | 2.08 s / 2.03 s |
| Inspection holds | 3 s each |
| RTL transit | 2.18 s |
| Landing to complete | 11.2 s |
| Total mission | ~33.8 s |
| Final state | `COMPLETE`, success, disarmed, `AUTO_LOITER` |

### Operational notes

- Setpoint stream: 20 Hz. PX4's `COM_OF_LOSS_T` default is 1.0 s, so the margin is large.
- While idle, the node publishes a 2 Hz `OffboardControlMode` heartbeat. Without it, PX4's `offboard_control_signal_lost` blocks `pre_flight_checks_pass` after a landing.
- After landing the node requests `AUTO_LOITER`. `POSCTL` requires a manual-control link, which is not present in this GCS-only SITL setup.
- Outgoing message timestamps use PX4 time from `VehicleLocalPosition`, not ROS epoch time.
- The controller never publishes actuator commands.

### Limitations

- No automated offboard integration test yet; fault injection and scripted abort tests are planned for Week 6.
- Mission speed limits are validated but not yet enforced by the controller; PX4's position controller handles the trajectory.
- Perception at inspection waypoints is not integrated yet (Week 4).

## 4. Safety rules

1. Never publish actuator outputs from ROS 2.
2. Never arm before a complete preflight check.
3. Never enter offboard mode without a live setpoint stream.
4. Treat stale or invalid telemetry as a fault, not as zero.
5. Log every command, mode change and arming transition with timestamps.
6. Test every failsafe deliberately (Week 6) before claiming it works.

## 5. Test plan

### Milestone 1

- Build the node and launch it with PX4 SITL running.
- Confirm continuous messages at the expected rate.
- Confirm `PosValid=YES` and `VelValid=YES` while the vehicle is stationary.
- Confirm `Moving=NO` at rest and `Moving=YES` when the vehicle is moved in simulation (or during a later flight).
- Record the first run in `docs/experiments/development_log.md`.

### Milestone 2

- Offboard heartbeat accepted by PX4.
- Arm, take off to 5 m, hold within a documented tolerance, land.
- Abort test: kill the setpoint publisher; verify PX4 failsafe behaviour.
- Abort test: invalid pose; verify the controller does not arm.
- Timing test: measure setpoint age and mode-switch latency.

## 6. Interface ownership

- `aeris_flight_control` owns the `/fmu/in/*` command topics.
- Mission code owns high-level goals, not PX4 commands.
- `aeris_safety` (Week 6) can veto or override mission commands; it does not bypass `aeris_flight_control`.
- `aeris_bringup` owns launch-time parameters and topic remappings.
