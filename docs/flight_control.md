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

## 3. Milestone 2 — offboard flight (Week 2)

The next executable will be an offboard controller. The intended sequence is:

1. **Preflight**
   - Wait for `/fmu/out/vehicle_status` and a valid `VehicleLocalPosition`.
   - Verify estimator validity, battery threshold and home position.
   - Refuse to arm if any check fails.
2. **Offboard heartbeat**
   - Publish `/fmu/in/offboard_control_mode` with the position-control bit set.
   - Publish a valid `/fmu/in/trajectory_setpoint` at a steady rate (target 20 Hz). PX4's offboard-loss timeout is short, so never rely on the minimum; verify the configured `COM_OF_LOSS_T` during the Week 2 tests.
   - Stream for at least one second before requesting offboard mode.
3. **Mode and arming**
   - Send `VEHICLE_CMD_DO_SET_MODE` with `PX4_CUSTOM_MAIN_MODE_OFFBOARD`.
   - Send `VEHICLE_CMD_COMPONENT_ARM_DISARM` to arm only after offboard mode is confirmed.
4. **Takeoff**
   - Command a position setpoint at the target altitude (e.g. `z = -5.0`) with a bounded vertical ramp.
   - Hold the setpoint and verify position convergence.
5. **Landing**
   - Command `VEHICLE_CMD_NAV_LAND` or ramp the setpoint down, depending on the test.
   - Disarm only after touchdown is confirmed.

The exact PX4 state transitions and timing will be measured, not assumed. Results go into this document and the development log.

### Offboard safety interlock

The controller must stop publishing setpoints if any of the following becomes false:

- local position valid;
- local velocity valid;
- setpoint age below the configured limit;
- PX4 still in offboard mode;
- mission supervisor heartbeat present (from Week 6 onward).

When the stream stops, PX4 executes its configured offboard-loss action. That action must be verified in simulation before autonomous missions are attempted.

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
