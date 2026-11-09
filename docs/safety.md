# Safety, Fault Injection and Failsafes

Week 6 adds an independent safety supervisor. It observes PX4 telemetry and the mission heartbeat, decides whether the flight is safe, and commands a safe action through the existing offboard-controller services. It never publishes `/fmu/in/*` or bypasses `aeris_flight_control`.

## Architecture

```text
PX4 telemetry ──► aeris_safety ──► /aeris/safety/status ──► mission manager
                        │
                        └── land / rtl / hold service calls ──► aeris_flight_control ──► PX4
```

- `aeris_flight_control` remains the only component that talks to PX4.
- `aeris_safety` is read-only with respect to PX4 and commands only through the flight-control abstraction.
- `aeris_mission_manager` stops sequencing when the supervisor reports an armed unsafe state and records a `safety_abort` event.

## Interfaces

### `aeris_interfaces/msg/SafetyStatus`

Published at 10 Hz on `/aeris/safety/status`.

| Field | Meaning |
|---|---|
| `safe` | No active fault |
| `armed`, `offboard` | Current vehicle flags |
| `action`, `trigger` | Selected action (`none`, `hold`, `rtl`, `land`) and causing fault |
| `pose_valid`, `pose_age_s` | Local-position validity and age |
| `battery_ok`, `battery_remaining` | Battery health and remaining fraction |
| `geofence_ok`, `distance_from_home_m` | Horizontal geofence status |
| `mission_heartbeat_ok`, `mission_heartbeat_age_s` | Mission-manager heartbeat |
| `active_faults` | All active fault names |

### `aeris_interfaces/srv/InjectFault`

Service `/aeris_safety/inject_fault`:

```text
string fault   # stale_pose, low_battery, geofence_breach,
               # mission_heartbeat_loss, all
bool enable
---
bool success
string message
```

`enable: false` clears a fault. The `/aeris_safety/clear_faults` `Trigger` service clears all injected faults.

### Flight-control services

| Service | Purpose |
|---|---|
| `/aeris_offboard_controller/land` | Land at the current position |
| `/aeris_offboard_controller/rtl` | Switch PX4 to AUTO_RTL |
| `/aeris_offboard_controller/hold` | Cancel the current goal and hold position |
| `/aeris_offboard_controller/abort` | Operator abort and land |

## Monitors

| Monitor | Source | Fault condition |
|---|---|---|
| Pose | `/fmu/out/vehicle_local_position_v1` | `xy_valid`/`z_valid` false, or age > `pose_timeout_s` |
| Battery | `/fmu/out/battery_status_v1` | warning critical/emergency/failed/unhealthy, or remaining below `battery_min_remaining` |
| Geofence | local position | horizontal distance from the configured centre > `geofence_radius_m` |
| Mission heartbeat | `/aeris_mission_manager/status` | mission active and status age > `mission_heartbeat_timeout_s` |
| Offboard | `/fmu/out/vehicle_control_mode` | reported in `SafetyStatus`; not an action trigger |

A startup gate suppresses evaluation until pose and vehicle-status telemetry have been received, avoiding a false stale-pose alarm during DDS discovery. Safe actions are only dispatched while the vehicle is armed.

## Fault-to-action mapping

| Fault | Default action | Rationale |
|---|---|---|
| `stale_pose` | `land` | Navigation is unreliable; descend at the current position |
| `low_battery` | `rtl` | Return before the battery becomes critical |
| `geofence_breach` | `rtl` | Return inside the permitted area |
| `mission_heartbeat_loss` | `rtl` | Mission owner is gone; return to launch |

Actions are configurable in `ros2_ws/src/aeris_bringup/config/safety.yaml`. The action service is retried every `action_retry_interval_s` until accepted.

## Run

Terminal 1 — simulation:

```bash
PX4_SIM_MODEL=gz_x500 bash scripts/launch_simulation.sh
```

Terminal 2 — mission stack:

```bash
source scripts/aeris_env.sh
MISSION_FILE=$(ros2 pkg prefix aeris_bringup)/share/aeris_bringup/config/missions/safety_test.yaml
ros2 launch aeris_bringup mission.launch.py mission_file:="$MISSION_FILE"
```

Terminal 3 — safety supervisor:

```bash
source scripts/aeris_env.sh
ros2 launch aeris_bringup safety.launch.py
```

Start the mission, then inject a fault during the inspection hold:

```bash
ros2 service call /aeris_mission_manager/start std_srvs/srv/Trigger
ros2 service call /aeris_safety/inject_fault aeris_interfaces/srv/InjectFault \
  "{fault: low_battery, enable: true}"
```

Or run a complete scenario automatically against a running SITL:

```bash
bash scripts/run_safety_fault_test.sh low_battery
```

The runner starts the mission stack and safety node, waits for the inspection hold, injects the fault, waits for the safety abort and disarm, then stops the stack. It expects QGroundControl to be connected because PX4 requires a GCS for arming in this SITL setup.

## Results — 2026-11-09

All scenarios were flown in SITL with the `safety_test` mission (one waypoint, 30 s hold).

| Scenario | Fault | Action | Flight outcome |
|---|---|---|---|
| Nominal | none | none | `COMPLETE`, no faults |
| Stale pose | `stale_pose` | `land` | `LAND → DONE`, disarmed |
| Low battery | `low_battery` | `rtl` | PX4 `Returning to launch` → landed → disarmed |
| Geofence breach | `geofence_breach` | `rtl` | PX4 `Returning to launch` → landed → disarmed |
| Mission heartbeat loss | `mission_heartbeat_loss` | `rtl` | PX4 `Returning to launch` → landed → disarmed |

Each fault run recorded `safety_abort` in the mission JSONL with the fault list and selected action, and the offboard controller reached `DONE` after landing.

Safety-evaluator tests: **11/11 passing**. The full workspace test suite is **32 tests, 0 failures**.

Evidence:

- `docs/experiments/week6_nominal_mission.jsonl`
- `docs/experiments/week6_stale_pose_mission.jsonl`
- `docs/experiments/week6_low_battery_mission.jsonl`
- `docs/experiments/week6_geofence_breach_mission.jsonl`
- `docs/experiments/week6_mission_heartbeat_loss_mission.jsonl`
- `docs/experiments/week6_*_run.log`
- `docs/experiments/week6_*_safety.log`

## SITL battery note

`scripts/launch_simulation.sh` keeps the simulated battery healthy by default:

```text
PX4_PARAM_SIM_BAT_MIN_PCT=100
PX4_PARAM_SIM_BAT_DRAIN=86400
```

Without this, PX4's simulated battery triggers its own battery failsafe during a long test campaign and interferes with injected ROS faults. Override those variables if you explicitly want to exercise PX4's battery path.

## Limitations

- Faults are injected at the ROS supervision layer, not by corrupting PX4 internals.
- The geofence is a horizontal circle around a configured local-NED centre; it is not a polygon or altitude-aware boundary.
- The supervisor is a single process; production systems should use redundant or hardware-level safeguards.
- The safety status is advisory for the mission manager; a mission manager that ignores it can still fly.
- `mission_heartbeat_loss` depends on the mission manager continuing to publish `MissionStatus`; the current manager publishes at 2 Hz.
