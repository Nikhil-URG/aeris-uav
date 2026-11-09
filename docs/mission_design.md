# Mission Design

This document defines the mission layer that will be implemented in Week 3. It is written now so the flight-control and safety interfaces have a concrete target.

## 1. Mission model

A mission is a bounded sequence of inspection goals with explicit safety and termination rules.

```yaml
mission:
  name: inspection_alpha
  frame: local_ned
  home:
    x: 0.0
    y: 0.0
    z: 0.0
  defaults:
    cruise_altitude_m: 5.0
    max_speed_mps: 1.5
    waypoint_tolerance_m: 0.4
    mission_timeout_s: 180
  waypoints:
    - id: wp_01
      x: 6.0
      y: 0.0
      z: -5.0
      hold_s: 3.0
      action: inspect
      target_id: valve_a
    - id: wp_02
      x: 6.0
      y: 4.0
      z: -5.0
      hold_s: 3.0
      action: inspect
      target_id: gauge_b
  return_to_launch: true
  land_on_complete: true
```

All positions are in PX4 local NED coordinates. A future ENU mission format can be added, but the internal contract stays explicit about frames.

## 2. State machine

```text
IDLE
  → PREFLIGHT
      → TAKEOFF
          → TRANSIT ──→ INSPECT ──→ TRANSIT
              │            │
              │            └──→ REPLAN (Week 5)
              │
              ├──→ RTL ──→ LAND ──→ COMPLETE
              └──→ ABORT ──→ LAND ──→ COMPLETE
```

| State | Entry condition | Exit condition | Owner |
|---|---|---|---|
| `IDLE` | Node start | Mission loaded and validated | Mission manager |
| `PREFLIGHT` | Mission accepted | Vehicle ready, pose valid, home set | Mission manager + safety |
| `TAKEOFF` | Preflight passed | Target altitude reached within tolerance | Flight control |
| `TRANSIT` | Waypoint commanded | Waypoint reached or replan required | Flight control + navigation |
| `INSPECT` | At inspection waypoint | Hold time elapsed and observation captured | Mission manager + perception |
| `REPLAN` | Obstacle or invalid path | New path accepted | Navigation |
| `RTL` | Mission complete, timeout, or fault | Home reached and landed | Flight control + safety |
| `LAND` | Landing commanded | Touchdown confirmed | Flight control |
| `COMPLETE` | Landed and disarmed | — | Mission manager |
| `ABORT` | Unrecoverable or operator abort | Landed and disarmed | Safety |

## 3. Validation rules

A mission is rejected before flight if any rule fails:

- frame is `local_ned`;
- at least one waypoint and no duplicate IDs;
- all waypoints inside the configured geofence;
- altitude within `[min_altitude_m, max_altitude_m]` (converted to NED `z`);
- horizontal leg length and speed limits respected;
- hold time within bounds;
- mission timeout positive;
- home position available;
- vehicle model and firmware version match the supported configuration.

Validation errors are reported as structured mission events, not just console text.

## 4. Preflight criteria

- PX4 connected and `vehicle_status` received;
- local position valid (`xy_valid`, `z_valid`);
- velocity valid;
- estimator not dead-reckoning;
- battery above threshold;
- home position set;
- no active failsafe;
- GCS link healthy;
- mission validated.

## 5. Logging and result reporting

Every run produces:

- `results/logs/<run_id>/mission.jsonl` — state transitions, waypoints, commands, faults;
- `results/logs/<run_id>/telemetry.csv` — sampled pose, velocity, validity, battery;
- `results/logs/<run_id>/rosbag2/` — selected topics;
- `results/reports/<run_id>.md` — human-readable summary;
- optional plots in `results/plots/`.

The evaluation script (`scripts/evaluate_mission.py`) will consume these artifacts in Week 7 and compute:

- mission completion (success/failure);
- time to each waypoint;
- position-hold error;
- path length vs. straight-line distance;
- maximum speed and acceleration;
- number and duration of failsafes;
- recovery outcome after injected faults.

## 6. Failure handling

| Failure | Detection | Response |
|---|---|---|
| Pose invalid | `xy_valid`/`z_valid` false | Hold or abort; never continue transit |
| Telemetry stale | timestamp/watchdog | Stop offboard stream; PX4 failsafe |
| Waypoint unreachable | navigation planner | Replan once, then abort |
| Obstacle too close | depth/range check | Stop, replan, or RTL |
| Mission timeout | wall-clock/ROS time | RTL |
| Battery low | `vehicle_status`/battery topic | RTL (ROS safety supervisor) |
| GCS loss | heartbeat age | Continue mission or RTL per policy |
| Operator abort | service/command | Land at current position or RTL |
| Supervisor fault | `SafetyStatus.safe == false` while armed | Stop sequencing; supervisor commands land or RTL |

Week 6 implements the safety supervisor policies. The mission manager records `safety_abort` and deliberately does not call its own abort service, so an RTL decision is not overridden by a landing command. See [safety.md](safety.md).

## 7. Week 3 deliverable

- `aeris_mission_manager` Python package;
- mission YAML parser and validator;
- state machine with explicit transitions and event logging;
- integration with `aeris_flight_control` for takeoff/transit/land;
- at least one synthetic inspection route in `simulation/scenarios/`;
- unit tests for validation and state transitions.
