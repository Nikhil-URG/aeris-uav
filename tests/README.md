# Tests

- `integration/` — tests that run multiple AERIS nodes together.
- `scenarios/` — end-to-end mission scenarios against PX4 SITL and Gazebo.

Unit tests live next to the package they test:

- C++: `colcon test` with `ament_cmake_gtest` or `ament_cmake_pytest`.
- Python: `pytest` under the package's `test/` directory.

Planned progression:

| Milestone | Test type |
|---|---|
| Week 1 | Telemetry topic/message smoke test, launch test |
| Week 2 | Offboard state-machine tests, arming/abort tests |
| Week 3 | Mission validation and transition unit tests |
| Week 5 | Planner/avoidance unit tests |
| Week 6 | Fault-injection integration tests |
| Week 7 | Scenario benchmark runs with recorded metrics |

Integration tests should be runnable in headless mode (`HEADLESS=1`) so they do not require a desktop session.
