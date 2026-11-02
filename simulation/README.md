# Simulation Assets

- `worlds/` — custom Gazebo worlds for AERIS inspection scenarios.
- `models/` — AERIS-specific models (targets, obstacles, fixtures).
- `scenarios/` — mission/scenario definitions (waypoint sets, fault-injection configs).

Week 1 uses PX4's upstream `default` world via `make px4_sitl gz_x500`. A custom inspection world is introduced when perception and navigation need defined targets and obstacles (Weeks 4–5).

No files are added here until they are used by a milestone; empty directories are tracked with `.gitkeep` so the layout is visible in a fresh clone.
