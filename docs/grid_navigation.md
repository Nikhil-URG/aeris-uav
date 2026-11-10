# Grid Navigation and Goal GUI

This extension adds a city-grid environment, occupied-cell planning and a
PyQt5 GUI for selecting a destination. The drone plans the shortest free path,
flies to the goal, returns to the start cell and lands.

## 1. Grid map

The canonical map is:

```text
ros2_ws/src/aeris_bringup/config/grids/city_grid.yaml
```

Schema:

```yaml
grid:
  name: city_grid
  width: 11
  height: 11
  cell_size_m: 4.0
  start: {x: 5, y: 5}
  cruise_altitude_m: 5.0
  buildings:
    - {x: 1, y: 1, width: 3, height: 3, height_m: 12.0}
```

- Cells are indexed `(x, y)`; x is north and y is east.
- A building occupies every cell in its rectangular `width × height` block.
- The start cell is always free.
- Cell centres map to PX4 local NED as:

```text
ned_x = (cell.x - start.x) * cell_size_m
ned_y = (cell.y - start.y) * cell_size_m
```

The `city_grid` has 121 cells, 12 buildings, 44 occupied cells and 77 free
cells. Buildings are 9–16 m tall, above the 5 m cruise altitude, so the drone
must route around them horizontally.

Spawn the matching Gazebo buildings:

```bash
source scripts/aeris_env.sh
bash scripts/spawn_grid.sh
```

The spawn script generates one static box per building, sized to its cell
block with a 1 m visual/collision gap. A 3×3 block is 11 m wide; adjacent free
cell centres are 2.5 m from the building edge.

## 2. Shortest-path planner

Package: `aeris_grid_navigation`  
Planner: A* on the free-cell graph.

- 8-connected moves; orthogonal cost 1, diagonal cost √2.
- Euclidean heuristic (admissible for the move costs).
- Diagonal moves are rejected if either orthogonal neighbour is occupied, so
  the path cannot cut through a building corner.
- The result includes the start and goal cells.
- The path length is the sum of the straight NED segments between cell
  centres.

Example: `(5,5) → (0,0)` on `city_grid` is 11 cells / 40 m because the direct
diagonal is blocked by a 3×3 building block.

## 3. Mission execution

Package: `aeris_grid_navigation`  
Executable: `grid_navigation`

Service:

```text
/aeris_grid_navigation/set_goal
  aeris_interfaces/srv/GridGoal
  request:  int32 x, int32 y
  response: accepted, message, path_cells, path_length_m
```

Status:

```text
/aeris/grid/status    aeris_interfaces/msg/GridStatus
```

The node:

1. validates the goal cell and plans the path from the current cell;
2. starts the offboard controller if the vehicle is not already in HOLD;
3. calls `go_to` for each cell centre at -5 m NED;
4. waits for the exact goal sequence and `goal_reached`;
5. holds for 3 s at the goal;
6. plans and flies the return path to the start cell;
7. calls `land` and waits for disarm;
8. writes `results/logs/grid_<timestamp>/grid.jsonl`.

It supports repeated goals in one stack: the offboard controller accepts a
new `start` after `DONE` or `ABORTED` once the vehicle is disarmed and
preflight checks pass.

If `aeris_safety` is running, the grid node stops sequencing on an armed
unsafe status and does not override the supervisor's land/RTL action.

## 4. GUI

Package: `aeris_grid_gui`  
Executable: `grid_gui`

The GUI loads the same grid YAML and draws:

- free cells and occupied building cells;
- building heights;
- start and goal cells;
- the planned path;
- the live drone position and heading;
- mission state, path length, remaining distance and progress;
- the safety verdict.

Click any free cell to send the drone there. The drone starts automatically,
flies the shortest path, returns and lands. Buttons:

- **Go Home** — select the start cell as the goal;
- **Land** — request a landing at the current position;
- **Quit**.

The launch files force the Qt `xcb` platform for consistent behavior under
XWayland, matching QGroundControl's setup.

## 5. Run the complete demo

Prerequisites: QGroundControl running and connected.

Terminal A — simulation:

```bash
cd /home/lucifer/Documents/Project_Aeris
source scripts/aeris_env.sh
PX4_SIM_MODEL=gz_x500 bash scripts/launch_simulation.sh
```

> Use `gz_x500`, not the camera airframe, for the grid demo. The camera +
> large city world caused the Gazebo server to crash during a long flight in
> this environment; the lighter airframe is stable and the grid demo does not
> need the camera.

Terminal B — buildings:

```bash
source scripts/aeris_env.sh
bash scripts/spawn_grid.sh
```

Terminal C — full stack + GUI:

```bash
source scripts/aeris_env.sh
ros2 launch aeris_bringup grid_demo.launch.py
```

Headless stack (no GUI):

```bash
ros2 launch aeris_bringup grid_navigation.launch.py
```

Select a cell in the GUI, or call the service:

```bash
ros2 service call /aeris_grid_navigation/set_goal \
  aeris_interfaces/srv/GridGoal "{x: 0, y: 0}"
```

## 6. Results — 2026-11-10

| Run | Goal | Outbound path | Return path | Outcome |
|---|---|---|---|---|
| 1 | `(0,0)` | 11 cells / 40 m | 11 cells / 40 m | `DONE`, disarmed |
| 2 | `(10,10)` | 11 cells / 40 m | 11 cells / 40 m | `DONE`, disarmed |
| 3 | `(2,5)` from `DONE` | 4 cells / 12 m | 4 cells / 12 m | `DONE`, disarmed |

Validation:

- all path cells and return cells are free;
- 131 logged flight positions checked against building footprints;
- minimum horizontal clearance: **2.269 m**;
- positions inside buildings: **0**;
- repeated goal accepted after `DONE` without restarting the stack.

Tests:

- `aeris_grid_navigation`: 9/9 planner tests;
- `aeris_grid_gui`: 1/1 headless click-mapping test;
- full workspace: **42 colcon tests + 4 evaluator tests**.

Evidence:

- `docs/experiments/grid_mission.jsonl`
- `docs/experiments/grid_repeat_goal.jsonl`
- `docs/experiments/grid_repeat_short.jsonl`
- `docs/experiments/grid_mission_run.log`
- `docs/experiments/grid_repeat_run.log`
- `docs/experiments/grid_validation.txt`
- `docs/experiments/plots/grid_gui.png`
- `docs/experiments/plots/grid_gui_repeat.png`
- `docs/experiments/plots/grid_gui_demo.png`

## 7. Limitations

- The map is 2D and static; there is no dynamic obstacle or no-fly-zone update.
- Buildings are axis-aligned rectangular blocks; arbitrary footprints are not
  represented.
- The planner treats a cell as occupied or free; it does not model building
  height in the path cost because the drone cruises below all roofs.
- The drone follows cell centres with an 0.8 m waypoint tolerance; it does not
  smooth the path into time-optimal curves.
- The GUI is a local desktop application, not a web interface.
- The Gazebo/GUI stack is heavier than the headless stack; close QGC or other
  GPU clients if the simulation stutters.
