# Grid Navigation and Goal GUI

This extension adds a city-grid environment, occupied-cell planning and a
PyQt5 GUI for selecting a destination. The drone plans the shortest free path,
flies to the goal, returns to the start cell and lands.

## 1. Grid map

The default map is:

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
  start: {x: 0, y: 0}
  cruise_altitude_m: 5.0
  buildings:
    - {x: 2, y: 2, width: 2, height: 2, height_m: 10.0}
```

- Cells are indexed `(x, y)`; x is north and y is east.
- A building occupies every cell in its rectangular `width × height` block.
- The start cell is always free. The default map starts at `(0,0)`, the
  south-west corner of the city.
- Cell centres map to PX4 local NED as:

```text
ned_x = (cell.x - start.x) * cell_size_m
ned_y = (cell.y - start.y) * cell_size_m
```

The default `city_grid` has 121 cells, **9 evenly spaced 2×2 buildings**,
**36 occupied cells** and **85 free cells**. Buildings are 10–16 m tall, above
the 5 m cruise altitude, so the drone routes around them horizontally. The
blocks sit on a 3×3 lattice with one-cell streets between them.

`city_grid_dense.yaml` preserves the initial 12-building map used for the
recorded SITL evidence. The planner and executor are unchanged; the default
map is a data-only layout revision.

Spawn the matching Gazebo buildings:

```bash
source scripts/aeris_env.sh
bash scripts/spawn_grid.sh
```

The spawn script generates one static box per building, sized to its cell
block with a 1 m visual/collision gap. A 2×2 block is 7 m wide; adjacent free
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

Example: `(0,0) → (10,10)` on the default `city_grid` is **20 cells / 77.7 m**
because the direct diagonal is blocked by the building lattice. The
straight-line displacement to the far corner is 56.6 m, within the offboard
controller's 65 m horizontal limit; the path length is longer because it
follows the streets.

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
  aeris_interfaces/srv/GridGoal "{x: 10, y: 10}"
```

## 6. Results — 2026-11-10

### Initial dense-map SITL validation

The first flight evidence used `city_grid_dense.yaml` (12 buildings, start
`(5,5)`). The planner, executor and GUI code are unchanged by the layout
revision.

| Run | Goal | Outbound path | Return path | Outcome |
|---|---|---|---|---|
| 1 | `(0,0)` | 11 cells / 40 m | 11 cells / 40 m | `DONE`, disarmed |
| 2 | `(10,10)` | 11 cells / 40 m | 11 cells / 40 m | `DONE`, disarmed |
| 3 | `(2,5)` from `DONE` | 4 cells / 12 m | 4 cells / 12 m | `DONE`, disarmed |

Validation on the dense map:

- all path cells and return cells free;
- 131 logged flight positions checked against building footprints;
- minimum horizontal clearance: **2.269 m**;
- positions inside buildings: **0**;
- repeated goal accepted after `DONE` without restarting the stack.

### Default spread-map planner validation

| Goal from `(0,0)` | Path | Path length | Min clearance |
|---|---|---|---|
| `(10,10)` | 20 cells | 77.7 m | 2.500 m |
| `(0,10)` | 11 cells | 40.0 m | 6.500 m |
| `(10,0)` | 11 cells | 40.0 m | 6.500 m |
| `(4,4)` | 8 cells | 29.7 m | 2.500 m |
| `(7,7)` | 14 cells | 53.7 m | 2.500 m |

All path cells are free; worst path clearance is **2.500 m**. This is an
offline geometric validation of the default map, not a new flight run. The
executor has already been flight-validated on the dense map.

Tests:

- `aeris_grid_navigation`: 9/9 planner tests;
- `aeris_grid_gui`: 1/1 headless click-mapping test;
- full workspace: **42 colcon tests + 4 evaluator tests**.

Evidence:

- `docs/experiments/grid_mission.jsonl` (dense map)
- `docs/experiments/grid_repeat_goal.jsonl` (dense map)
- `docs/experiments/grid_repeat_short.jsonl` (dense map)
- `docs/experiments/grid_mission_run.log`
- `docs/experiments/grid_repeat_run.log`
- `docs/experiments/grid_validation.txt`
- `docs/experiments/grid_layout_validation.txt` (default spread map)
- `docs/experiments/plots/grid_gui.png`
- `docs/experiments/plots/grid_gui_repeat.png`
- `docs/experiments/plots/grid_gui_demo.png`
- `docs/experiments/plots/grid_gui_spread.png` (default layout preview)

## 7. Limitations

- The map is 2D and static; there is no dynamic obstacle or no-fly-zone update.
- Buildings are axis-aligned rectangular blocks; arbitrary footprints are not
  represented.
- The planner treats a cell as occupied or free; it does not model building
  height in the path cost because the drone cruises below all roofs.
- The drone follows cell centres with an 0.8 m waypoint tolerance; it does not
  smooth the path into time-optimal curves.
- The offboard horizontal limit applies to each target's straight-line
  displacement from takeoff, not the accumulated path length. The default far
  corner is 56.6 m away, within the 65 m limit.
- The GUI is a local desktop application, not a web interface.
- The Gazebo/GUI stack is heavier than the headless stack; close QGC or other
  GPU clients if the simulation stutters.
