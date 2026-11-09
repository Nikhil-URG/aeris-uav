# Perception

This document describes the Week 4 camera, detection and localization stack.

## Camera model

AERIS uses the PX4 Gazebo model `gz_x500_mono_cam_down`:

- downward-facing monocular camera, mounted on the X500;
- 1280 × 960 image;
- horizontal field of view 1.74 rad;
- sensor name `imager`, camera link `camera_link`.

Start the camera-enabled simulation:

```bash
PX4_SIM_MODEL=gz_x500_mono_cam_down bash scripts/launch_simulation.sh
```

Gazebo publishes:

```text
/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image
/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info
```

`ros_gz_bridge` maps them to:

```text
/aeris/camera/image_raw       sensor_msgs/msg/Image
/aeris/camera/camera_info     sensor_msgs/msg/CameraInfo
```

Bridge configuration: `ros2_ws/src/aeris_bringup/config/gz_camera_bridge.yaml`.

## Target model

The inspection target is a static red panel:

```text
simulation/models/red_target/model.sdf
```

Spawn the targets used by `inspection_alpha`:

```bash
bash scripts/spawn_targets.sh
```

The script spawns:

| Target | Gazebo position |
|---|---|
| `red_target_wp01` | `(4.0, 0.0, 0.05)` |
| `red_target_wp02` | `(4.0, 4.0, 0.05)` |

The mission waypoints `wp_01` and `wp_02` are directly above these panels.

## Perception node

Package: `aeris_perception`  
Executable: `perception`  
Launch: `ros2 launch aeris_bringup perception.launch.py`

### Inputs

| Topic | Type |
|---|---|
| `/aeris/camera/image_raw` | `sensor_msgs/msg/Image` |
| `/aeris/camera/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/fmu/out/vehicle_local_position_v1` | `px4_msgs/msg/VehicleLocalPosition` |

### Output

| Topic | Type |
|---|---|
| `/aeris/perception/detections` | `aeris_interfaces/msg/DetectionArray` |

Each `Detection` contains:

- target id/class and confidence;
- pixel bounding box;
- bearing in radians;
- slant range in metres;
- estimated local NED position (approximate).

### Algorithm

1. Convert the image with `cv_bridge`.
2. Convert BGR → HSV.
3. Threshold two red hue bands (0–10 and 170–180) with saturation/value floors.
4. Morphological open to remove speckle.
5. Find external contours and reject small areas.
6. Publish the largest candidate(s), sorted by area.
7. Compute bearing from the camera intrinsics.
8. Estimate slant range from vehicle altitude and the pixel's angular offset.
9. Project the pixel to the ground plane and rotate the body-frame offset by vehicle yaw into local NED.

This is deliberately classical computer vision: no YOLO or neural network is needed for the first inspection target.

## Run

Terminal 1 — camera simulation:

```bash
PX4_SIM_MODEL=gz_x500_mono_cam_down bash scripts/launch_simulation.sh
```

Terminal 2 — targets:

```bash
bash scripts/spawn_targets.sh
```

Terminal 3 — bridge and perception:

```bash
source scripts/aeris_env.sh
ros2 launch aeris_bringup perception.launch.py
```

Terminal 4 — mission:

```bash
source scripts/aeris_env.sh
ros2 launch aeris_bringup mission.launch.py
ros2 service call /aeris_mission_manager/start std_srvs/srv/Trigger
```

## Results — 2026-11-09

- Camera bridge: 1280 × 960, `fx=fy=539.9`, `cx=640`, `cy=480`, ~12.8 Hz.
- Detector unit tests: **4/4 passing**.
- Mission: 2/2 waypoints; `target_detected` logged at both inspection points.
- Detection confidence: 1.00 at both waypoints.
- Slant range: ~6.27 m and ~6.28 m (target offset from the vehicle's nadir).
- Mission final state: `COMPLETE`.

Evidence:

- `docs/experiments/week4_mission.jsonl`
- `docs/experiments/week4_perception.log`
- `docs/experiments/week4_mission_run.log`

## Known limitation — localization accuracy

The ground projection assumes a perfectly nadir camera, square pixels, and the body-frame image orientation used by the X500 mono-cam-down model. It is accurate enough to report a detection and a rough ground point, but it is **not** a calibrated extrinsic solution.

The downward camera also sees the neighbouring target at the edge of the frame, so the detected target may not be the panel directly under the vehicle. For Week 4 the acceptance criterion is robust detection at inspection waypoints; high-accuracy target localization is a Week 5/7 refinement (camera extrinsics, homography calibration, or a depth camera).

## Tests

```bash
colcon test --packages-select aeris_perception
colcon test-result --verbose
```

Synthetic-image tests cover:

- red rectangle detection and bounding box;
- blank image rejection;
- small-speck rejection;
- centre-pixel ground projection.
