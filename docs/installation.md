# Installation

This document describes the exact environment used by AERIS. The setup is scripted so a fresh machine can be reproduced without relying on shell history.

## Supported environment

- Ubuntu 24.04 LTS or Pop!_OS 24.04 LTS (both report `noble`)
- ROS 2 Jazzy
- PX4-Autopilot `v1.17.0`
- Gazebo Harmonic (installed by the PX4 setup script)
- Micro XRCE-DDS Agent `v2.4.3`
- QGroundControl `v5.1.5` (optional, recommended)

The development machine used for the first milestone:

| Item | Value |
|---|---|
| OS | Pop!_OS 24.04 LTS |
| Kernel | 7.1.5-76070105-generic |
| CPU | Intel i5-13420H, 12 threads |
| RAM | 15 GiB |
| GPU | NVIDIA RTX 4050 Laptop, 6 GiB |
| Disk | 54 GiB free at setup time |

## Important: keep conda out of the ROS shell

This machine has Miniforge first in `PATH`, so `python3` resolves to Python 3.13. ROS 2 Jazzy and PX4 SITL require the system Python 3.12 at `/usr/bin/python3`.

Use the environment script in every shell used for ROS or PX4 work:

```bash
source scripts/aeris_env.sh
```

It:

- deactivates conda if a conda environment is active;
- removes conda/Miniforge paths from `PATH`, `PYTHONPATH` and `LD_LIBRARY_PATH`;
- sources `/opt/ros/jazzy/setup.bash`;
- sources `ros2_ws/install/setup.bash` when the workspace has been built.

Verify:

```bash
which python3        # must be /usr/bin/python3
python3 --version    # must be Python 3.12.x
```

Do not run PX4's `Tools/setup/ubuntu.sh` directly from a conda shell. `scripts/bootstrap_px4.sh` sanitizes the environment first.

## One-command setup

Run as your **normal user**, not with `sudo`:

```bash
bash scripts/setup.sh
```

This runs the four stages below in order. If you prefer to inspect each stage, run them individually.

## 1. System dependencies and ROS 2 Jazzy

Run as your **normal user**, not with `sudo`:

```bash
bash scripts/install_dependencies.sh
```

The script installs:

- build tools: `cmake`, `ninja-build`, `build-essential`, `git`, `curl`, `wget`
- Python tooling: `python3`, `python3-pip`, `python3-venv`
- ROS 2 development tools: `colcon`, `vcstool`, `rosdep`
- the official ROS 2 apt source package (`ros2-apt-source` 1.3.0, `noble`)
- `ros-jazzy-desktop` and `ros-dev-tools`
- libraries needed by the DDS agent and QGC AppImage

It does **not** run a blanket `apt upgrade`. On a Pop!_OS workstation, a broad upgrade can pull desktop and kernel changes unrelated to AERIS; those should be handled separately.

### Why HTTP for the ROS repository?

On this network, `https://packages.ros.org` resolves to an OSUOSL mirror whose certificate only covers `*.osuosl.org`. HTTPS therefore fails certificate validation. The official `ros2-apt-source` package already configures `http://packages.ros.org/ros2/ubuntu`, so the standard installation path works. This is recorded in the development log because it looks alarming in a browser.

### Verify ROS 2

```bash
source /opt/ros/jazzy/setup.bash
ros2 --version
printenv ROS_DISTRO
```

Expected:

```text
jazzy
```

## 2. PX4 and Gazebo Harmonic

```bash
bash scripts/bootstrap_px4.sh
```

By default the script:

1. clones PX4-Autopilot into `~/aeris_dev/PX4-Autopilot` (override with `AERIS_PX4_DIR`);
2. checks out tag `v1.17.0`;
3. initializes all submodules;
4. runs PX4's Ubuntu setup with `--no-nuttx`;
5. installs Gazebo Harmonic and the simulation dependencies.

The NuttX embedded toolchain is not needed for SITL and costs significant disk/time. To install it anyway:

```bash
AERIS_INSTALL_NUTTX=1 bash scripts/bootstrap_px4.sh
```

Verify:

```bash
source scripts/aeris_env.sh
cd ~/aeris_dev/PX4-Autopilot
git describe --tags
gz sim --versions | head -1
```

Expected: `v1.17.0` and a Gazebo Harmonic version (`8.x`).

## 3. Micro XRCE-DDS Agent

```bash
bash scripts/install_microxrce_agent.sh
```

The script pins the agent to `v2.4.3`, builds it from source and installs it to `/usr/local/bin`. Verify:

```bash
command -v MicroXRCEAgent
MicroXRCEAgent --help | head -5
```

## 4. Build the ROS 2 workspace

```bash
bash scripts/build.sh
```

This initializes the `px4_msgs` submodule if needed and runs:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

After a successful build:

```bash
source scripts/aeris_env.sh
ros2 pkg list | grep aeris
```

Expected packages: `aeris_bringup`, `aeris_flight_control`, `px4_msgs`.

## 5. Run the simulation and telemetry

Terminal 1 — DDS agent and PX4 SITL:

```bash
bash scripts/launch_simulation.sh
```

The script starts `MicroXRCEAgent udp4 -p 8888`, then runs `make px4_sitl gz_x500`. The PX4 shell appears after the build. For a headless run (useful for automation):

```bash
HEADLESS=1 bash scripts/launch_simulation.sh
```

Terminal 2 — telemetry monitor:

```bash
source scripts/aeris_env.sh
ros2 launch aeris_bringup telemetry.launch.py
```

Expected output (values are illustrative):

```text
[aeris_telemetry_monitor]: AERIS telemetry monitor initialized (topic='/fmu/out/vehicle_local_position', log_every_n=50).
[aeris_telemetry_monitor]: Position [NED] X=0.02 Y=-0.01 Z=-0.04 | Velocity [NED] VX=0.00 VY=0.01 VZ=0.00 | Speed=0.01 m/s | PosValid=YES VelValid=YES | Moving=NO
```

Check that PX4 topics are visible:

```bash
source scripts/aeris_env.sh
ros2 topic list | grep /fmu/
ros2 topic hz /fmu/out/vehicle_local_position
```

## 6. QGroundControl (optional but recommended)

```bash
bash scripts/install_qgroundcontrol.sh
scripts/run_qgroundcontrol.sh
```

QGC is downloaded as an AppImage into `~/Applications`. The run wrapper forces the X11 (XWayland) Qt backend, which is the reliable path on this COSMIC/Wayland desktop.

QGC should auto-connect to PX4 SITL on UDP 14550 after the simulation starts. Use it to monitor arming state, flight mode, altitude, battery, GPS/estimator status and the vehicle map.

## Milestone 1 acceptance checklist

| Test | Expected result |
|---|---|
| PX4 SITL | Boots successfully |
| Gazebo | X500 visible |
| QGroundControl | Connects to PX4 |
| DDS agent | Establishes connection |
| ROS 2 topics | PX4 topics discoverable |
| C++ telemetry node | Compiles and executes |
| Position telemetry | Received continuously |
| Estimator validity | Valid during normal flight conditions |

Record the first successful run in [experiments/development_log.md](experiments/development_log.md).

## Troubleshooting

### `ros2: command not found`

Source the environment:

```bash
source scripts/aeris_env.sh
```

### Wrong Python / `ModuleNotFoundError` / colcon import errors

A conda environment is still active. Check:

```bash
which python3
python3 --version
```

It must be `/usr/bin/python3` and Python 3.12. Re-source `scripts/aeris_env.sh`; if the shell was started from a conda-activated session, the script removes it. Do not run `colcon build` from a conda shell.

### `ros2 topic list` is empty

1. Confirm `MicroXRCEAgent udp4 -p 8888` is running.
2. Confirm PX4 SITL is running and has printed DDS client creation messages.
3. Confirm both shells use the same `ROS_DOMAIN_ID` (default 0).
4. Restart the ROS 2 daemon:

   ```bash
   ros2 daemon stop
   ros2 topic list
   ```

5. Check that a firewall is not blocking DDS multicast/local traffic.

### Gazebo GUI fails or is black on Wayland/COSMIC

Try the X11 backend:

```bash
QT_QPA_PLATFORM=xcb bash scripts/launch_simulation.sh
```

For automated runs or low-memory situations, use headless mode:

```bash
HEADLESS=1 bash scripts/launch_simulation.sh
```

### PX4 build fails while installing Python packages

The PX4 setup script is using conda Python. Re-run through the bootstrap script, which sanitizes `PATH`:

```bash
bash scripts/bootstrap_px4.sh
```

Confirm `python3 --version` is 3.12 before the PX4 setup step.

### QGroundControl AppImage does not start

- Ensure FUSE is installed: `libfuse2t64`.
- Try extracting the AppImage:

  ```bash
  cd ~/Applications
  ./QGroundControl.AppImage --appimage-extract
  ./squashfs-root/AppRun
  ```

- Force X11: the `scripts/run_qgroundcontrol.sh` wrapper already sets `QT_QPA_PLATFORM=xcb`.

### Low disk space

PX4 builds and Gazebo caches grow quickly. Check with:

```bash
df -h ~
du -sh ~/aeris_dev/PX4-Autopilot/build ~/.ccache 2>/dev/null
```

The project keeps `results/logs/` and `results/plots/` out of Git; prune old runs when needed.
