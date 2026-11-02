# AERIS Development Log

This is an engineering journal, not a changelog. Record what was attempted, what worked, what failed, why decisions were made, and what was measured.

---

## 2026-11-02 — Environment audit and Week 1 bootstrap

### Goal

Establish the machine state before installing anything, then create a reproducible Week 1 scaffold for the simulation foundation and telemetry monitor.

### Environment observed

| Item | Value |
|---|---|
| OS | Pop!_OS 24.04 LTS (Ubuntu 24.04 `noble` base) |
| Kernel | `7.1.5-76070105-generic` |
| CPU | Intel Core i5-13420H, 8 cores / 12 threads |
| RAM | 15 GiB total, 9.2 GiB available, 19 GiB swap |
| GPU | NVIDIA RTX 4050 Laptop, 6 GiB, driver 580.173.02 |
| Disk | 54 GiB free on `/` (88% used) |
| Session | Wayland / COSMIC, XWayland display `:1` |
| Python | System 3.12.3; Miniforge 3.13.13 first in `PATH` |
| ROS 2 | not installed |
| Gazebo | not installed |
| Build tools | `git`, `gcc`, `g++`, `make`, `terminator` present; `cmake`, `ninja`, `colcon`, `vcstool`, `rosdep` missing |
| Sudo | password required |

### Findings that changed the plan

1. **Conda shadows the system Python.**
   - `command -v python3` resolves to `/home/lucifer/miniforge3/bin/python3` (3.13.13).
   - ROS 2 Jazzy and PX4 SITL require system Python 3.12.
   - PX4's `Tools/setup/ubuntu.sh` would install its Python requirements into the conda base environment if run unchanged.
   - Decision: add `scripts/aeris_env.sh` to sanitize `PATH`, `PYTHONPATH` and `LD_LIBRARY_PATH`; make the PX4 bootstrap script refuse to run with the wrong interpreter. Do not modify the user's global conda configuration.

2. **ROS apt HTTPS certificate mismatch.**
   - `https://packages.ros.org` resolves to an OSUOSL mirror with a certificate for `*.osuosl.org`, so TLS validation fails.
   - `http://packages.ros.org` works, and the official `ros2-apt-source` 1.3.0 package configures HTTP.
   - Decision: use the official apt source package, pinned to 1.3.0; document the observation.

3. **Disk is adequate but not generous.**
   - 54 GiB free; PX4, Gazebo, ROS desktop and build artifacts will consume a significant fraction.
   - Decision: skip the blanket `apt upgrade`, install PX4 without the NuttX toolchain by default, and keep result artifacts out of Git.

4. **Wayland/COSMIC.**
   - Gazebo and QGC should run through XWayland (`QT_QPA_PLATFORM=xcb` fallback).
   - Headless PX4/Gazebo runs will be used for automated checks.

### Decisions

| ID | Decision | Rationale |
|---|---|---|
| D-001 | Repository root is `/home/lucifer/Documents/Project_Aeris` | It is the active workspace; the Git repository can still be named `aeris-uav` |
| D-002 | PX4 remains external at `~/aeris_dev/PX4-Autopilot` | Keeps this repository reviewable and pins upstream explicitly |
| D-003 | Pin PX4 `v1.17.0` (commit `d6f12ad`), `px4_msgs` `v1.17.0` (commit `86d8239`), Micro XRCE-DDS Agent `v2.4.3` (commit `7362281`) | Reproducibility |
| D-004 | Use a `px4_msgs` Git submodule at `ros2_ws/src/px4_msgs` | Records the exact message revision in the superproject |
| D-005 | Keep `aeris_flight_control` and `aeris_bringup` as the only active packages in Week 1 | Avoid building architecture that has no milestone yet |
| D-006 | Default PX4 setup without NuttX; opt in with `AERIS_INSTALL_NUTTX=1` | SITL does not need the embedded toolchain; saves disk/time |
| D-007 | Use best-effort QoS on `/fmu/...` topics | Matches PX4 uXRCE-DDS publications |

### Implemented

- Repository scaffold for Week 1.
- `aeris_flight_control` C++ package with the read-only `telemetry_monitor` node.
- `aeris_bringup` Python package with a telemetry launch file and YAML parameters.
- `px4_msgs` submodule pinned to `v1.17.0`.
- Setup scripts for system/ROS, PX4/Gazebo, Micro XRCE-DDS Agent, QGC, build, simulation launch, environment checks, and a one-command orchestrator (`scripts/setup.sh`).
- Architecture, installation, flight-control and mission-design documentation.

### Commands run (audit only, no installs yet)

```bash
cat /etc/os-release
uname -r
python3 --version
which -a python3
nvidia-smi
df -h ~
git ls-remote --tags https://github.com/PX4/PX4-Autopilot.git refs/tags/v1.17.0
git ls-remote --tags https://github.com/PX4/px4_msgs.git refs/tags/v1.17.0
```

### Pending

- Run `scripts/install_dependencies.sh`.
- Run `scripts/bootstrap_px4.sh`.
- Run `scripts/install_microxrce_agent.sh`.
- Build the workspace and run the first telemetry session.
- Record the Milestone 1 acceptance results below.

### Milestone 1 run (to be filled after the first successful session)

| Check | Result | Evidence |
|---|---|---|
| PX4 SITL boots | pending | |
| Gazebo shows X500 | pending | |
| QGC connects | pending | |
| DDS agent connects | pending | |
| `/fmu/*` topics visible | pending | |
| `telemetry_monitor` builds | pending | |
| Position telemetry continuous | pending | |
| Estimator validity | pending | |

Measured message rate: pending.

---

## Entry template

### YYYY-MM-DD — Title

**Goal:**

**Environment / versions:**

**What was implemented:**

**What worked:**

**What failed:**

**Debugging decisions:**

**Measurements:**

**Next steps:**
