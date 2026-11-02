# PX4 Upstream Dependency

AERIS does not vendor PX4-Autopilot. The autopilot is cloned as a separate upstream dependency, pinned to a tag and recorded by commit.

## Pin

| Item | Value |
|---|---|
| Repository | `https://github.com/PX4/PX4-Autopilot.git` |
| Tag | `v1.17.0` |
| Commit | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| Default location | `~/aeris_dev/PX4-Autopilot` |
| Override | `AERIS_PX4_DIR` |
| Simulation target | `px4_sitl gz_x500` |
| Gazebo version | Harmonic (`gz sim` 8.x) |

The annotated tag object is `a5eb12d2ab591251faa009f76b2685b8cc64405d`; it dereferences to the commit above.

## Why external?

- PX4 is large (source + submodules + build tree).
- AERIS does not modify PX4 in Phase 1.
- Upgrades should be deliberate, reviewable events, not accidental submodule bumps.
- The flight-control boundary is the DDS topic contract, not PX4's source tree.

If a PX4 patch ever becomes necessary, it will be carried as a documented patch series or a fork with its own pinned commit — not an unrecorded local edit.

## Setup

```bash
bash scripts/bootstrap_px4.sh
```

The script:

1. Clones into `~/aeris_dev/PX4-Autopilot` if absent.
2. Checks out `v1.17.0`.
3. Runs `git submodule update --init --recursive`.
4. Runs `Tools/setup/ubuntu.sh --no-nuttx` (unless `AERIS_INSTALL_NUTTX=1`).

The NuttX toolchain is not required for SITL. It is only needed for embedded targets.

## Run

```bash
source scripts/aeris_env.sh
cd ~/aeris_dev/PX4-Autopilot
make px4_sitl gz_x500
```

Or use the project wrapper, which also starts the DDS agent:

```bash
bash scripts/launch_simulation.sh
```

Headless:

```bash
HEADLESS=1 bash scripts/launch_simulation.sh
```

## Verify the pin

```bash
cd ~/aeris_dev/PX4-Autopilot
git describe --tags --exact-match
git rev-parse HEAD
git submodule status --recursive | head
```

Expected tag: `v1.17.0`.

## Upgrade policy

When moving to a new PX4 release:

1. Read the upstream release notes and breaking changes.
2. Update `PX4_VERSION` in `scripts/bootstrap_px4.sh`.
3. Update `px4_msgs` to the matching release branch/tag and commit the submodule change.
4. Re-run the full Milestone 1 acceptance check.
5. Record the old/new versions, commands, and any behavioural differences in `docs/experiments/development_log.md`.
6. Only then update this document and `README.md`.

Do not upgrade PX4 and change AERIS flight logic in the same commit.

## Relationship to `px4_msgs`

`px4_msgs` is a separate upstream repository, included in this project as a Git submodule at:

```text
ros2_ws/src/px4_msgs
```

It is pinned to `v1.17.0` / `release/1.17` (commit `86d8239e962f6939e05c3737784f60c02fa884db`), matching the PX4 firmware. A firmware/message mismatch is a common source of silent field errors, so the two versions are always updated together.

## Useful references

- PX4 user guide: <https://docs.px4.io/main/en/>
- PX4 ROS 2 integration: <https://docs.px4.io/main/en/ros2/user_guide>
- Gazebo simulation: <https://docs.px4.io/main/en/sim_gazebo_gz/>
- PX4 releases: <https://github.com/PX4/PX4-Autopilot/releases>
