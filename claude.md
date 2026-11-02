# AERIS — Autonomous UAV Inspection & Reliability System

Development phase 1



We will build AERIS as a modular UAV autonomy platform, using PX4 as the flight controller and ROS 2 as the autonomy layer.

The project will be developed as if it were an actual robotics software engineering product: modular architecture, reproducible setup, automated testing, simulation scenarios, documented interfaces and measurable performance.

Our first objective is not to build an intelligent drone. It is to establish a reliable simulated flight-control system that we can progressively make autonomous.

## 1. Project definition

Mission: Autonomous industrial inspection.

The simulated drone will eventually:

1. Arm and take off.
2. Navigate to inspection locations.
3. Maintain stable position and altitude.
4. Observe inspection targets using its camera.
5. Detect and avoid obstacles.
6. Replan its trajectory when necessary.
7. Return to its launch position.
8. Land safely.
9. Report mission results and failures.

### Technical architecture

Mission Planning Layer

Inspection goals · Waypoints · Mission state machine

Perception

Camera · OpenCV · Detection

Navigation

Planner · Trajectory · Avoidance

Flight Interface

ROS 2 · PX4 messages · uXRCE-DDS

PX4 Autopilot

EKF2 · Position controller · Attitude controller · Actuators

Gazebo Simulation

Quadrotor · Physics · Sensors · Environment

Reliability & Observability

Logging · Health monitoring · Fault injection · Evaluation

One architectural decision is important: PX4 remains responsible for low-level flight stabilization. Our ROS 2 application handles mission-level intelligence and sends appropriate setpoints to PX4.

This separation makes the project more realistic and allows us to eventually support another autopilot.

## 2. Development roadmap

I suggest a 7-week development plan, with a working deliverable at the end of every week.

## AERIS development schedule

Target: approximately 8–12 hours per week

### Week 1

Simulation foundation

PX4 SITL, Gazebo, QGroundControl, ROS 2 connectivity.

Simulated drone flying and telemetry visible.

### Week 2

Offboard flight controller

C++ ROS 2 node, arming, takeoff, position setpoints and landing.

Programmatically controlled flight.

### Week 3

Mission manager

State machine, waypoint missions, mission validation and logging.

Autonomous inspection route.

### Week 4

Perception

Camera integration, OpenCV, target detection and target localization.

Drone identifies inspection targets.

### Week 5

Navigation

Depth camera, obstacle detection, local planning and replanning.

Obstacle-aware inspection.

### Week 6

Safety and reliability

Fault injection, failsafes, return-to-launch and automated tests.

Recoverable simulated failures.

### Week 7

Evaluation and portfolio

Benchmarking, experiment reports, documentation and demo video.

Publishable engineering project.

We will not rush into YOLO or neural networks. The first three weeks establish the control architecture on which everything else depends.

## 3. Repository structure

I want the repository to look like a proper robotics software project rather than a collection of scripts.

Repository name: `aeris-uav`

```
aeris-uav/
│
├── README.md
├── LICENSE
├── .gitignore
├── .gitmodules
│
├── docs/
│   ├── architecture.md
│   ├── installation.md
│   ├── flight_control.md
│   ├── mission_design.md
│   └── experiments/
│
├── px4/
│   └── README.md
│
├── ros2_ws/
│   └── src/
│       ├── aeris_interfaces/
│       │   ├── msg/
│       │   └── srv/
│       │
│       ├── aeris_bringup/
│       │   ├── launch/
│       │   └── config/
│       │
│       ├── aeris_flight_control/
│       │   ├── src/
│       │   └── include/
│       │
│       ├── aeris_mission_manager/
│       │   ├── aeris_mission_manager/
│       │   └── test/
│       │
│       ├── aeris_navigation/
│       │   ├── src/
│       │   └── config/
│       │
│       ├── aeris_perception/
│       │   ├── aeris_perception/
│       │   └── models/
│       │
│       └── aeris_safety/
│           ├── src/
│           └── test/
│
├── simulation/
│   ├── worlds/
│   ├── models/
│   └── scenarios/
│
├── scripts/
│   ├── setup.sh
│   ├── launch_simulation.sh
│   └── evaluate_mission.py
│
├── tests/
│   ├── integration/
│   └── scenarios/
│
└── results/
    ├── logs/
    ├── plots/
    └── reports/
```

### What each package does

| Package                 | Language | Responsibility                       |
| ----------------------- | -------- | ------------------------------------ |
| `aeris_interfaces`      | ROS 2    | Custom messages and services         |
| `aeris_bringup`         | Python   | Launch configuration                 |
| `aeris_flight_control`  | C++      | PX4 communication and control        |
| `aeris_mission_manager` | Python   | Mission state machine                |
| `aeris_navigation`      | C++      | Trajectory and obstacle planning     |
| `aeris_perception`      | Python   | Image processing                     |
| `aeris_safety`          | C++      | Health checks and safety supervision |

We will avoid building all these packages immediately. They are the intended architecture; initially we only need `aeris_flight_control` and a basic `aeris_bringup`.

One important engineering choice: the PX4 repository will be a separate upstream dependency, not copied into our Git repository. We will pin its version and document the exact commit.



### Step A — Prepare the machine


```
sudo apt update
sudo apt upgrade -y

sudo apt install -y \
    git \
    curl \
    wget \
    build-essential \
    cmake \
    ninja-build \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    terminator
```

Install ROS 2 Jazzy using the official installation instructions:

[ROS 2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html?utm_source=chatgpt.com)

Install the development tools:

```
sudo apt install ros-jazzy-desktop ros-dev-tools
```

Add ROS to your shell:

```
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

source ~/.bashrc
```

Verify:

```
ros2 --version
printenv ROS_DISTRO
```

Expected distribution:

```
jazzy
```

### Step B — Install PX4

We will use the upstream PX4 repository rather than modifying its code initially.

```
mkdir -p ~/aeris_dev
cd ~/aeris_dev

git clone https://github.com/PX4/PX4-Autopilot.git \
    --recursive

cd PX4-Autopilot

git checkout v1.17.0

git submodule update --init --recursive
```

Install the PX4 development dependencies:

```
bash ./Tools/setup/ubuntu.sh
```

Restart your machine after the installation.

The script installs the required simulation and development toolchain.&#x20;

[image](https://www.google.com/s2/favicons?domain=https://docs.px4.io\&sz=32)

PX4 Guide (main)+1



### Step C — Run your first drone simulation

Navigate to PX4:

```
cd ~/aeris_dev/PX4-Autopilot

make px4_sitl gz_x500
```

This launches the X500 quadrotor in Gazebo with PX4 running in Software-in-the-Loop mode.

[Zoom In / Out in Gazebo Sim GUI - PX4 Autopilot - Discussion Forum for PX4, Pixhawk, QGroundControl, MAVSDK, MAVLink](https://images.openai.com/static-rsc-4/BLhMBuwpSmOuYrndFEC-maCSOa3jX8BiC7ZYzXeB1YEwI5ZHboDx5-5hMvfL1jQhQp-9Z8vi6R4cDP3Lv1gvFzOGyID-pEmbIWyGf00JemsnjqGbP6TcBrkg_cuTmD1Ayz3qrNathUh7UQsQ9hQBoHkgHmAvESbEM_ZqX2NgGew?purpose=inline)

[discuss.px4.io](https://discuss.px4.io/t/zoom-in-out-in-gazebo-sim-gui/48308?utm_source=chatgpt.com)

This command is the official PX4 Gazebo simulation target.&#x20;

[image](https://www.google.com/s2/favicons?domain=https://docs.px4.io\&sz=32)

PX4 Guide (main)+1



Success criteria:

- Gazebo opens.
- You see the X500 quadrotor.
- PX4 boots without errors.
- The vehicle appears in its initial position.
- You can interact with the simulation.

### Step D — Install QGroundControl

Download the Linux version:

[QGroundControl download](https://qgroundcontrol.com/download/?utm_source=chatgpt.com)

[Fly View | QGC Guide (4.4)](https://images.openai.com/static-rsc-4/4YfbCvOLiq-bYjiKwLltKfcutg9AlnvpAwFSZyQzoL9UHMx6jwvVD3d9g64tVFOg3vN9lDR4LT3eJGxZmDnvlUwJCg7bQI7xFGiDIz9eUVJxDJg4EekTGIkAwNU2824khcZ89T1rTWnRccni9LtGguxxt0MuOGE_PQwKckhoIyU?purpose=inline)

[docs.qgroundcontrol.com](https://docs.qgroundcontrol.com/Stable_V4.4/en/qgc-user-guide/fly_view/fly_view.html?utm_source=chatgpt.com)

QGroundControl will let us monitor:

- Vehicle position.
- Altitude.
- Flight mode.
- Battery status.
- GPS and estimator status.
- Arming state.

It will also serve as a ground station and monitoring interface during our experiments.

### Step E — Connect ROS 2 to PX4

This is the most important integration step.

PX4 uses uXRCE-DDS to expose internal flight-controller messages to ROS 2. A client runs inside PX4 and communicates with a Micro XRCE-DDS Agent running on our computer.&#x20;

[image](https://www.google.com/s2/favicons?domain=https://github.com\&sz=32)

GitHub+1



Create our ROS 2 workspace:

```
mkdir -p ~/aeris_dev/aeris-uav/ros2_ws/src
cd ~/aeris_dev/aeris-uav/ros2_ws/src
```

Clone the message definitions matching PX4 v1.17:

```
git clone -b release/1.17 \
    https://github.com/PX4/px4_msgs.git
```

Build:

```
cd ~/aeris_dev/aeris-uav/ros2_ws

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

Next, install the Micro XRCE-DDS Agent using the official PX4 instructions:

[PX4 ROS 2 integration guide](https://docs.px4.io/main/en/ros2/user_guide?utm_source=chatgpt.com)

The current Jazzy setup uses Micro XRCE-DDS Agent v2.4.3.

Start the agent in a separate terminal:

```
MicroXRCEAgent udp4 -p 8888
```

Then launch the PX4 simulator in another terminal:

```
cd ~/aeris_dev/PX4-Autopilot

make px4_sitl gz_x500
```

Once connected, PX4 should report successful creation of DDS data writers.

We can verify ROS 2 communication using:

```
source /opt/ros/jazzy/setup.bash

source ~/aeris_dev/aeris-uav/ros2_ws/install/setup.bash

ros2 topic list
```

You should see topics such as:

```
/fmu/out/vehicle_status
/fmu/out/vehicle_local_position
/fmu/out/vehicle_attitude
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
```

The exact available topics depend on the PX4 firmware's DDS topic configuration.

## 5. First implementation milestone: PX4 telemetry monitor

Milestone 1 — Simulation and telemetry

Before making the drone move autonomously, I want us to establish that our software can reliably observe the aircraft.

We will write our first AERIS C++ ROS 2 node.

### Objective

Create a node called:

`aeris_telemetry_monitor`

It will subscribe to PX4's local position topic and continuously report:

- X, Y, Z position.
- X, Y, Z velocity.
- Whether the position estimate is valid.
- Whether the vehicle is moving.

[Abel Mendoza - Robotics & Embedded Systems Engineer](https://images.openai.com/static-rsc-4/dwq__6yv-pV_X6McuiUHH4DYr4nsfwumX7qj2eaQAaooisWJmzgVRty6LCHiWKcMY8JKPWazT6mS96-ktEbfC7JkbOphTRoJHxnl6zC3leDqq3rpopQRj8pwgbeVC2qmQSxEoRCdPp7ba7BvGTIsUTnPDqcvo07FMDymMeVT1dU?purpose=inline)

[abelxmendoza.com](https://www.abelxmendoza.com/?utm_source=chatgpt.com)

### Implementation

Create the package:

```
cd ~/aeris_dev/aeris-uav/ros2_ws/src

ros2 pkg create aeris_flight_control \
    --build-type ament_cmake \
    --dependencies rclcpp px4_msgs
```

Create the source directory:

```
mkdir -p aeris_flight_control/src
```

Create:

`aeris_flight_control/src/telemetry_monitor.cpp`

```

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"

using namespace std::chrono_literals;

class TelemetryMonitor : public rclcpp::Node
{
public:
    TelemetryMonitor()
        : Node("aeris_telemetry_monitor")
    {
        auto qos = rclcpp::QoS(10).best_effort();

        subscription_ =
            this->create_subscription<
                px4_msgs::msg::VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position",
                qos,
                std::bind(
                    &TelemetryMonitor::position_callback,
                    this,
                    std::placeholders::_1
                )
            );

        RCLCPP_INFO(
            this->get_logger(),
            "AERIS telemetry monitor initialized."
        );
    }

private:
    void position_callback(
        const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        if (++counter_ % 50 != 0) {
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Position [NED]: X=%.2f Y=%.2f Z=%.2f | "
            "Velocity: VX=%.2f VY=%.2f VZ=%.2f | "
            "Valid=%s",
            msg->x,
            msg->y,
            msg->z,
            msg->vx,
            msg->vy,
            msg->vz,
            msg->xy_valid && msg->z_valid ? "YES" : "NO"
        );
    }

    rclcpp::Subscription<
        px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;

    uint64_t counter_{0};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<TelemetryMonitor>()
    );

    rclcpp::shutdown();

    return 0;
}

```

This first node is deliberately read-only. It cannot issue flight commands.

### Configure CMake

Replace the contents of `aeris_flight_control/CMakeLists.txt` with:

```
cmake_minimum_required(VERSION 3.8)
project(aeris_flight_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(telemetry_monitor
    src/telemetry_monitor.cpp
)

ament_target_dependencies(telemetry_monitor
    rclcpp
    px4_msgs
)

install(TARGETS telemetry_monitor
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

Build:

```
cd ~/aeris_dev/aeris-uav/ros2_ws

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

Run the node:

```
ros2 run aeris_flight_control telemetry_monitor
```

Expected output will resemble:

```
[AERIS] telemetry monitor initialized.

Position [NED]:
X=0.02 Y=-0.01 Z=-0.04
Velocity:
VX=0.00 VY=0.01 VZ=0.00
Valid=YES
```

The numbers above are illustrative, not expected exact simulator measurements.

Important: PX4 uses NED coordinates (North-East-Down), whereas ROS conventionally uses ENU (East-North-Up). We will explicitly handle these transformations when developing the navigation system.

### Milestone 1 acceptance criteria

| Test               | Expected result                       |
| ------------------ | ------------------------------------- |
| PX4 SITL           | Boots successfully                    |
| Gazebo             | X500 visible                          |
| QGroundControl     | Connects to PX4                       |
| DDS agent          | Establishes connection                |
| ROS 2 topics       | PX4 topics discoverable               |
| C++ telemetry node | Compiles and executes                 |
| Position telemetry | Received continuously                 |
| Estimator validity | Valid during normal flight conditions |

We will save the first successful run's logs and document the environment versions.

## 6. Git and project management

Initialize the project once the initial structure exists:

```
cd ~/aeris_dev/aeris-uav

git init

cat > .gitignore <<'EOF'
build/
install/
log/
__pycache__/
*.pyc
.vscode/
.idea/
results/logs/*
results/plots/*
EOF

git add .
git commit -m "Initial AERIS project structure"
```

I also want us to maintain a development journal:

`docs/experiments/development_log.md`

Record:

- What we implemented.
- What worked.
- What failed.
- Debugging decisions.
- Relevant performance measurements.

This is particularly useful when explaining your engineering decisions in interviews.

## 7. What comes after milestone 1?

## Milestone 2 — Autonomous flight controller

[Documentation | Tianhua Gao](https://images.openai.com/static-rsc-4/L5aQ10HvtljbhqfeperWrQ4LMIk0N0eQnE0fCNsTngjmFhK5cNG6yf9GIGrVtKdEs48OarBLJk30Yv8dOOS0J8RcCrrcyJTM6TnjhynxJQtzuJ1aYkFtFchU-zhtRzfeabPBgp8aXNLQT5DskUdhDjdt9Kv1gQxTbbxBHwljkZo?purpose=inline)

[tianhuagao.github.io](https://tianhuagao.github.io/documentation/?utm_source=chatgpt.com)

We will develop a C++ flight-control node capable of:

1. Checking vehicle readiness.
2. Publishing offboard-control heartbeats.
3. Requesting offboard mode.
4. Arming the vehicle.
5. Ascending to 5 metres.
6. Maintaining a position setpoint.
7. Requesting a controlled landing.

We will use PX4's `OffboardControlMode`, `TrajectorySetpoint` and `VehicleCommand` interfaces.

PX4 requires a continuous offboard setpoint stream before entering offboard mode and while remaining in it. We will implement this explicitly.&#x20;

[image](https://www.google.com/s2/favicons?domain=https://docs.px4.io\&sz=32)

PX4 Guide (main)+1



This will be our first actual autonomous flight implementation.

## 8. Your immediate next steps

I don't want you installing everything blindly. Let's first establish the state of your machine so that I can give you the correct installation commands.

Run these commands in your current terminal and send me the output:

```
echo "===== OPERATING SYSTEM ====="
cat /etc/os-release

echo "===== KERNEL ====="
uname -r

echo "===== ROS ====="
echo $ROS_DISTRO
which ros2

echo "===== PYTHON ====="
python3 --version

echo "===== GPU ====="
nvidia-smi

echo "===== DISK SPACE ====="
df -h ~

echo "===== GAZEBO ====="
gz sim --version 2>/dev/null || echo "Gazebo not installed"
```



Once I have these results, we can start setting up AERIS without disturbing your existing ROS 2 and robotics projects.

Our first tangible target is simple: a simulated X500 flying in Gazebo, connected to PX4, with our own C++ ROS 2 software monitoring its flight telemetry.