# AeroNerve Workspace

ROS 2 Humble workspace for autonomous drone navigation using visual-inertial odometry (ZED 2i), OctoMap 3D mapping, Nav2 path planning, and MAVROS flight control. Designed for GPS-denied ISR (Intelligence, Surveillance, Reconnaissance) missions on ArduPilot/PX4 hardware.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Architecture & Data Flow](#architecture--data-flow)
3. [Package Structure](#package-structure)
4. [Prerequisites](#prerequisites)
5. [Setup & Installation](#setup--installation)
6. [Docker Setup](#docker-setup)
7. [Configuration](#configuration)
8. [Launch Guide](#launch-guide)
9. [Nodes Reference](#nodes-reference)
10. [Mission Files](#mission-files)
11. [Health Check](#health-check)
12. [CI Pipeline](#ci-pipeline)

---

## System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        AeroNerve Stack                          │
│                                                                 │
│   ZED 2i Camera                                                 │
│   ├── VIO Pose  ──────────────────► PX4/ArduPilot EKF          │
│   └── Point Cloud ─► OctoMap ─► Nav2 Costmap                   │
│                                      │                          │
│                              Global Planner (A*)                │
│                                      │                          │
│                           Local Planner (MPPI/DWA)              │
│                                      │                          │
│                            cmd_vel_bridge                        │
│                                      │                          │
│                       MAVROS ──► Flight Controller              │
│                                                                 │
│   mission_sequencer ──► Nav2 Action Client                      │
│   drone_state_broadcaster ──► GCS (UDP)                         │
└─────────────────────────────────────────────────────────────────┘
```

---

## Architecture & Data Flow

### Sensor → Estimation
1. **ZED 2i** publishes point cloud (`/zed/zed_node/point_cloud/cloud_registered`) and VIO pose (`/zed/zed_node/pose`)
2. **vision_pose_relay** bridges ZED VIO pose → `/mavros/vision_pose/pose`
3. **PX4/ArduPilot EKF** fuses vision pose as GPS replacement

### Mapping
4. **OctoMap server** consumes point cloud → publishes 3D occupancy map → projected 2D `/projected_map`
5. **odom_qos_relay** re-publishes `/mavros/local_position/odom` with RELIABLE QoS so Nav2 can consume it

### Planning & Execution
6. **Nav2 stack** uses `/projected_map` for costmap, runs A* global planner + MPPI local planner
7. **mission_sequencer** reads a JSON mission file (or accepts via TCP) and sends goals to Nav2 `NavigateToPose` action
8. **cmd_vel_bridge** translates Nav2 `/cmd_vel` → MAVROS `/mavros/setpoint_raw/local` (velocity setpoints)

### Monitoring
9. **drone_state_broadcaster** streams pose + flight mode to GCS over UDP at 20 Hz
10. **health_check** verifies all critical topics are alive on startup

---

## Package Structure

```
aeronerve_workspace/
├── src/
│   └── odom_vision/                        # Main package
│       ├── config/
│       │   ├── nav2_drone.yaml             # Nav2 parameters (MPPI + NavFn A*)
│       │   └── drone_nav_bt.xml            # Behavior tree for navigation
│       ├── launch/
│       │   ├── full_system.launch.py       # Full bringup (ZED + OctoMap + MAVROS)
│       │   ├── drone.launch.py             # Minimal drone bringup
│       │   ├── nav2_drone.launch.py        # Nav2 stack launch
│       │   ├── octomap_mapping.launch.py   # OctoMap server only
│       │   ├── planner_executor.launch.py  # Planner + mission executor
│       │   ├── pose.launch.py              # Pose/VIO bridge only
│       │   └── health_check.py             # Health check launcher
│       └── odom_vision/
│           ├── mission_sequencer.py        # Mission runner (file or TCP)
│           ├── mission_executor_node.py    # Action-based mission executor
│           ├── global_planner_node.py      # A* global planner wrapper
│           ├── reactive_nav_node.py        # DWA reactive local planner
│           ├── reactive_nav_node_updated.py
│           ├── cmd_vel_bridge.py           # cmd_vel → MAVROS setpoint bridge
│           ├── odom_qos_relay.py           # Odom QoS relay (BEST_EFFORT → RELIABLE)
│           ├── drone_state_broadcaster.py  # UDP state stream to GCS
│           ├── health_check.py             # System health verifier
│           ├── send_move.py                # Utility: send velocity commands
│           ├── send_forward_goal.py        # Utility: send single Nav2 goal
│           └── test_send_goal.py           # Test goal sender
├── Dockerfile
├── ros_entrypoint.sh
└── .github/workflows/ros_ci.yml
```

---

## Prerequisites

| Requirement | Version |
|---|---|
| Ubuntu | 22.04 (Jammy) |
| ROS 2 | Humble |
| Python | 3.10+ |
| ZED SDK | 4.x |
| ArduPilot / PX4 | Any MAVLink-compatible firmware |

**ROS 2 packages required:**

```bash
sudo apt install -y \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  ros-humble-nav2-bringup \
  ros-humble-octomap-server \
  ros-humble-tf2-ros \
  ros-humble-topic-tools \
  python3-colcon-common-extensions \
  python3-rosdep
```

**Python dependencies:**

```bash
pip3 install numpy scipy
```

---

## Setup & Installation

### 1. Clone the repository

```bash
git clone git@github.com:gauravkhanduri/aeronerve_workspace.git
cd aeronerve_workspace
```

### 2. Initialize rosdep

```bash
sudo rosdep init        # skip if already done
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro humble
```

### 3. Build the workspace

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 4. Source the overlay

```bash
source install/setup.bash
```

Add to `~/.bashrc` to make it persistent:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
```

---

## Docker Setup

The provided Dockerfile builds the full workspace on top of `ros:humble-ros-base-jammy` and works on both x86 and ARM64 (Jetson).

### Build the image

```bash
cd aeronerve_workspace
docker build -t aeronerve:humble .
```

### Run a container shell

```bash
docker run -it --rm \
  --network host \
  aeronerve:humble bash
```

### Verify the package built correctly

```bash
# Inside the container
ros2 pkg list | grep odom_vision
```

### Run with device access (hardware serial/USB)

```bash
docker run -it --rm \
  --network host \
  --privileged \
  -v /dev:/dev \
  aeronerve:humble bash
```

---

## Configuration

### FCU connection (`fcu_url`)

| Hardware | Value |
|---|---|
| Jetson UART | `/dev/ttyTHS1:921600` |
| USB (Pixhawk) | `/dev/ttyACM0:57600` |
| SITL (UDP) | `udp://:14540@` |

### PX4 EKF parameters (for GPS-denied VIO)

Set these on the flight controller before flight:

```
EKF2_EV_CTRL  = 15   # fuse vision position + velocity + yaw
EKF2_HGT_REF  = 3    # use vision as height reference
EKF2_GPS_CTRL = 0    # disable GPS fusion
EKF2_EV_DELAY = 0    # adjust to match actual latency
```

### ArduPilot EKF parameters

```
VISO_TYPE       = 1   # MAVLink vision
EK3_SRC1_POSXY  = 6   # ExternalNav
EK3_SRC1_VELXY  = 6   # ExternalNav
EK3_SRC1_POSZ   = 6   # ExternalNav
EK3_SRC1_YAW    = 6   # ExternalNav
```

### Camera TF offset

Edit `full_system.launch.py` — the `static_tf_base_to_camera` node arguments set the physical offset from `base_link` to `zed_camera_link`:

```python
arguments=['0.05', '0.0', '-0.03', '0', '0', '0', '1', 'base_link', 'zed_camera_link']
#           x_fwd   y_lft  z_up    qx   qy   qz   qw
```

---

## Launch Guide

### Full system bringup (ZED + OctoMap + MAVROS + VIO bridge)

```bash
ros2 launch odom_vision full_system.launch.py
```

With custom arguments:

```bash
ros2 launch odom_vision full_system.launch.py \
  fcu_url:=/dev/ttyACM0:57600 \
  flight_altitude:=8.0 \
  octomap_resolution:=0.3
```

### Nav2 stack only

```bash
ros2 launch odom_vision nav2_drone.launch.py
```

### OctoMap mapping only

```bash
ros2 launch odom_vision octomap_mapping.launch.py
```

### Mission planner + executor

```bash
ros2 launch odom_vision planner_executor.launch.py
```

### Recommended startup order

```
Step 1 — Hardware bringup
  ros2 launch odom_vision full_system.launch.py

Step 2 — Navigation stack
  ros2 launch odom_vision nav2_drone.launch.py

Step 3 — Verify system health
  ros2 run odom_vision health_check

Step 4 — Start mission
  ros2 launch odom_vision planner_executor.launch.py
```

---

## Nodes Reference

| Node | Executable | Description |
|---|---|---|
| `MissionSequencer` | `mission_sequencer` | Reads JSON mission, arms drone, sends Nav2 goals sequentially |
| `MissionExecutorNode` | `mission_executor_node` | Action-server-based mission execution |
| `GlobalPlannerNode` | `global_planner_node` | A* global path planner wrapper |
| `ReactiveNavNode` | `reactive_nav_node` | DWA local planner with obstacle avoidance |
| `CmdVelBridge` | `cmd_vel_bridge` | Converts `/cmd_vel` → `/mavros/setpoint_raw/local` |
| `OdomQosRelay` | `odom_qos_relay` | Relays odom with RELIABLE QoS for Nav2 compatibility |
| `DroneStateBroadcaster` | `drone_state_broadcaster` | Streams pose + state to GCS via UDP at 20 Hz |
| `HealthCheckNode` | `health_check` | Verifies critical topics are publishing on startup |

### Key topics

| Topic | Type | Direction |
|---|---|---|
| `/mavros/local_position/pose` | `PoseStamped` | Drone pose (EKF output) |
| `/mavros/vision_pose/pose` | `PoseStamped` | VIO input to EKF |
| `/zed/zed_node/point_cloud/cloud_registered` | `PointCloud2` | ZED point cloud |
| `/projected_map` | `OccupancyGrid` | 2D costmap from OctoMap |
| `/cmd_vel` | `Twist` | Nav2 velocity commands |
| `/mavros/setpoint_raw/local` | `PositionTarget` | MAVROS velocity setpoints |
| `/mavros/state` | `State` | FCU armed/mode status |

---

## Mission Files

Mission files are JSON arrays of waypoints. Each waypoint requires `x`, `y`, `z` (meters, local frame) and optionally `yaw_deg`:

```json
[
  { "x": 0.0,  "y": 0.0,  "z": 5.0, "yaw_deg": 0.0 },
  { "x": 10.0, "y": 0.0,  "z": 5.0, "yaw_deg": 90.0 },
  { "x": 10.0, "y": 10.0, "z": 5.0, "yaw_deg": 180.0 }
]
```

### Run from a file

```bash
ros2 run odom_vision mission_sequencer \
  --ros-args -p mission_file:=/path/to/mission.json
```

### Run via TCP (upload from GCS at runtime)

```bash
ros2 run odom_vision mission_sequencer \
  --ros-args -p tcp_port:=9000
```

Then send the JSON payload from your GCS over TCP to port 9000.

### Key mission parameters

| Parameter | Default | Description |
|---|---|---|
| `arrival_radius` | `1.0 m` | Waypoints closer than this skip Nav2 (yaw + stabilize only) |
| `altitude_tolerance` | `0.3 m` | Altitude deadband before proceeding to next waypoint |
| `stabilize_s` | `2.0 s` | Hold time at each waypoint |
| `precision_radius` | `0.3 m` | Tight arrival radius for precision hold/landing |
| `step_hz` | `10.0 Hz` | Mission loop rate |

---

## Health Check

After launching the full system, run the health check to verify all critical topics are active:

```bash
ros2 run odom_vision health_check
```

Checks performed:
- ZED point cloud publishing
- MAVROS FCU connected
- OctoMap `/projected_map` updating
- Odometry publishing
- VIO pose bridge active

The node waits up to 10 seconds per topic before reporting a failure.

---

## CI Pipeline

GitHub Actions runs automatically on every push and pull request to `main`:

1. Spins up `ros:humble-ros-base-jammy` container
2. Installs all `package.xml` dependencies via `rosdep`
3. Builds with `colcon build`
4. Runs `colcon test`

Add this badge to display live build status:

```markdown
![CI](https://github.com/gauravkhanduri/aeronerve_workspace/actions/workflows/ros_ci.yml/badge.svg)
```
