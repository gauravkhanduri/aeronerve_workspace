# Nav2 Drone Integration — `aero_nav2`

> **ROS 2 Humble | Ardupilot + MAVROS + Gazebo SITL | Open Navigation Nav2**

---

## Overview

This document describes the `aero_nav2` package — a dedicated Nav2 integration layer for the `aero_ws` drone workspace. It connects the Open Navigation (Nav2) motion planning stack to the existing PX4/MAVROS drone control pipeline **without modifying** the `odom_vision` package.

### Why a separate package?

`odom_vision` handles sensor bridging, odometry, and custom planners (A\*, APF, Hybrid). `aero_nav2` adds full Nav2 mission planning on top, keeping concerns separated and both packages independently usable.

---

## Architecture

```text
┌─────────────────────────────────────────────────────────────┐
│                    aero_ws (ROS 2 Humble)                   │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  odom_vision (existing)                              │   │
│  │  ┌──────────────┐  ┌───────────────┐  ┌──────────┐  │   │
│  │  │ MAVROS bridge│  │  tf_publisher │  │depth_dist│  │   │
│  │  │ odom→vision  │  │  map→odom→    │  │  node    │  │   │
│  │  │  pose        │  │  base_link    │  │          │  │   │
│  │  └──────────────┘  └───────────────┘  └──────────┘  │   │
│  └──────────────────────────────────────────────────────┘   │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  aero_nav2 (new)                                     │   │
│  │                                                      │   │
│  │  /depth_camera ──► depth_to_costmap ──► /depth_      │   │
│  │  (Image 32FC1)      (pinhole proj         pointcloud │   │
│  │                      + TF to map)      (PointCloud2) │   │
│  │                                              │        │   │
│  │  Nav2 Stack:                                 │        │   │
│  │  bt_navigator ◄── nav2_mission_client        │        │   │
│  │      │                                       ▼        │   │
│  │  planner_server ─────────────► VoxelLayer costmap    │   │
│  │  (SmacHybrid A*)               ObstacleLayer          │   │
│  │      │ nav_msgs/Path                                  │   │
│  │  controller_server                                    │   │
│  │  (RegulatedPurePursuit)                               │   │
│  │      │ /cmd_vel (Twist)                               │   │
│  │  nav2_mavros_bridge                                   │   │
│  │  (velocity → position)                                │   │
│  │      │ /mavros/setpoint_position/local                │   │
│  └──────┼───────────────────────────────────────────────┘   │
│         ▼                                                   │
│      PX4 OFFBOARD flight controller                        │
└─────────────────────────────────────────────────────────────┘
```

---

## Package Structure

```text
src/aero_nav2/
├── aero_nav2/
│   ├── __init__.py
│   ├── depth_to_costmap.py      # /depth_camera → /depth_pointcloud
│   ├── nav2_mavros_bridge.py    # /cmd_vel → /mavros/setpoint_position/local
│   └── nav2_mission_client.py   # nav2_simple_commander interface
├── config/
│   └── nav2_params.yaml         # All Nav2 node parameters
├── launch/
│   └── nav2_launch.py           # Launches Nav2 + bridge nodes
├── resource/aero_nav2
├── package.xml
├── setup.cfg
└── setup.py
```

---

## Node Reference

### `depth_to_costmap`

| Property | Value |
| --- | --- |
| **Purpose** | Converts raw depth image to PointCloud2 for Nav2 costmaps |
| **Sub** | `/depth_camera` (sensor_msgs/Image, 32FC1) |
| **Pub** | `/depth_pointcloud` (sensor_msgs/PointCloud2, frame=`map`) |
| **TF needed** | `camera_link` → `map` |

Key parameters:

```yaml
image_width: 640
image_height: 480
hfov_deg: 90.0
downsample_factor: 8   # stride — 1/64 of pixels used; tune for perf
min_depth: 0.1
max_depth: 8.0
```

### `nav2_mavros_bridge`

| Property | Value |
| --- | --- |
| **Purpose** | Integrates Nav2 velocity commands into MAVROS position setpoints |
| **Sub** | `/cmd_vel` (geometry_msgs/Twist), `/mavros/local_position/pose` |
| **Pub** | `/mavros/setpoint_position/local` (geometry_msgs/PoseStamped) @ 20 Hz |

Key parameters:

```yaml
publish_rate: 20.0      # Hz — must be >2 Hz for PX4 OFFBOARD
max_step_size: 0.5      # m  — safety clamp per timestep
yaw_from_velocity: true # derive heading from velocity direction
cmd_vel_timeout: 0.5    # s  — hold position if cmd_vel goes stale
```

> **Note:** 20 Hz publishing continues even when Nav2 is idle, satisfying PX4's OFFBOARD heartbeat requirement.

### `nav2_mission_client`

| Property | Value |
| --- | --- |
| **Purpose** | High-level mission interface; sends NavigateToPose to bt_navigator |
| **Uses** | `nav2_simple_commander.BasicNavigator` |

Parameters:

```yaml
goal_x: 5.0   # map frame, metres
goal_y: 0.0
goal_z: 3.0   # altitude
goal_yaw: 0.0 # radians
```

---

## Nav2 Configuration Summary (`nav2_params.yaml`)

| Component | Plugin / Setting |
| --- | --- |
| Global planner | `nav2_smac_planner/SmacPlannerHybrid` |
| Local controller | `nav2_regulated_pure_pursuit_controller/RegulatedPurePursuitController` |
| Local costmap | `VoxelLayer` + `InflationLayer` (10 m × 10 m rolling, 0.2 m/cell) |
| Global costmap | `ObstacleLayer` + `InflationLayer` (200 m × 200 m, 0.2 m/cell) |
| Obstacle source | `/depth_pointcloud` (PointCloud2) |
| Voxel height | 4 m (20 voxels × 0.2 m) |
| Goal tolerance | 0.5 m XY, 0.5 rad yaw |
| Cruise speed | 2.0 m/s |
| Lookahead | 2.0 m (dynamic scaling on) |
| Localisation | No AMCL — uses existing TF from `tf_publisher.py` |

---

## TF Tree

```text
map  (static)
 └── odom              ← tf_publisher.py (identity to map for Gazebo SITL)
      └── base_link    ← tf_publisher.py (from Gazebo /odom)
           └── camera_link  ← tf_publisher.py (0.08 m forward, horizontal)
```

> `camera_link` is configured as **forward-facing (horizontal)** in `tf_publisher.py`: `position=(0.08, 0, -0.05)`, `rotation=identity (w=1.0)`. This ensures the depth costmap sees obstacles in the flight path.

---

## Topics Summary

| Topic | Type | Direction | Node |
| --- | --- | --- | --- |
| `/depth_camera` | Image (32FC1) | in | `depth_to_costmap` |
| `/depth_pointcloud` | PointCloud2 | out | `depth_to_costmap` |
| `/mavros/local_position/pose` | PoseStamped | in | `nav2_mavros_bridge` |
| `/cmd_vel` | Twist | in | `nav2_mavros_bridge` ← Nav2 |
| `/mavros/setpoint_position/local` | PoseStamped | out | `nav2_mavros_bridge` |
| `/odom` | Odometry | in | Nav2 bt_navigator |
| `/local_costmap/costmap` | OccupancyGrid | out | Nav2 |
| `/global_costmap/costmap` | OccupancyGrid | out | Nav2 |
| `/plan` | Path | out | Nav2 planner_server |

---

## Prerequisites Before Running

1. **PX4 SITL + Gazebo running** with the drone model loaded
2. **Drone armed and hovering** at target altitude in OFFBOARD mode
   - `nav2_mavros_bridge` publishes 20 Hz setpoints (OFFBOARD heartbeat)
   - Must arm and switch to OFFBOARD before launching the Nav2 mission
3. **`odom_vision` stack running** (TF, MAVROS bridge, Gazebo bridge)

---

## Launch Instructions

```bash
# Terminal 0: Source workspace
source /home/grv22/aero_ws/install/setup.bash

# Terminal 1: Start PX4 SITL + Gazebo (your simulation command here)
# e.g. make px4_sitl gazebo

# Terminal 2: Start base drone stack
ros2 launch odom_vision pose.launch.py

# Terminal 3: Start Nav2 stack
ros2 launch aero_nav2 nav2_launch.py

# Terminal 4: Send a navigation goal (once drone is hovering in OFFBOARD)
ros2 run aero_nav2 nav2_mission_client \
  --ros-args -p goal_x:=5.0 -p goal_y:=0.0 -p goal_z:=3.0
```

---

## Build

```bash
cd /home/grv22/aero_ws
colcon build --symlink-install --packages-select aero_nav2
source install/setup.bash
```

To build the whole workspace:

```bash
colcon build --symlink-install
```

---

## Verification Checklist

```bash
# 1. Nav2 lifecycle nodes active
ros2 lifecycle list controller_server    # expect: active
ros2 lifecycle list planner_server       # expect: active
ros2 lifecycle list bt_navigator         # expect: active

# 2. Sensor bridge publishing
ros2 topic hz /depth_pointcloud          # expect: ~10-30 Hz

# 3. Costmaps populated
ros2 topic hz /local_costmap/costmap     # expect: ~2 Hz
ros2 topic hz /global_costmap/costmap    # expect: ~0.5 Hz

# 4. MAVROS bridge active
ros2 topic hz /mavros/setpoint_position/local  # expect: 20 Hz

# 5. TF tree complete
ros2 run tf2_tools view_frames
# map → odom → base_link → camera_link must all be connected

# 6. During active navigation
ros2 topic hz /cmd_vel                   # expect: ~20 Hz
ros2 topic echo /plan --no-arr --once    # expect: non-empty path
```

---

## Known Limitations

| Limitation | Detail | Workaround |
| --- | --- | --- |
| No true 3D planner | Nav2 Humble has no volumetric 3D planner plugin | VoxelLayer gives 3D obstacle awareness; altitude tracked via bridge |
| Controller is C++ only | Nav2 controller plugins must be C++ | Using RegulatedPurePursuit + Python bridge node |
| OFFBOARD prerequisite | PX4 must be armed and in OFFBOARD before Nav2 missions | Arm/takeoff manually or via a separate arming script |
| 2D costmap | Global/local costmaps are 2D projections | VoxelLayer captures 3D obstacles but plans in 2D |

---

## File Index

| File | Purpose |
| --- | --- |
| `aero_nav2/depth_to_costmap.py` | Depth image → PointCloud2 sensor bridge |
| `aero_nav2/nav2_mavros_bridge.py` | cmd_vel → MAVROS position setpoints |
| `aero_nav2/nav2_mission_client.py` | NavigateToPose mission interface |
| `config/nav2_params.yaml` | All Nav2 node parameters |
| `launch/nav2_launch.py` | Nav2 stack launch file |
| `odom_vision/launch/pose.launch.py` | Updated: adds `depth_distance` node |
