# Autonomous Drone Landing on a Moving Platform

A ROS 2 / Gazebo simulation of a quadrotor drone that autonomously detects, tracks, and lands on a moving ground platform. The drone uses a perception-tracking-control pipeline to follow a randomly moving platform and execute a multi-stage precision landing.

https://github.com/aditya-hubli/Autonomous-Drone-landing-on-a-Moving-Platform/raw/main/video.mp4

> *Demo video showing the drone detecting the platform, tracking its motion, descending, and landing successfully.*

---

## System Architecture

```
                    +-----------------+
                    |     Gazebo      |
                    |   Simulation    |
                    +--------+--------+
                             |
              +--------------+--------------+
              |                             |
     /drone/odom                   /platform/odom
              |                             |
    +---------v----------+       +----------v---------+
    |   Platform         |       |   Platform Mover   |
    |   Detector         |       |   (Random Walk)    |
    +--------+-----------+       +--------------------+
             |
      /aruco/detection
             |
    +--------v-----------+
    |   Platform         |
    |   Tracker          |
    +--------+-----------+
             |
    /platform/tracked_state
             |
    +--------v-----------+
    |   Drone            |
    |   Controller       |
    +--------+-----------+
             |
       /drone/cmd_vel
             |
    +--------v-----------+
    |   Drone Plugin     |
    |   (Gazebo C++)     |
    +--------------------+
```

### ROS 2 Packages

| Package | Type | Description |
|---------|------|-------------|
| `mars_simulation` | ament_cmake | Gazebo world, drone/platform SDF models, C++ drone velocity-control plugin |
| `mars_perception` | ament_python | Platform detection using odometry-based simulation of camera FOV |
| `mars_tracking` | ament_python | Camera-to-world frame conversion, velocity estimation, linear position prediction |
| `mars_drone_control` | ament_python | 7-state landing state machine (TAKEOFF -> SEARCH -> TRACK -> APPROACH -> DESCEND -> LAND -> LANDED) |
| `mars_platform` | ament_python | Random-walk motion controller for the landing platform |
| `mars_msgs` | ament_cmake | Custom `PlatformState` message (position, velocity, predicted position, detection confidence) |

### Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/drone/odom` | nav_msgs/Odometry | Drone pose and velocity (50 Hz) |
| `/drone/cmd_vel` | geometry_msgs/Twist | Velocity commands to drone (world frame) |
| `/platform/odom` | nav_msgs/Odometry | Platform pose and velocity (50 Hz) |
| `/platform/cmd_vel` | geometry_msgs/Twist | Velocity commands to platform (body frame) |
| `/aruco/detection` | geometry_msgs/PoseStamped | Detected platform pose in camera frame |
| `/platform/tracked_state` | mars_msgs/PlatformState | Tracked position, velocity, prediction, confidence |
| `/mission/state` | std_msgs/String | Current state machine phase |

---

## Landing State Machine

```
TAKEOFF ──> SEARCH ──> TRACK ──> APPROACH ──> DESCEND ──> LAND ──> LANDED
               ^          |         |            |
               └──────────┘         |            |
               (lost detection)     |            |
               ^                    |            |
               └────────────────────┘            |
               (lost during approach)            |
                          ^                      |
                          └──────────────────────┘
                          (lost during descent)
```

| State | Behavior |
|-------|----------|
| **TAKEOFF** | Climb to 3.0m hover height |
| **SEARCH** | Sweep back and forth along X-axis (+-2.5m) looking for platform |
| **TRACK** | Follow platform's predicted position at hover height, wait for alignment |
| **APPROACH** | Descend to 2.0m while maintaining XY tracking |
| **DESCEND** | Lower to 0.55m while keeping alignment within 0.2m tolerance |
| **LAND** | Final descent at 0.8 m/s until touchdown |
| **LANDED** | Motors off, success message displayed, platform stops |

---

## Key Design Decisions

- **Drone plugin operates in world frame** - velocity commands are world-aligned (not body frame), simplifying control since the drone maintains level attitude
- **1D tracking problem** - platform moves along X-axis only; drone holds Y = 0, reducing complexity while still demonstrating the full tracking pipeline
- **Velocity feedforward** - controller adds estimated platform velocity to proportional tracking, enabling smooth pursuit of a moving target
- **Linear prediction** - tracker extrapolates platform position 1.0s ahead using velocity estimate from a sliding window of recent detections
- **Ground-truth detection** - uses odometry with simulated camera FOV constraints instead of image processing, avoiding Gazebo Classic camera rendering limitations while maintaining the same perception pipeline interface

---

## Prerequisites

- **Ubuntu 22.04**
- **ROS 2 Humble**
- **Gazebo Classic 11** (comes with `ros-humble-gazebo-ros-pkgs`)

```bash
# Install dependencies
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-cv-bridge ros-humble-vision-opencv
```

## Build

```bash
cd <workspace_root>
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
# Full system (Gazebo + all nodes with timed sequencing)
ros2 launch mars_simulation mars_full_launch.py
```

The launch file handles everything:
1. Starts Gazebo with the arena world
2. Spawns the landing platform (bright green, 1m x 1m)
3. Spawns the drone at 3m altitude
4. Starts platform mover, detector, tracker, and controller in sequence

### Run Components Separately

```bash
# Gazebo + models only (no autonomy)
ros2 launch mars_simulation simulation_launch.py

# Autonomy nodes only (requires Gazebo already running)
ros2 launch mars_simulation autonomy_launch.py

# Individual nodes
ros2 launch mars_perception perception_launch.py
ros2 launch mars_tracking tracking_launch.py
ros2 launch mars_drone_control control_launch.py
ros2 launch mars_platform platform_launch.py
```

## Monitor

```bash
# Watch state transitions
ros2 topic echo /mission/state

# Platform tracking data
ros2 topic echo /platform/tracked_state

# Drone position
ros2 topic echo /drone/odom
```

---

## Configuration

Key parameters can be tuned via the launch file or command line:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `hover_height` | 3.0 m | Search/takeoff altitude |
| `approach_height` | 2.0 m | Altitude during approach phase |
| `descend_speed` | 0.8 m/s | Descent rate during landing |
| `land_height` | 0.55 m | Height to trigger final landing |
| `xy_tolerance` | 0.2 m | Alignment threshold for state transitions |
| `max_speed` (platform) | 0.5 m/s | Platform movement speed |

---

## Project Structure

```
src/
  mars_simulation/       # Gazebo world, models, C++ drone plugin, launch files
    models/
      mars_drone/        # Quadrotor SDF with downward camera
      landing_platform/  # Moving platform SDF with ArUco marker texture
    worlds/
      mars_world.world   # Arena with invisible corridor walls
    src/
      drone_plugin.cpp   # Velocity-controlled drone physics (gravity comp + PD control)
    launch/
      mars_full_launch.py      # Complete system launch
      simulation_launch.py     # Gazebo + models only
      autonomy_launch.py       # All autonomy nodes
  mars_perception/       # Platform detection node
  mars_tracking/         # Position tracking + velocity estimation + prediction
  mars_drone_control/    # Landing state machine + trajectory control
  mars_platform/         # Random-walk platform motion controller
  mars_msgs/             # Custom PlatformState message definition
```

---

## License

MIT
