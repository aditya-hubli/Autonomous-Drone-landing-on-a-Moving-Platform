# Autonomous Drone Landing on a Moving Platform

A ROS 2 / Gazebo simulation of a quadrotor drone that autonomously detects, tracks, and lands on a moving platform using only the drone's onboard downward-facing camera. The drone uses a perception-tracking-control pipeline based on real-time ArUco marker detection to follow a platform moving in the full XY plane and execute a multi-stage precision landing.

https://github.com/aditya-hubli/Autonomous-Drone-landing-on-a-Moving-Platform/raw/main/Video.mp4

> *Demo video showing the drone detecting the platform via camera, tracking its motion in 2D, descending, and landing successfully.*

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
     /drone/camera/image_raw        /platform/odom
              |                             |
    +---------v----------+       +----------v---------+
    |   Platform         |       |   Platform Mover   |
    |   Tracker          |       |   (XY Billiard     |
    |   (ArUco + OpenCV) |       |    Bounce)         |
    +--------+-----------+       +--------------------+
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
| `mars_tracking` | ament_python | Camera image processing with OpenCV ArUco detection, camera-to-world frame conversion, velocity estimation, linear position prediction |
| `mars_drone_control` | ament_python | 6-state landing state machine (TAKEOFF → SEARCH → TRACK → DESCEND → LAND → LANDED) |
| `mars_platform` | ament_python | 2D XY billiard-ball motion controller for the landing platform |
| `mars_msgs` | ament_cmake | Custom `PlatformState` message (position, velocity, predicted position, detection confidence) |

### Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/drone/odom` | nav_msgs/Odometry | Drone pose and velocity (50 Hz) |
| `/drone/cmd_vel` | geometry_msgs/Twist | Velocity commands to drone (world frame) |
| `/drone/camera/image_raw` | sensor_msgs/Image | Downward-facing camera feed from the drone |
| `/platform/odom` | nav_msgs/Odometry | Platform pose and velocity (50 Hz) |
| `/platform/cmd_vel` | geometry_msgs/Twist | Velocity commands to platform (body frame) |
| `/platform/tracked_state` | mars_msgs/PlatformState | Tracked position, velocity, prediction, confidence |
| `/mission/state` | std_msgs/String | Current state machine phase |

---

## Landing State Machine

```
TAKEOFF ──> SEARCH ──> TRACK ──> DESCEND ──> LAND ──> LANDED
               ^          |
               └──────────┘
               (lost detection)
                    ^
                    └──── (lost during descent)
```

| State | Behavior |
|-------|----------|
| **TAKEOFF** | Climb to 5.0 m — the full ±2.5 m arena fits within camera FOV at this height |
| **SEARCH** | Hold at 5.0 m; ArUco detection is near-instant since the full arena is in frame |
| **TRACK** | Follow platform's predicted XY position at overview height; wait for stable alignment |
| **DESCEND** | Lower altitude while tracking in XY; descent rate scales with alignment quality |
| **LAND** | Final descent at 0.45 m/s with continuous XY correction |
| **LANDED** | Drone halts; platform stops once drone reaches within 0.6 m altitude |

---

## Image Processing Pipeline

Platform detection relies entirely on the drone's downward camera and OpenCV ArUco processing. No ground-truth pose information is used for detection.

1. **Camera feed** — the drone's 800×600 downward camera (120° HFOV) streams `/drone/camera/image_raw` at simulation rate.
2. **ArUco detection** — `platform_tracker` converts each frame to grayscale and runs `cv2.aruco.detectMarkers` using `DICT_4X4_50`. The platform top face carries four markers (IDs 0–3) arranged at the corners of a 1 m × 1 m green surface.
3. **Pose estimation** — `cv2.aruco.estimatePoseSingleMarkers` computes the translation vector of the first visible marker in the camera frame. Marker size is 0.2 m; the camera intrinsic matrix is matched to the SDF camera parameters (f = 600 px, cx = 400, cy = 300).
4. **Camera-to-world transform** — the camera-frame translation is rotated into the world frame using the drone's current pose from `/drone/odom`. The downward camera is body-fixed, so the axes map as: world X = drone X − cam Y, world Y = drone Y − cam X.
5. **Velocity estimation** — an exponential moving average (α = 0.7) filters the velocity computed from successive detections.
6. **Position prediction** — the tracker linearly extrapolates position 0.3 s ahead to compensate for control latency.
7. **Transient drop handling** — up to 4 consecutive missed detections are tolerated without resetting state; the last known pose is held during brief occlusions.

---

## Platform Motion

The platform moves in the full **XY horizontal plane** within a ±2.5 m bounding box. It travels at constant speed (0.5 m/s) on a straight line at a 45° diagonal. When it reaches a wall, the velocity component perpendicular to that wall is reflected, producing clean 90° rebounds (billiard-ball behavior). The starting angle is chosen randomly from the four diagonal directions. The platform stops only once the drone descends within 0.6 m of the ground, indicating physical contact is imminent.

This is a substantial change from the prior version, which moved the platform only along the 1D X-axis with a random-walk velocity profile.

---

## Key Design Decisions

- **Overview takeoff height of 5.0 m** — at this altitude the camera's 90° VFOV covers a ±5 m ground footprint, which fully contains the ±2.5 m arena. Detection is available immediately without any search sweep maneuver.
- **No sweep in SEARCH** — the drone holds position at 5.0 m; the platform is always visible in the first frames, so lateral sweeping is unnecessary and was removed.
- **6-state machine (APPROACH state removed)** — merging the old APPROACH and DESCEND stages into a single DESCEND stage simplifies the controller. Descent rate is modulated by XY alignment error instead of using a separate intermediate altitude.
- **Full 2D XY tracking** — both the platform mover and drone controller operate in the XY plane. Velocity feedforward is applied on both axes.
- **Altitude-based platform stop** — the platform mover subscribes to `/drone/odom` and stops when `drone_z < 0.6 m`, which reliably corresponds to the drone being immediately above the platform surface regardless of mission state message ordering.
- **Drone plugin operates in world frame** — velocity commands are world-aligned, simplifying the controller since body and world axes coincide at near-zero yaw.
- **Velocity feedforward** — the controller adds the estimated platform velocity (from the tracker) to the proportional XY error term, enabling smooth pursuit of a moving target without lag.

---

## Prerequisites

- **Ubuntu 22.04**
- **ROS 2 Humble**
- **Gazebo Classic 11** (comes with `ros-humble-gazebo-ros-pkgs`)

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-cv-bridge ros-humble-vision-opencv
pip3 install opencv-contrib-python
```

## Build

```bash
cd <workspace_root>
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
# Full system (Gazebo + all nodes + RViz camera feed)
ros2 launch mars_simulation mars_full_launch.py
```

The launch file handles everything in sequence:

| Time | Action |
|------|--------|
| 0 s | Kill any stale Gazebo processes |
| 2 s | Start Gazebo with the arena world |
| 8 s | Spawn landing platform at (1, 1, 0.15) |
| 12 s | Spawn drone at (0, 0, 5.0) — overview height |
| 15 s | Start platform mover (XY bounce) |
| 17 s | Start ArUco detector / platform tracker |
| 18 s | Start platform tracker state publisher |
| 19 s | Start drone controller |
| 20 s | Open RViz with drone camera feed |

### Run Components Separately

```bash
# Gazebo + models only (no autonomy)
ros2 launch mars_simulation simulation_launch.py

# Autonomy nodes only (requires Gazebo already running)
ros2 launch mars_simulation autonomy_launch.py

# Individual nodes
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

# Camera feed (requires rqt_image_view)
ros2 run rqt_image_view rqt_image_view /drone/camera/image_raw
```

---

## Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `overview_height` | 5.0 m | Takeoff and search altitude |
| `descend_speed` | 0.4 m/s | Descent rate during landing phase |
| `land_height` | 0.55 m | Altitude that triggers final LAND state |
| `xy_tolerance` | 0.2 m | Alignment threshold for state transitions |
| `max_speed` (platform) | 0.5 m/s | Platform movement speed |
| `boundary` (platform) | 2.5 m | Arena half-width for wall rebounds |
| `prediction_horizon` | 0.3 s | How far ahead the tracker predicts platform position |
| `velocity_alpha` | 0.7 | EMA coefficient for tracker velocity filter |
| `max_missed_frames` | 4 | Consecutive missed detections before resetting tracker state |

---

## ArUco Marker Generation

The platform texture is pre-generated and committed to the repository. To regenerate it:

```bash
python3 scripts/generate_markers.py
```

This writes four individual marker PNGs (`aruco_0.png` – `aruco_3.png`) and a combined `platform_top.png` (1024×1024, green background, four DICT_4X4_50 markers at the corners) into `src/mars_simulation/models/landing_platform/materials/textures/`.

---

## Project Structure

```
src/
  mars_simulation/       # Gazebo world, models, C++ drone plugin, launch files
    config/
      mars_rviz.rviz           # RViz config with drone camera display
    models/
      mars_drone/              # Quadrotor SDF with downward camera (120° HFOV)
      landing_platform/        # Platform SDF with ArUco marker texture on top face
    worlds/
      mars_world.world         # Arena with bounding walls
    src/
      drone_plugin.cpp         # Velocity-controlled drone physics (gravity comp + PD)
    launch/
      mars_full_launch.py      # Complete system launch (Gazebo + autonomy + RViz)
      simulation_launch.py     # Gazebo + models only
      autonomy_launch.py       # All autonomy nodes
  mars_tracking/         # Camera-based ArUco detection, world-frame pose, velocity/prediction
  mars_drone_control/    # 6-state landing state machine + 2D trajectory control
  mars_platform/         # XY billiard-ball platform motion controller
  mars_msgs/             # Custom PlatformState message definition
scripts/
  generate_markers.py    # Regenerates ArUco marker PNGs and platform top texture
```

---
