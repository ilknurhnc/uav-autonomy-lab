# UAV Autonomy Lab

Autonomous UAV software project built with **PX4 SITL, Gazebo, MAVSDK, Python, OpenCV and 2D LiDAR**.

The project is developed incrementally to understand the complete autonomy pipeline:

```text
Perception
   ↓
Mapping
   ↓
Decision
   ↓
Navigation
   ↓
Control
```

The current goal is an autonomous UAV that can detect unknown obstacles, build a simple local map, inspect objects and visually locate a target.

---

## Current Mission

```text
TAKEOFF
   ↓
BUILD_MAP
   ↓
SELECT_OBJECT
   ↓
MOVE_TO_VIEWPOINT
   ↓
SEARCH_TARGET
   ├── Target not found → NEXT_OBJECT
   │                         ↓
   │                   SELECT_OBJECT
   │
   └── Target found → TRACK
                         ↓
                     CENTERED
```

The red target may initially be hidden behind an obstacle.

The UAV must use LiDAR to understand the environment and inspect detected objects until the camera finds the target.

---

## Current System

Implemented and tested:

- PX4 SITL + Gazebo simulation
- X500 multicopter
- MAVSDK telemetry and Offboard Control
- Arm / Takeoff / Land
- Velocity and yaw control
- Gazebo camera → Python pipeline
- OpenCV red target detection
- Vision-guided target tracking
- `SEARCH → TRACK → CENTERED`
- 2D LiDAR integration
- LaserScan filtering
- Multi-obstacle clustering
- Sensor → Local NED coordinate transformation
- Persistent `ObstacleMap`
- Tentative / confirmed obstacle observations
- LiDAR-based inspection flight
- Custom X500 with **camera + 2D LiDAR on the same vehicle**

---

## Vision-Guided Tracking

Camera frames are processed with OpenCV.

```text
Gazebo Camera
      ↓
OpenCV
      ↓
HSV Red Detection
      ↓
Contour
      ↓
Target Center
      ↓
error_x
      ↓
P Controller
      ↓
yaw_speed
      ↓
MAVSDK Offboard
      ↓
PX4
```

Horizontal error:

```text
error_x = target_x - image_center_x
```

The controller converts this error into yaw speed.

When the target enters the deadband:

```text
CENTERED
```

is reached.

This closed-loop behavior has been validated in Gazebo.

---

## LiDAR Obstacle Detection

The 2D LiDAR pipeline:

```text
LaserScan
   ↓
Filter invalid measurements
   ↓
distance + angle + scan index
   ↓
Scan segmentation
   ↓
Obstacle clusters
```

Invalid measurements are filtered:

```text
inf
nan
distance <= 0
```

Initial clustering parameters:

```python
DISTANCE_JUMP_THRESHOLD = 1.0
INDEX_GAP_THRESHOLD = 2
MIN_CLUSTER_POINTS = 3
```

Distance continuity and scan-index continuity allow different obstacles at similar distances to remain separate.

---

## Coordinate Transformation

LiDAR initially reports obstacle positions relative to the UAV.

These measurements are transformed into PX4 local NED coordinates.

```text
LiDAR measurement
      ↓
Sensor coordinates
      ↓
Body frame
      ↓
Drone yaw
      ↓
Local NED
      ↓
Obstacle position
```

Coordinate assumptions:

```text
Sensor / FLU-like
+X = forward
+Y = left

PX4 body / FRD
+X = forward
+Y = right

PX4 Local NED
+X = North
+Y = East
+Z = Down
```

This allows obstacle observations from different UAV positions to be compared in the same local coordinate system.

---

## Obstacle Map

`ObstacleMap` associates repeated LiDAR observations with previously detected objects.

```text
New observation
      ↓
Find nearest mapped object
      ↓
Within merge distance?
   /             \
 YES              NO
  ↓                ↓
Update object    Create object
  ↓                ↓
Observation++    TENTATIVE
  ↓
3 observations
  ↓
CONFIRMED
```

Current confirmation threshold:

```python
MIN_CONFIRMATION_OBSERVATIONS = 3
```

This prevents single noisy LiDAR measurements from immediately becoming trusted map objects.

Current data association is intentionally simple and based mainly on centroid distance.

Long surfaces can still produce duplicate objects when their visible centroid moves between viewpoints. More geometric association is planned later.

---

## LiDAR Inspection

The UAV can currently:

```text
Takeoff
   ↓
Build confirmed obstacle map
   ↓
Select suitable obstacle
   ↓
Calculate safe inspection point
   ↓
Fly toward viewpoint
   ↓
Hover
   ↓
Land
```

A safety distance is maintained instead of flying directly onto an obstacle.

This module will now be combined with visual target detection.

---

## Custom Camera + LiDAR X500

PX4 provides camera and LiDAR X500 simulation models separately.

For this project a custom model was created:

```text
x500_lidar_camera
│
├── X500 base
├── Mono Camera
└── 2D LiDAR
```

The model was created using Gazebo SDF includes and fixed joints.

The custom model required:

```text
SDF model
   ↓
PX4 airframe configuration
   ↓
CMake airframe registration
   ↓
CMake configuration
   ↓
Ninja build
   ↓
PX4 SITL
```

Custom PX4 airframe:

```text
4015_gz_x500_lidar_camera
```

Simulation target:

```bash
PX4_GZ_WORLD=vision_test make px4_sitl gz_x500_lidar_camera
```

The camera and LiDAR now operate on the **same simulated UAV**.

---

## Simulation Launcher

The project includes:

```bash
./scripts/start_simulation.sh
```

The launcher is being updated to start:

```text
vision_test world
      ↓
PX4 SITL
      ↓
x500_lidar_camera
      ↓
Gazebo GUI
   /       \
Camera    LiDAR
```

Stop script:

```bash
./scripts/stop_simulation.sh
```

---

## Project Structure

```text
uav-autonomy-lab/
│
├── companion/
│   ├── autonomy/
│   │   ├── motion_controller.py
│   │   ├── coordinate_transform.py
│   │   └── obstacle_map.py
│   │
│   ├── missions/
│   │   ├── takeoff_land.py
│   │   ├── offboard_forward.py
│   │   ├── offboard_square.py
│   │   ├── vision_tracking.py
│   │   └── lidar_inspection.py
│   │
│   ├── telemetry/
│   │   ├── telemetry_monitor.py
│   │   ├── local_pose_monitor.py
│   │   └── obstacle_local_monitor.py
│   │
│   └── vision/
│       ├── camera_viewer.py
│       └── lidar_viewer.py
│
├── simulation/
│   └── worlds/
│       └── vision_test.sdf
│
├── scripts/
│   ├── start_simulation.sh
│   └── stop_simulation.sh
│
├── docs/
│   ├── architecture.md
│   └── troubleshooting.md
│
└── requirements.txt
```

---

## Current Development Stage

Completed:

```text
Vision Tracking              ✓
SEARCH / TRACK / CENTERED    ✓
2D LiDAR                     ✓
Obstacle Clustering          ✓
Coordinate Transformation    ✓
Local Obstacle Map           ✓
Observation Confirmation     ✓
LiDAR Inspection Flight      ✓
Camera + LiDAR X500          ✓
```

Current work:

```text
Camera + LiDAR
      ↓
Unified Mission State Machine
      ↓
Object-by-object inspection
      ↓
Red target search
      ↓
TRACK
      ↓
CENTERED
```

Next stages:

```text
Dynamic viewpoints
Obstacle-aware inspection
Improved data association
Vehicle detection
Unknown-area exploration
ROS2 integration
```

---

## Technologies

- C / C++
- Python
- PX4
- Gazebo
- MAVSDK
- OpenCV
- 2D LiDAR
- CMake
- Ninja
- Git
- Linux / Unix

---

## Engineering Approach

Development follows:

```text
Understand
   ↓
Implement
   ↓
Test
   ↓
Debug
   ↓
Validate
   ↓
Document
```

The project intentionally builds autonomy step by step instead of treating PX4, perception and control as black boxes.

---

## Author

**İlknur Hançer**  
Software Engineering Student

Focus areas:

- C / C++
- Linux / Unix
- UAV Software
- Autonomous Systems
- PX4 / Gazebo
- Computer Vision