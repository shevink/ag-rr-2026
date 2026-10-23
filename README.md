# RR: Rectangular Robot Navigation with Rotation
ROS 2 and Gazebo project for orientation-aware navigation of a rectangular mobile robot through obstacle-filled environments.

This task is part of the *Algorithmic Geometry (Summer 2026)* repository developed in cooperation with the HTWK Smart Driving Team.

## Objective
Development of a navigation planner that moves the robot from its current pose to a requested goal, whilst avoiding collisions with mapped obstacles. The intended planner must account for:
- The robot's rectangular footprint and heading.
- Narrow passages where orientation determines whether the robot fits.
- Static obstacle geometry from SVG-based maps.
- New goals superseding previous goals.
- Optional previously unknown static or moving obstacles.

## ROS interfaces
| Topic | Direction | Purpose |
| --- | --- | --- |
| `/goal` | Input | Target position; newer goals replace earlier goals |
| `/vehicle/position` | Input | Global vehicle pose and orientation, specified at 50 Hz |
| `/planning/path` | Output | Planned navigation route |


## Getting started
### Requirements
- Docker and Docker Compose
- The repository's VNC container profile.

### 1. Start the development environment (linux)
```bash
xhost +local:docker
docker compose --profile ag-linux build
docker compose --profile ag-linux up -d
docker exec -it ag-linux bash
```

### 2. Build and source the ROS workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch a scenario 
```bash
ros2 launch rr easy_without_obstacles.launch.py
ros2 launch rr hard_with_obstacles.launch.py
```
