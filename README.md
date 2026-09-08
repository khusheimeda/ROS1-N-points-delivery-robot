# Autonomous Multi-Goal Delivery Robot

COMPSCI-603 Robotics  
Smitha Maganti · Khushei Meghana Meda · Akarsh Gupta · Skanda Chandrashekar

An indoor delivery robot that maps a space, visits N drop-off locations in an efficient order, and replans when something unexpected blocks its path.

This is a proof-of-concept inspired by hospital delivery robots (for example Aethon TUG): nurses spend a lot of time moving medications and supplies between rooms. The robot is meant to handle that kind of multi-stop indoor route on a mapped floor.

## What it does

- Builds a 2D occupancy map of the environment with GMapping (LiDAR + odometry)
- Localizes on that map with AMCL
- Plans obstacle-aware paths with a custom A* planner (obstacles inflated by the robot radius)
- Visits multiple goals using a fixed order or greedy nearest-neighbor (by A* path cost)
- Watches the LiDAR for new obstacles on the current path, stops, updates the map, and replans
- Uses a two-ring stop on the holonomic base so the robot slows down, then parks without jittering at the goal

## Platform

| | |
| --- | --- |
| Robot | Triton, omnidirectional (holonomic) base |
| Computer | NVIDIA Jetson Nano, ROS Noetic |
| LiDAR | RPLIDAR A2M12 (`/scan`) — SLAM, localization, obstacle checks |
| Odometry | Wheel encoders (`/odom`) — AMCL motion model |
| Camera | Intel RealSense D435 is on the robot but not required for this 2D navigation stack |

## Architecture

Independent ROS nodes talk over topics and services:

| Node | Role |
| --- | --- |
| `astar_planner` | Inflates the occupancy grid and returns an A* path as a service |
| `omni_controller` | Holonomic path following with outer brake ring (~40 cm) and inner stop (~15 cm) |
| `obstacle_monitor` | Filters LiDAR to a nearby “danger” region and a corridor along the planned path |
| `delivery_manager` | Orders remaining goals, sends them to the planner, handles arrival vs. replan |
| `led_controller` | Status colors on the robot LED ring (moving, obstacle, delivered, done) |

**Replanning:** if a new obstacle sits on the path, motion is stopped and a virtual wall is drawn on a copy of the map so A* cannot return the same blocked route. After all N points are done, the original map can be restored.

The package code lives in [`stingray_camera/`](stingray_camera/) (Triton / Stingray ROS package plus the delivery nodes). Hardware bring-up, TF, and SLAM details are in [`stingray_camera/README.md`](stingray_camera/README.md).

## Repository layout

```
stingray_camera/
  launch/          triton.launch, mapping, AMCL, navigation, full delivery
  scripts/         planner, controller, obstacle monitor, delivery manager, teleop
  maps/            saved occupancy grids (including house_final)
  rviz/            AMCL / SLAM configs
  config/          GMapping params
demo.mov.zip       demo video (Git LFS)
Autonomous Multi-Goal Delivery Robot.pdf
```

The demo video is stored with Git LFS. Install [Git LFS](https://git-lfs.com/) before cloning if you want the video:

```bash
git lfs install
git clone https://github.com/khusheimeda/ROS1-N-points-delivery-robot.git
```

## Setup

On a Triton with the Stingray camera package already installed, skip to [Run](#run). For a fresh Jetson image, follow the dependency steps in [`stingray_camera/README.md`](stingray_camera/README.md) (RPLIDAR, GMapping, AMCL, map_server, rosserial, and so on).

Put this repo’s `stingray_camera` package on the robot (or symlink it) inside your catkin workspace:

```bash
cd ~/catkin_ws/src
# copy or clone so stingray_camera is a catkin package here
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Update the map path in `stingray_camera/launch/triton_amcl.launch` if your workspace is not `/home/triton/catkin_ws`.

Edit delivery goals in `stingray_camera/launch/triton_full_delivery.launch` so the `{x, y, name}` entries match your map.

## Map the space

Furniture changes invalidate the map and AMCL, so remap after the environment changes.

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch stingray_camera triton.launch
```

In other terminals:

```bash
rosrun stingray_camera teleop_robot.py
roslaunch stingray_camera triton_gmapping.launch
```

Drive the robot through the space, then save:

```bash
rosrun map_server map_saver -f ~/catkin_ws/src/stingray_camera/maps/house_final
```

## Run

Bring up the robot, localization, then the delivery pipeline:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch stingray_camera triton.launch
roslaunch stingray_camera triton_amcl.launch
roslaunch stingray_camera triton_full_delivery.launch
```

`triton_full_delivery.launch` starts navigation (A* + omni controller), the obstacle monitor, LED feedback, and the delivery manager.

Set `ordering_strategy` in that launch file to `fixed` (launch-file order) or `greedy` (nearest remaining goal by A* cost).

## Challenges we hit

- **Perceptual aliasing** — similar-looking places confuse localization
- **Map invalidation** — environment changes break AMCL until you remap
- **Kidnapping** — if the robot is moved, it may not know it was relocated
- **Floor transitions** — teleop can cross inter-room thresholds that autonomous driving still struggles with
- **Memoryless A*** — without writing new obstacles into the map, replanning kept returning the blocked path

## Future work

Multi-floor delivery, stronger obstacle handling, a payload / medicine-carrying demo, delivery tracking and notifications, multi-robot coordination, and emergency-stop behavior.
