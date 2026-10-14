# Rover Workspace — ROS 2

<p align="center">
  <strong>ROS 2 workspace for a mobile rover with simulation, teleoperation, localization, mapping, coverage planning, firmware communication, and vision modules.</strong>
</p>

<p align="center">
  <a href="https://www.ros.org/"><img src="https://img.shields.io/badge/ROS%202-Humble--oriented%20configs-22314E?logo=ros" alt="ROS 2"></a>
  <a href="https://gazebosim.org/"><img src="https://img.shields.io/badge/Simulation-Gazebo-orange" alt="Gazebo"></a>
  <img src="https://img.shields.io/badge/Language-C%2B%2B%20%7C%20Python-blue" alt="Languages">
  <img src="https://img.shields.io/badge/Platform-Ubuntu%20%7C%20Linux-informational" alt="Platform">
</p>

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Repository Structure](#repository-structure)
- [Requirements](#requirements)
- [Installation](#installation)
- [Build the Workspace](#build-the-workspace)
- [Run in Simulation](#run-in-simulation)
- [Run on the Real Robot](#run-on-the-real-robot)
- [Individual Packages and Nodes](#individual-packages-and-nodes)
- [Topics and Interfaces](#topics-and-interfaces)
- [Configuration Guide](#configuration-guide)
- [Troubleshooting](#troubleshooting)
- [Known Limitations and Pre-run Checks](#known-limitations-and-pre-run-checks)
- [Contributing](#contributing)
- [License](#license)

## Overview

This repository contains a ROS 2 workspace for a differential-drive rover. It brings together the robot model, control nodes, `ros2_control` configuration, hardware serial interface, joystick teleoperation, localization, map publishing and SLAM, path planning, and vision-based mine/cell processing.

This is a **workspace**, not a single ROS package: clone it, install its dependencies, build it with `colcon`, and source the generated setup file before launching nodes.

> **Important:** The launch files and parameters represent the current repository configuration. Hardware device names, wheel geometry, serial protocol, TF frame names, map path, and ROS distribution must match your setup. Review [Known Limitations and Pre-run Checks](#known-limitations-and-pre-run-checks) before running the real robot.

## Features

- **Robot description:** URDF/Xacro model with mesh assets, wheel joints, gripper, IMU and LiDAR links.
- **Simulation:** Gazebo launch integration and an RViz robot-description view.
- **Control:** C++ and Python controller implementations, wheel-velocity commands, odometry and TF publishing.
- **Joystick operation:** Joystick input, configurable axes/buttons, velocity multiplexing, and helper nodes for gripper and camera-servo teleoperation.
- **Hardware interface:** A `ros2_control` hardware plugin and serial communication utilities for the microcontroller.
- **Localization:** EKF configuration and launch files for local/global localization components.
- **Mapping:** Existing map files and a SLAM Toolbox launch/configuration.
- **Planning:** A* and Dijkstra grid planners plus a Zigzag coverage-path planner.
- **Vision:** Camera and vision nodes using OpenCV/EasyOCR, mine classification, grid-cell publishing, and a visual mine-map module.

## System Architecture

At a high level, the main data flow is:

1. **Robot description** supplies the robot model and joint/link definitions.
2. **Controller and hardware interface** turn velocity commands into wheel or hardware commands and publish feedback/odometry.
3. **Joystick teleoperation** reads the gamepad and routes motion commands through `twist_mux` and the configured relay.
4. **Mapping/localization** provide map and robot-pose/TF information.
5. **Planning** generates a path after a goal-trigger message.
6. **Vision modules** process camera imagery and publish recognized cell/mine information.

Modules can be launched individually for debugging. A bringup launch attempts to combine several modules into a single stack.

## Repository Structure

Source packages are located under `src/`.

| Package | Responsibility |
|---|---|
| `rover_bringup` | Combined launch files for simulation and the real robot |
| `rover_controller` | Simple/noisy controllers, joystick teleoperation, velocity relay, and controller YAML files |
| `rover_description` | URDF/Xacro model, meshes, RViz and Gazebo launch files |
| `rover_firmware` | `ros2_control` hardware interface, serial utilities, firmware examples, and hardware launch |
| `rover_localization` | EKF/AMCL-related configuration and localization launch files |
| `rover_map` | Static maps, map-server launch, initial-pose and annotated-map utilities |
| `rover_mapping` | SLAM Toolbox configuration and SLAM launch |
| `rover_planning` | A*, Dijkstra, and Zigzag planner implementations |
| `rover_motion` | Motion-related ROS 2 package |
| `rover_msgs` | Custom ROS interface package |
| `vision_pkg` | Camera processing, OCR/vision, mine classification, and map visualization |

Useful entry-point files:

- `rover_bringup/launch/simulated_robot.launch.py` — simulation-oriented stack.
- `rover_bringup/launch/real_robot.launch.py` — hardware-oriented stack.
- `rover_description/urdf/rover.urdf.xacro` — main robot model.
- `rover_controller/config/sim_rover_controllers.yaml` — simulation controller settings.
- `rover_controller/config/real_rover_controllers.yaml` — separate real-robot controller settings.
- `rover_controller/launch/joystick_teleop.launch.py` — joystick, teleop, mux, and relay nodes.
- `rover_localization/config/ekf.yaml` — EKF sensor/frame configuration.
- `rover_mapping/config/slam_toolbox.yaml` — SLAM Toolbox parameters.
- `rover_map/maps/map.yaml` — metadata for the committed static map.
- `rover_planning/src/zigzag_planner.cpp` — Zigzag coverage planner.

## Requirements

### Core environment

The source contains compatibility logic for ROS 2 Humble and newer Gazebo naming conventions. Start with a single ROS 2 distribution for the entire workspace and its dependencies. Avoid mixing packages from different ROS distributions in the same environment.

A reasonable starting environment for the checked-in configurations is:

- Ubuntu 22.04
- ROS 2 Humble
- `colcon`, `rosdep`, and the standard ROS 2 development tools

Other ROS 2 distributions may require launch/configuration changes; this README does not claim that all packages have been validated on every distribution.

### Main dependencies referenced by the source

- `ros2_control`, `ros2_controllers`, `controller_manager`
- `robot_state_publisher`, `xacro`, `joint_state_publisher_gui`
- `rviz2`, `ros_gz_sim`, `ros_gz_bridge`
- `joy`, `joy_teleop`, `twist_mux`
- `robot_localization`, `nav2_map_server`, `nav2_lifecycle_manager`, `nav2_amcl`
- `slam_toolbox`
- `tf2_ros`, `tf2`, `tf_transformations`
- Eigen and libserial development packages
- OpenCV, NumPy, `cv_bridge`, and EasyOCR for vision functionality

Install dependency packages compatible with your ROS distribution; exact apt package names may vary.

## Installation

### 1. Install ROS 2 and development tools

Follow the official instructions for the ROS 2 distribution you plan to use:

- [ROS 2 documentation](https://docs.ros.org/)
- [ROS 2 Humble on Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

Install common workspace tools:

```bash
sudo apt update
sudo apt install -y git python3-rosdep python3-colcon-common-extensions
```

Initialize `rosdep` if it has not already been initialized:

```bash
sudo rosdep init
rosdep update
```

If `sudo rosdep init` reports that it was already initialized, continue with `rosdep update`.

### 2. Clone the repository

```bash
mkdir -p ~/rover_ws
cd ~/rover_ws
git clone https://github.com/Ahmednadi812/rover_ws.git .
```

### 3. Install package dependencies

The example below uses Humble. Source ROS, then run `rosdep` from the workspace root:

```bash
source /opt/ros/humble/setup.bash
cd ~/rover_ws
rosdep install --from-paths src --ignore-src -r -y
```

If `rosdep` cannot resolve a dependency, confirm the correct ROS apt repository is enabled and the dependency exists for your ROS distribution. Hardware serial support may also require the system libserial development package.

## Build the Workspace

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
cd ~/rover_ws
colcon build --symlink-install
```

Build a specific package while debugging:

```bash
colcon build --symlink-install --packages-select rover_controller
```

Replace `rover_controller` with the package you are working on. After changing computers, moving the workspace directory, or switching ROS distributions, cached CMake paths may be stale. Remove generated outputs only—not `src/`—then rebuild:

```bash
cd ~/rover_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Source both the ROS environment and the workspace in **every new terminal** used to run the project:

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
```

Confirm ROS can discover the packages:

```bash
ros2 pkg list | grep '^rover_'
ros2 pkg list | grep '^vision_pkg$'
```

For another ROS distribution, replace `humble` consistently with the distribution installed on the machine.

## Run in Simulation

Start with simulation before testing physical motion. Source the same ROS installation and workspace in each terminal.

### Full simulation bringup

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
ros2 launch rover_bringup simulated_robot.launch.py
```

The launch file is intended to bring up Gazebo, the controller stack, joystick teleoperation, map server, localization, RViz, and the Zigzag planner. It depends on all required packages being installed and the launch interfaces, topic names, TF frames, and configurations matching. If launch fails, use the individual commands below to isolate the failing component.

### Robot-description visualization only

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
ros2 launch rover_description display.launch.py
```

This launch starts the robot-state publisher, joint-state GUI, and RViz visualization.

### Gazebo only

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
ros2 launch rover_description gazebo.launch.py
```

### Inspect the running graph

In another sourced terminal:

```bash
ros2 node list
ros2 topic list
ros2 control list_controllers
```

If `ros2 control list_controllers` cannot connect to `/controller_manager`, check that the robot description or bringup launch successfully started the controller manager.

## Run on the Real Robot

> **Safety:** For the first motor test, use an emergency-stop method and raise the drive wheels off the ground. Confirm wiring, battery polarity, motor-driver limits, the correct microcontroller firmware, and the intended serial device before enabling motion.

Run the combined hardware bringup:

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
ros2 launch rover_bringup real_robot.launch.py
```

This launch is intended to start the hardware interface, controllers, joystick teleoperation, map server, localization, and Zigzag planner. The current launch file should be checked on the target machine before relying on it as a single-command bringup; see [Known Limitations and Pre-run Checks](#known-limitations-and-pre-run-checks).

Before operating, verify:

1. The serial device and baud rate used by the hardware interface match the microcontroller firmware.
2. Wheel radius, wheel separation, and joint names match the physical rover.
3. The Zigzag planner's `base_frame` exists in the active TF tree.
4. Real-hardware nodes use the intended real clock (`use_sim_time:=false` in typical deployments).
5. The loaded map describes the actual operating area and is aligned with localization.

### Check available serial ports

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

Configure the hardware interface for the device that actually appears on the target computer. Device names can change after reconnecting USB or rebooting. Ensure only one process owns the serial port at a time.

### Verify joystick input

```bash
ros2 topic echo /joy
```

If no messages arrive, confirm the gamepad is connected, the corresponding Linux input device exists, and the `joy_node` configuration selects the correct device. Button and axis indices may differ by controller.

## Individual Packages and Nodes

### Controller

The `rover_controller` package provides simple/noisy controllers, a velocity relay, and joystick helper nodes. Its controller launch file exposes options for the simple/noisy controller and C++/Python variants.

```bash
ros2 launch rover_controller controller.launch.py use_simple_controller:=true use_python:=false
```

For the Python simple-controller implementation, use `use_python:=true`. Run the controller only when its corresponding robot description/controller manager is active and the configured input/output topics are connected.

The checked-in `controller.launch.py` declares these arguments:

| Argument | Default | Purpose |
|---|---:|---|
| `use_sim_time` | `True` | Use simulated ROS time |
| `use_simple_controller` | `True` | Select the simple-controller group |
| `use_python` | `False` | Select Python rather than the C++ node variant |
| `wheel_radius` | `0.1` | Wheel radius used by the simple controller |
| `wheel_separation` | `0.5767` | Wheel separation used by the simple controller |
| `wheel_radius_error` | `0.005` | Offset used by the noisy controller |
| `wheel_separation_error` | `0.02` | Offset used by the noisy controller |

These values are configuration defaults, not universal rover dimensions. Set them to match the selected robot model and hardware.

### Joystick teleoperation

```bash
ros2 launch rover_controller joystick_teleop.launch.py use_sim_time:=true
```

For physical hardware, typical use is `use_sim_time:=false`. The launch includes `joy_node`, `joy_teleop`, `twist_mux`, a velocity relay, and the gripper teleoperation node.

Check the input and command topics:

```bash
ros2 topic echo /joy
ros2 topic list | grep -E 'cmd_vel|input_joy'
```

The committed mapping uses axis 1 for linear velocity, axis 3 for angular velocity, and button 5 as the deadman button. The gripper helper is configured with button indices 2 and 3. Validate all indices with your controller before moving the rover.

### Map server

The map server launch loads map metadata from `rover_map/maps/map.yaml`:

```bash
ros2 launch rover_map map_server.launch.py
```

The current launch also starts helper nodes and a static transform. Review its output-image path, map metadata, and frame choices before using a different machine or map.

### SLAM mapping

```bash
ros2 launch rover_mapping slam.launch.py use_sim_time:=true
```

Use `use_sim_time:=true` in simulation. For a real robot, normally use `false` when sensors publish wall-clock ROS timestamps. SLAM requires valid sensor data and TF; check the LiDAR topic, `base_frame`, and odometry settings in `rover_mapping/config/slam_toolbox.yaml`.

### Localization

Local and global localization launch files are located in `rover_localization/launch/`. The current bringup launch includes `local_localization.launch.py`.

To inspect localization separately:

```bash
ros2 launch rover_localization local_localization.launch.py use_sim_time:=true
```

Choose the time setting for the active clock source. The EKF file `rover_localization/config/ekf.yaml` expects input topics and frames as configured there; align those names with the actual running graph.

### Zigzag coverage planner

The Zigzag planner is a C++ node in `rover_planning/src/zigzag_planner.cpp`. It creates parallel coverage lanes within map bounds, samples points along the path, estimates pose headings, and publishes a `nav_msgs/msg/Path`.

| Interface | Type | Purpose |
|---|---|---|
| `/map` | `nav_msgs/msg/OccupancyGrid` | Input occupancy grid |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Triggers planning; its coordinates are not used as the final coverage endpoint |
| `/zigzag/path` | `nav_msgs/msg/Path` | Output coverage path |

Main parameters:

| Parameter | Node default | Meaning |
|---|---:|---|
| `base_frame` | `base_footprint_ekf` | Robot frame used for the start pose TF lookup |
| `margin` | `2.0` m | Inset from map borders |
| `lane_spacing` | `1.0` m | Distance between coverage lanes |
| `point_step` | `0.1` m | Approximate spacing between generated path points |
| `lanes_along_x` | `true` | `true` for lanes along X; `false` for lanes along Y |
| `area_min_x`, `area_max_x`, `area_min_y`, `area_max_y` | unset | Optional manual bounds in the map frame |

The bringup launch overrides `base_frame` to `base_footprint` and sets `margin:=2.0`, `lane_spacing:=1.0`, `point_step:=0.1`, and `lanes_along_x:=true`.

After the map and TF are available, start the planner alone:

```bash
ros2 run rover_planning zigzag_planner
```

Trigger a plan through RViz's Pose Estimate/goal publishing workflow as configured, or publish a test `PoseStamped` message:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

The pose is only a trigger for the Zigzag planner; it does not define the endpoint of the coverage path. Change the test coordinates if another component requires a valid start/goal position.

Inspect the output:

```bash
ros2 topic echo /zigzag/path
```

> **Planning limitation:** The current implementation checks whether generated points fall on occupied map cells and emits a warning, but it does not reroute around obstacles. Do not execute this path on a physical rover around obstacles without adding collision checking and validating the controller.

### A* and Dijkstra planners

The `rover_planning` package also includes Python and C++ A*/Dijkstra implementations. The Python nodes subscribe to `/map` and `/goal_pose`, obtain the robot pose through TF, and publish paths on `/a_star/path` or `/dijkstra/path`; they also publish visited-cell visualization maps.

```bash
ros2 run rover_planning a_star_planner
ros2 run rover_planning dijkstra_planner
```

These nodes require a valid map, a reachable robot TF, and matching frame names. Test them in RViz/simulation before real-world use.

### Vision package

The `vision_pkg` package exposes these Python entry points:

- `camera_node`
- `vision_node`
- `mine_classifier`
- `map_node`

Launch one node at a time to debug:

```bash
ros2 run vision_pkg camera_node
ros2 run vision_pkg vision_node
ros2 run vision_pkg mine_classifier
ros2 run vision_pkg map_node
```

Vision functionality uses OpenCV, NumPy, `cv_bridge`, and EasyOCR. Install compatible Python/ROS packages before running these nodes; the current `vision_pkg` manifest does not explicitly declare every runtime library imported in the code.

The current mine-map node subscribes to `/current_cell` and `/mine_type`, expects cell names in the A–S / 1–19 range, and recognizes the strings `Surface` and `Buried`. Check each node's source for its exact camera topic and output behavior.

### Firmware and serial utilities

The `rover_firmware` package contains a hardware plugin and serial transmitter/receiver utilities, along with example Arduino sketches under `rover_firmware/firmware/`. Its hardware launch builds the robot description and starts `ros2_control_node`.

Serial port, baud rate, command encoding, telemetry encoding, and the microcontroller firmware must match on both sides. The example sketches should not be assumed to use the same protocol as the production rover firmware.

## Topics and Interfaces

These interfaces are referenced directly in source. Relative topic names may resolve under namespaces, so confirm the resolved names on a running system.

| Topic | Type | Purpose |
|---|---|---|
| `/map` | `nav_msgs/msg/OccupancyGrid` | Static/SLAM occupancy grid |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Trigger for A*, Dijkstra, and Zigzag planning |
| `/zigzag/path` | `nav_msgs/msg/Path` | Zigzag coverage path |
| `/a_star/path` | `nav_msgs/msg/Path` | A* path output |
| `/dijkstra/path` | `nav_msgs/msg/Path` | Dijkstra path output |
| `/a_star/visited_map` | `nav_msgs/msg/OccupancyGrid` | A* exploration visualization |
| `/dijkstra/visited_map` | `nav_msgs/msg/OccupancyGrid` | Dijkstra exploration visualization |
| `/joy` | `sensor_msgs/msg/Joy` | Raw joystick input |
| `/joint_states` | `sensor_msgs/msg/JointState` | Joint feedback |
| `rover_controller/odom` | `nav_msgs/msg/Odometry` | Odometry from the simple controller (resolved name depends on namespace) |
| `/current_cell` | `std_msgs/msg/String` | Cell identifier consumed by the mine-map node |
| `/mine_type` | `std_msgs/msg/String` | Mine type consumed by the mine-map node |

Useful commands:

```bash
ros2 topic list
ros2 topic info /map
ros2 topic info /zigzag/path
ros2 node list
ros2 run tf2_tools view_frames
```

## Configuration Guide

### Wheel geometry and controllers

- `rover_controller/config/sim_rover_controllers.yaml` uses simulation values `wheel_radius: 0.1` and `wheel_separation: 0.5767`.
- `rover_controller/config/real_rover_controllers.yaml` contains `wheel_radius: 0.033` and `wheel_separation: 0.17`.
- `rover_firmware/launch/hardware_interface.launch.py` currently references `rover_controller/config/rover_controllers.yaml`.

Confirm which YAML is loaded in each launch path. Do not copy values between configs without checking the model and measuring the real rover.

### Clock source

- Simulation typically uses `use_sim_time:=true`.
- Physical sensors and hardware typically use `use_sim_time:=false`.

All nodes participating in one sensor/TF pipeline should use a consistent time source.

### TF frames

Names used across the repository include `map`, `odom`, `base_footprint`, `base_footprint_ekf`, `imu_link`, and `imu_link_ekf`. Check the published TF tree and ensure that every consumer is configured for a frame that actually exists.

### Static map metadata

The committed `rover_map/maps/map.yaml` points to `map.pgm` and declares a resolution of `0.025` m/cell with origin `[-0.5, -0.5, 0.0]`. Use a map that matches the physical environment, its origin, and localization setup.

## Troubleshooting

### Package not found after build

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
ros2 pkg list | grep rover
```

If the package is still missing, inspect the build output and resolve the package/dependency error first.

### Build fails after moving the workspace

CMake caches can retain absolute paths from an old user directory or machine. Remove generated output and rebuild:

```bash
cd ~/rover_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### No joystick messages

```bash
ros2 topic echo /joy
ls -l /dev/input/js*
```

Check the input device, user permissions, controller index, and axis/button mapping. A gamepad may be enumerated differently on a Raspberry Pi than on a laptop.

### Zigzag path is not published

```bash
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic echo /zigzag/path
```

If the planner reports that robot TF is missing, set `base_frame` to a valid frame in the current TF tree. The node can fall back to an area corner if it cannot find the robot, but a valid robot transform is preferable.

### Controller manager or controller is not active

```bash
ros2 control list_controllers
ros2 service list | grep controller_manager
```

Confirm the controller YAML loaded and that its joint names match the URDF and hardware interface.

### Serial interface does not work

Check the detected serial port, user permissions, baud rate, firmware, and the command/telemetry protocol. Ensure no second program is holding the same serial device.

### Vision dependencies or GUI fail

Install the ROS-compatible `cv_bridge` and Python libraries needed by the node. When running headless over SSH, OpenCV windows may not open; use an available display or update the node to save images/log results.

## Known Limitations and Pre-run Checks

The current repository contains several environment-dependent settings. Review them before relying on a one-command bringup:

1. **Machine-specific image path:** `rover_map/launch/map_server.launch.py` contains an output image path rooted at `/home/nadi/rover_ws`. Change it for your machine or make it configurable/relative.
2. **Simulation time in real bringup:** `rover_bringup/launch/real_robot.launch.py` currently passes `use_sim_time: True` to the Zigzag planner and passes `use_sim_time: True` into the localization launch. Real hardware normally uses wall-clock time; verify all nodes' clock configuration.
3. **TF frame consistency:** The Zigzag node declares `base_footprint_ekf` as its default frame, while bringup overrides it to `base_footprint`. Ensure the map/localization/odometry frame tree matches this choice.
4. **Controller YAML selection:** The hardware interface launch refers to `rover_controller/config/rover_controllers.yaml`, while `real_rover_controllers.yaml` also exists. Confirm which configuration is intended for the physical robot.
5. **Zigzag obstacle handling:** The planner warns about path points in occupied cells but does not avoid obstacles.
6. **Map launch dependencies:** The map-server launch starts helper executables as well as a map server. Confirm the helpers are built/installed and that the selected map image matches the YAML metadata.
7. **Vision dependencies:** The Python package manifest does not list every imported runtime library. Install and validate OpenCV, NumPy, `cv_bridge`, and EasyOCR separately.
8. **ROS distribution compatibility:** Gazebo integrations and package names vary between ROS distributions; validate them before switching between Humble, Jazzy, or another release.
9. **Bringup launch-source validation:** Review the `IncludeLaunchDescription` sources in bringup against the installed ROS 2 launch API if a combined launch fails before starting nodes; individual package launches can help isolate the failure.

When a launch or build fails, start with the earliest error message rather than relying only on the final `colcon` summary.

## Contributing

1. Create a feature branch for your change.
2. Keep simulation and hardware configuration differences explicit.
3. Validate the modified package:

   ```bash
   colcon build --symlink-install --packages-select <package_name>
   ```
4. Test new launch behavior and topic/frame contracts in simulation before using the physical robot.
5. Open a pull request with a concise description and reproducible test steps.

## License

Check the individual package manifests and the repository owner's licensing intent before redistributing or reusing the complete workspace. Several package manifests mention Apache 2.0, while others still contain placeholder license metadata; this README does not assign a single license to every file.

---

## Maintainer

**Ahmed Nadi**  
GitHub: [@Ahmednadi812](https://github.com/Ahmednadi812)

For issues, open a [GitHub Issue](https://github.com/Ahmednadi812/rover_ws/issues) and include your ROS distribution, Ubuntu version, exact launch command, the earliest relevant error output, and the result of `ros2 topic list` / `ros2 node list`.
