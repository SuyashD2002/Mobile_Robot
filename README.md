# Autonomous Mobile Robot — ROS 2

A ROS 2-based autonomous mobile robot simulation developed for warehouse and indoor navigation applications.

The project integrates LiDAR sensing, SLAM, localization, Nav2 autonomous navigation, Gazebo simulation, safety monitoring, battery monitoring, and a velocity watchdog into a complete robotic software architecture.

---

## Project Overview

The goal of this project is to develop and simulate an autonomous indoor mobile robot capable of:

- Perceiving its environment using LiDAR
- Building a 2D map using SLAM
- Localizing itself within a known map
- Planning collision-free paths
- Following planned paths autonomously
- Detecting obstacles and stopping safely
- Monitoring battery status
- Controlling robot velocity through a safety watchdog
- Operating inside a simulated warehouse environment

The complete system was developed and tested using ROS 2 Humble and Gazebo.

---

## System Architecture

```text
                    ┌─────────────────────┐
                    │      RViz 2         │
                    │   2D Goal Pose      │
                    └──────────┬──────────┘
                               │
                               ▼
                    ┌─────────────────────┐
                    │       Nav2          │
                    │                     │
                    │  NavFn Planner      │
                    │  RPP Controller     │
                    └──────────┬──────────┘
                               │
                         /cmd_vel_nav
                               │
                               ▼
                    ┌─────────────────────┐
                    │  Safety Watchdog    │
                    │                     │
                    │ Velocity Monitoring │
                    │ Safety Control      │
                    └──────────┬──────────┘
                               │
                         /cmd_vel_safe
                               │
                               ▼
                    ┌─────────────────────┐
                    │      Gazebo         │
                    │  Differential Drive │
                    └──────────┬──────────┘
                               │
                               ▼
                         Mobile Robot
                               │
              ┌────────────────┴────────────────┐
              │                                 │
              ▼                                 ▼
          LiDAR Sensor                     Odometry
              │                                 │
              ▼                                 ▼
            /scan                              /odom
              │
              ▼
        SLAM / Localization
```

---

## Key Features

### Autonomous Navigation

The robot uses the ROS 2 Navigation2 (Nav2) stack for autonomous navigation.

The navigation pipeline consists of:

- **NavFn** — global path planning
- **Regulated Pure Pursuit (RPP)** — local path following
- **Recovery behaviors** — handling navigation situations
- **Goal checking** — determining when the robot has reached its destination
- **Progress checking** — monitoring robot movement during navigation

The robot can receive a navigation goal from RViz and autonomously plan and follow a path.

---

### LiDAR Perception

A simulated LiDAR sensor provides 2D laser scan data through:

```text
/scan
```

The LiDAR is used for:

- Obstacle detection
- Environment perception
- SLAM
- Navigation and collision checking

The sensor publishes `sensor_msgs/msg/LaserScan` data using the `lidar_link` frame.

---

### SLAM

SLAM Toolbox is used to generate a 2D occupancy grid map from LiDAR measurements.

The project includes a saved warehouse map:

```text
maps/
├── warehouse_map.pgm
└── warehouse_map.yaml
```

The SLAM configuration can be found in:

```text
config/slam.yaml
```

---

### Localization

The robot uses the map and sensor information for localization during autonomous navigation.

The navigation system uses the following coordinate frames:

```text
map
 └── odom
      └── base_link
           └── lidar_link
```

---

### Safety Watchdog

A dedicated velocity watchdog is implemented between Nav2 and the simulated robot.

Instead of sending navigation commands directly to the robot:

```text
Nav2 → Robot
```

the project uses:

```text
Nav2
  │
  ▼
/cmd_vel_nav
  │
  ▼
Safety Watchdog
  │
  ▼
/cmd_vel_safe
  │
  ▼
Gazebo Robot
```

This provides an additional safety layer for velocity commands.

The watchdog can prevent unsafe velocity commands from reaching the robot.

---

### Obstacle Safety

The robot continuously monitors LiDAR measurements.

When an obstacle is detected within the configured safety distance, the robot enters an obstacle-stop state and velocity is reduced to zero.

Example safety state:

```text
OBSTACLE_STOP
Velocity: 0 m/s
```

This safety mechanism operates independently of the navigation planner.

---

### Battery Monitoring

A battery monitoring node tracks the simulated battery state of the robot.

The system can detect low battery conditions and report the robot's battery status.

Example state:

```text
LOW_BATTERY
```

This provides a foundation for future battery-aware navigation and automatic charging behavior.

---

## Software Architecture

The main ROS 2 package is:

```text
robot_system
```

### Package Structure

```text
robot_system/
│
├── CMakeLists.txt
├── package.xml
│
├── config/
│   ├── ekf.yaml
│   ├── slam.yaml
│   └── nav2/
│       └── nav2_params.yaml
│
├── include/
│   ├── Robot.h
│   └── StateManager.h
│
├── launch/
│   ├── nav2.launch.py
│   ├── robot_system.launch.py
│   └── simulation.launch.py
│
├── maps/
│   ├── warehouse_map.pgm
│   └── warehouse_map.yaml
│
├── scripts/
│   └── dashboard.py
│
├── src/
│   ├── Robot.cpp
│   ├── StateManager.cpp
│   ├── battery_node.cpp
│   ├── cmd_vel_watchdog.cpp
│   ├── gazebo_lidar_node.cpp
│   ├── lidar_node.cpp
│   └── robot_controller_node.cpp
│
├── urdf/
│   ├── mobile_robot.urdf
│   └── mobile_robot_backup.urdf
│
└── worlds/
    └── slam_environment.world
```

---

## Technologies

| Technology | Purpose |
|---|---|
| ROS 2 Humble | Robot middleware and system architecture |
| C++ | Robot control, safety and ROS 2 nodes |
| Python | Launch files and dashboard |
| Gazebo Classic | Robot and environment simulation |
| RViz 2 | Visualization and navigation goals |
| Nav2 | Autonomous navigation |
| NavFn | Global path planning |
| Regulated Pure Pursuit | Local path following |
| SLAM Toolbox | 2D mapping |
| LiDAR | Environment perception |
| URDF | Robot modelling |
| CMake | C++ build system |

---

## Main ROS 2 Nodes

### `robot_controller_node`

Responsible for the main robot control and state management.

Publishes robot state and velocity information.

---

### `cmd_vel_watchdog`

Provides a safety layer between Nav2 and the robot.

```text
/cmd_vel_nav → /cmd_vel_safe
```

---

### `battery_node`

Monitors the simulated battery level and reports battery state.

---

### `lidar_node`

Handles LiDAR-related robot sensing and processing.

---

### `gazebo_lidar_node`

Provides LiDAR integration with the Gazebo simulation environment.

---

### `dashboard`

Provides a simple monitoring interface for robot state and system information.

---

## Navigation Pipeline

The autonomous navigation workflow is:

```text
1. Start Gazebo
       ↓
2. Spawn Mobile Robot
       ↓
3. Start Sensors
       ↓
4. Start Nav2
       ↓
5. Load Warehouse Map
       ↓
6. Set Initial Pose in RViz
       ↓
7. Send Navigation Goal
       ↓
8. NavFn calculates global path
       ↓
9. RPP follows the path
       ↓
10. Safety Watchdog monitors velocity
       ↓
11. Robot reaches goal
```

---

## Installation

### Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic
- Nav2
- SLAM Toolbox
- `robot_localization`

Create a ROS 2 workspace:

```bash
mkdir -p ~/ros2_robot_ws/src
cd ~/ros2_robot_ws/src
```

Clone the repository:

```bash
git clone https://github.com/SuyashD2002/Mobile_Robot.git
```

Build the workspace:

```bash
cd ~/ros2_robot_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install
```

Source the workspace:

```bash
source ~/ros2_robot_ws/install/setup.bash
```

---

## Running the Simulation

Start the robot simulation:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_robot_ws/install/setup.bash

ros2 launch robot_system robot_system.launch.py
```

---

## Running Nav2

After the simulation is running:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_robot_ws/install/setup.bash

ros2 launch nav2_bringup bringup_launch.py \
  map:=/home/$USER/ros2_robot_ws/src/robot_system/maps/warehouse_map.yaml \
  params_file:=/home/$USER/ros2_robot_ws/src/robot_system/config/nav2/nav2_params.yaml \
  use_sim_time:=True
```

---

## RViz Navigation

In RViz:

1. Set the **Fixed Frame** to:

```text
map
```

2. Add or verify:

```text
Map
LaserScan
TF
RobotModel
Path
```

3. Use **2D Pose Estimate** to provide the robot's initial pose.

4. Use **2D Goal Pose** to send a navigation goal.

The robot then plans and follows the path autonomously.

---

## Safety Architecture

The project intentionally separates navigation from safety control.

```text
             Nav2
              │
              │ Navigation command
              ▼
        /cmd_vel_nav
              │
              ▼
     ┌─────────────────┐
     │ Safety Watchdog │
     │                 │
     │ Velocity Check  │
     │ Safety Logic    │
     └────────┬────────┘
              │
              ▼
        /cmd_vel_safe
              │
              ▼
           Gazebo
```

This architecture provides an additional control layer between the autonomous navigation stack and the robot actuators.

---

## Current Status

The current system supports:

- [x] ROS 2 robot package
- [x] Gazebo simulation
- [x] URDF robot model
- [x] LiDAR sensing
- [x] 2D SLAM
- [x] Saved warehouse map
- [x] Nav2 integration
- [x] NavFn global planner
- [x] Regulated Pure Pursuit controller
- [x] RViz navigation goals
- [x] Autonomous robot movement
- [x] Obstacle safety stop
- [x] Velocity watchdog
- [x] Battery monitoring
- [x] Robot state management

---

## Future Improvements

Planned improvements include:

- Dynamic obstacle avoidance
- Automatic battery charging
- Battery-aware navigation
- Improved robot diagnostics
- Mission management
- Waypoint-based warehouse missions
- Sensor fusion
- Improved recovery behaviors
- Multi-robot coordination
- Hardware deployment

---

## Project Objective

This project was developed to gain practical experience in autonomous robotics software development, focusing on the integration of perception, localization, planning, control, and safety within a ROS 2-based robotic system.

The architecture is designed to provide a foundation that can be extended from simulation toward real-world autonomous mobile robot applications.

---

## Author

**Suyash D2002**

Master's Student — Mechatronics and Cyber-Physical Systems

Germany

---

## License

This project is intended for educational and portfolio purposes.
