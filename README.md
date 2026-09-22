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
                    │   2D Goal Pose     │
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
