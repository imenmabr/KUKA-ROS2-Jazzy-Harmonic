# KUKA-ROS2 Jazzy-Harmonic

ROS 2 Jazzy integration for KUKA industrial robots using Gazebo Harmonic,
`gz_ros2_control`, and MoveIt 2.

> [!NOTE]
> This repository is a maintained fork of
> [REZ3LIET/KUKA-ROS2](https://github.com/REZ3LIET/KUKA-ROS2).
>
> The original project targets ROS 2 Humble and Gazebo Fortress.
> This fork updates the simulation stack for ROS 2 Jazzy and Gazebo Harmonic.

## Overview

This repository provides simulation and motion-planning support for KUKA
industrial robot arms using the current ROS 2 Jazzy / Gazebo Harmonic stack.

The migration replaces the previous Ignition Gazebo / Fortress integration
with the modern Gazebo Harmonic architecture while preserving the original
KUKA robot descriptions, MoveIt configurations, controller structure, and
Robotiq gripper support.

This project currently targets simulation only and does not provide hardware
integration for physical KUKA controllers.

## Supported Software

| Component | Version |
|---|---|
| Ubuntu | 24.04 LTS |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic |
| Gazebo / ROS interface | ros_gz |
| Simulation control | gz_ros2_control |
| Motion planning | MoveIt 2 |
| Robot | KUKA KR70 R2100 |
| Grippers | Robotiq 2F-85 / 2F-140 |


## Features

- KUKA KR70 R2100 simulation in Gazebo Harmonic
- ROS 2 Jazzy integration
- `gz_ros2_control`
- Joint trajectory control
- MoveIt 2 motion planning
- RViz visualization
- Robotiq gripper support
- Gazebo / ROS 2 simulation-time integration