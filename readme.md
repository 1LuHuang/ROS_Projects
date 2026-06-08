# ROS2 Autonomous Navigation Project

## Project Overview

This project implements a complete autonomous navigation pipeline for a differential-drive vehicle in ROS2 and Gazebo.

The project evolved progressively from basic trajectory tracking to dynamic obstacle-aware navigation and trajectory prediction.

The objective is not only to follow a predefined path, but also to react safely to dynamic obstacles and prepare for future prediction-aware replanning.

---

# Stage 1: Trajectory Tracking

## Goal

Enable the vehicle to accurately follow a predefined trajectory.

## Implemented Components

### Reference Path Generator

A reference trajectory (e.g. figure-eight path) is generated and published.

### Pure Pursuit Controller

The controller computes steering commands based on a look-ahead target point on the reference trajectory.

### Vehicle Control

Velocity commands are sent through ROS2 topics and executed inside Gazebo.

## Result

The vehicle successfully follows the figure-eight trajectory with acceptable tracking accuracy.

---

# Stage 2: Static Obstacle Navigation

## Goal

Navigate through environments containing static obstacles.

## Implemented Components

### Occupancy Grid Map

Static obstacles are represented as occupied cells.

### A* Global Planner

A* searches for the shortest collision-free path from start to goal.

### Path Tracking

The Pure Pursuit controller tracks the generated A* path.

## Result

The vehicle successfully reaches the target while avoiding static obstacles.

---

# Stage 3: Dynamic Obstacle Generation

## Goal

Introduce moving obstacles into the environment.

## Implemented Components

### Dynamic Obstacle Model

A movable obstacle is added into the Gazebo world.

### Obstacle Motion Node

A ROS2 node continuously updates obstacle positions.

### Obstacle Pose Publisher

The obstacle's ground-truth pose is published as:

/dynamic_obstacle/pose

## Result

A dynamic environment is created where obstacles move continuously across the vehicle's path.

---

# Stage 4: Dynamic Obstacle Detection (V1)

## Goal

Detect whether a moving obstacle blocks the current path.

## Core Idea

Compute the minimum distance between:

* obstacle position
* global reference path

If the distance is below a safety threshold:

path_blocked = True

Otherwise:

path_blocked = False

## Output

/path_blocked

(Boolean topic)

## Result

The system can identify when a moving obstacle enters the planned path corridor.

---

# Stage 5: Obstacle Velocity Estimation (V2)

## Goal

Estimate obstacle motion.

## Method

Finite difference approximation:

vx = (x_t - x_(t-1)) / dt

vy = (y_t - y_(t-1)) / dt

## Result

The obstacle velocity can be estimated online.

---

# Stage 6: Future Trajectory Prediction (V3)

## Goal

Predict future obstacle positions.

## Method

Constant velocity prediction:

future_x = x + vx * horizon

future_y = y + vy * horizon

## Result

The system can estimate future obstacle locations before a collision occurs.

This allows proactive rather than reactive obstacle handling.

---

# Stage 7: Prediction-Aware Local Replanning (V4)

## Goal

Generate a new path before a future collision occurs.

## Planned Method

1. Predict obstacle future position.
2. Inflate predicted obstacle region.
3. Insert temporary obstacle into local map.
4. Run local A* replanning.
5. Publish a new reference path.
6. Continue tracking with Pure Pursuit.

## Expected Outcome

The vehicle no longer simply stops when obstacles appear.

Instead, it proactively generates an alternative trajectory and safely bypasses dynamic obstacles.

---

# System Architecture

Reference Path / A*
↓
Trajectory Tracking (Pure Pursuit)
↓
Vehicle Control
↑
Dynamic Obstacle Detection
↑
Velocity Estimation
↑
Trajectory Prediction
↑
Local Replanning (V4)

---

# Future Work

* Local MPC trajectory optimization
* Dynamic Window Approach (DWA)
* Learning-based local planners
* Reinforcement Learning for navigation
* MPC-RL hybrid navigation
* Multi-obstacle prediction
* Real sensor integration (LiDAR / Camera)
