# MPC Based Dynamic Obstacle Avoidance and Path Tracking

## Problem Statement

This assignment is based on the reference repository:

https://github.com/manojkarnekar/smoothing-rpp/tree/master

The original repository uses a Pure Pursuit controller for trajectory tracking. The robot stops whenever an obstacle comes in the path and resumes only after the obstacle is removed.

### Objective

Replace the Pure Pursuit tracker with an MPC-based trajectory tracker and enable the robot to:

- Follow a smooth trajectory
- Avoid static obstacles
- Avoid dynamic obstacles
- Rejoin the original path after obstacle avoidance
- Use 2D LiDAR for obstacle detection

---

# Implemented Solution

## 1. MPC Based Trajectory Tracking

The original Pure Pursuit tracker was replaced with a custom MPC-like trajectory tracker implemented in Python using ROS2.



### MPC Workflow

1. Generate candidate trajectories
2. Simulate robot motion
3. Compute cost for every trajectory
4. Select minimum cost trajectory
5. Publish optimal velocity commands

---

# MPC Cost Function

The cost function considers:

| Parameter | Purpose |
|---|---|
| Path Error | Keep robot close to path |
| Goal Error | Move robot towards goal |
| Obstacle Cost | Avoid collisions |
| Angular Velocity Cost | Reduce oscillations |
| Velocity Reward | Encourage forward motion |

Final cost:

```python
cost += 3.0 * path_error
cost += 2.0 * goal_error
cost += obstacle_cost
cost += 0.2 * abs(w)
cost -= 8.0 * v
```
---



# Dynamic Bubble Planner

A custom Dynamic Bubble Planner was implemented in C++.

## Responsibilities

- Read CSV waypoints
- Generate smooth path
- Detect obstacles using LiDAR
- Generate local detour around obstacle
- Reconnect robot back to original path

---

# Obstacle Detection

Obstacle information is obtained using:

- `/scan` topic
- `sensor_msgs/msg/LaserScan`

Detected obstacle points are transformed from:

- LiDAR frame
- to
- Odom frame



---

# Static Obstacle Avoidance

When an obstacle blocks the path:

1. Collision region is detected
2. Start and end indices are computed
3. Safe offset is generated
4. Temporary curved detour path is created
5. MPC follows detour path
6. Robot rejoins original path

---

# Dynamic Obstacle Handling

Dynamic obstacles are handled continuously.

## Approach

- LiDAR updates obstacles in real time
- Planner regenerates avoidance path
- MPC replans trajectory every control cycle
- Robot smoothly avoids moving obstacles

This allows the robot to react to randomly appearing obstacles during traversal.

---

# ROS2 Topics Used

| Topic | Type | Purpose |
|---|---|---|
| `/path` | nav_msgs/Path | Planned path |
| `/raw_path` | nav_msgs/Path | Original path |
| `/odom` | nav_msgs/Odometry | Robot pose |
| `/scan` | sensor_msgs/LaserScan | LiDAR data |
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands |

---

# Important Parameters

## MPC Parameters

```python
self.horizon = 30
self.dt = 0.06
self.max_v = 0.3
self.max_w = 1.25
```

## Planner Parameters

```cpp
safe_distance = 0.4
lookahead_points = 80
path_resolution = 0.05
```

---

# Algorithm Flow

## Planner

1. Load CSV waypoints
2. Smooth path
3. Detect obstacles
4. Generate local detour
5. Publish updated path

## MPC Controller

1. Receive path
2. Read odometry
3. Read LiDAR
4. Simulate trajectories
5. Compute trajectory cost
6. Publish best velocity

---

# Complete Execution Order

Run the following commands in separate terminals:

## Terminal 1

```bash
ros2 launch nav sim_bringup.py
```

## Terminal 2

```bash
ros2 run nav smoothing
```



# Conclusion

The assignment successfully replaces the Pure Pursuit controller with an MPC-based trajectory tracker.

The system:

- Tracks smooth trajectories
- Avoids static obstacles
- Handles dynamic obstacles
- Rejoins original path
- Uses real-time LiDAR data

The final solution provides significantly smoother and safer navigation compared to the original Pure Pursuit implementation.

---
