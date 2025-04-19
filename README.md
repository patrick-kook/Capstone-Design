# Capstone-Design
# Trajectory Generator using D* Algorithm with ROS

This project implements a complete motion planning and control pipeline for autonomous driving using the D* (Dynamic A*) algorithm in a ROS environment. It is designed to operate both in simulation and on real vehicles.

## Dependencies & Base Repositories

This project builds upon the following:

- **Motion Planning Core**  
  https://github.com/ai-winter/ros_motion_planning  
  Used as the base implementation for global and local path planning architecture.

- **Simulation Environment**  
  https://github.com/f1tenth/f1tenth_simulator  
  Used for testing and validating the planner in a realistic driving simulation environment.

## System Overview

The system works as follows:

1. **Global Path Planning**  
   The D* algorithm subscribes to a map in the form of an `OccupancyGrid` topic and generates an initial global path from the robot's start point to the goal.

2. **Local Path Planning**  
   During runtime, LiDAR data is subscribed through the `/scan` topic. The system detects dynamic obstacles and updates the local path accordingly, ensuring real-time replanning and collision avoidance.

3. **Control**  
   The generated local path is tracked using an LQR (Linear Quadratic Regulator) controller to ensure smooth and optimal vehicle control.

## Parameter Configuration

You can customize the behavior of the planner by editing the files located in:


Key files and parameters include:

### global_costmap_params.yaml

- `cost_scaling_factor`: Determines how fast obstacle cost drops off. (default: `2.0`)
- `inflation_radius`: Safe buffer zone around obstacles. (default: `0.8`)
- Sensor input via `/scan` topic is configured in the `obstacle_layer`.

### global_costmap_plugins.yaml

Enables costmap layers:
- `costmap_2d::StaticLayer`
- `costmap_2d::ObstacleLayer`
- `costmap_2d::VoronoiLayer`
- `costmap_2d::InflationLayer`

### local_costmap_params.yaml

- Uses a `rolling_window` centered on the robot in the `odom` frame.
- `inflation_radius` can be tuned for local obstacle sensitivity.

### move_base_params.yaml

- Controls frequency and patience of planning and control:
  - `planner_frequency`, `controller_frequency`
  - `planner_patience`, `controller_patience`

## Important Notes

- Before launching, make sure a valid `tf` tree is available.
- A `base_link` frame must be published, as the planner depends on transform lookups.

## How to Run

```bash
# 1. Clone the repository into your catkin workspace
cd ~/catkin_ws/
git clone https://github.com/patrick-kook/Capstone-Design.git

# 2. Build the workspace
cd ~/catkin_ws/src
catkin_make

# 3. Launch the simulation environment
roslaunch sim_env move_base.launch
