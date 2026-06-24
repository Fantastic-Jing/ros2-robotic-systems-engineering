# ROS 2 for Robotic Systems Engineering: Autonomous Navigation and Coordination with TurtleBot3

This repository contains the complete implementation of the robotics vision, control, and multi-agent coordination pipeline developed during the **Robotic Systems Engineering** lab course at Hochschule Darmstadt (h-da). 

The project transitions from simulation environments to physical deployment on TurtleBot3 Burger robots using ROS 2 (Humble), tracking implementations from single-robot collision avoidance to multi-robot leader-follower configurations.

---

## Lab & Academic Supervision

This project was developed and validated in the robotics laboratory under the guidance of **Prof. Dr. Stephan Neser**.

* **Supervisor:** Prof. Dr. Stephan Neser
* **Institution:** Hochschule Darmstadt (h-da), Department of FBMN
* **Office:** Room C10/6.31
* **Contact:** [stephan.neser@h-da.de](mailto:stephan.neser@h-da.de) (Email for appointment during office hours)
* **Lab Website:** [www.fbmn.h-da.de/~neser](http://www.fbmn.h-da.de/~neser)

---

## Hardware & Environment Setup

All packages and nodes were tested under dual environments (Gazebo simulation and real physical deployment):
* **Robot Hardware:** TurtleBot3 Burger equipped with a Single Board Computer (SBC, Raspberry Pi 4) and a 360-degree Laser Distance Sensor (LDS-01).
* **OS & Middleware:** Ubuntu 22.04 LTS with **ROS 2 Humble**.
* **Remote Workspace:** Connected via SSH (`ssh -l obv <bot_name>`) to manage internal nodes on the robot's physical hardware.

---

## Laboratory Packages & Milestones

### Lab 1 & 2: Autonomous Navigation and Reactive Obstacle Avoidance
* **Package Path:** `lab1pkg`
* **Core Principle:** Transitioning from basic publisher/subscriber mechanics to a safety-guaranteed reactive driving node.
* **Nodes Implemented:**
  1. `stop_turtle_node`: A safety-override node that continuously broadcasts zero velocity onto `/cmd_vel` with a $0.2\text{ s}$ pause interval to reliably stop a running robot.
  2. `randwalk_turtle_node`: A basic timer-callback node ($T = 2\text{ s}$) that moves the robot at medium speed utilizing random linear and angular velocity inputs.
  3. `randwalk_safe_turtle_node`: An advanced node that subscribes to the `/scan` topic using a **BEST_EFFORT** QoS profile. It evaluates raw LaserScan arrays to sense physical boundaries. If an obstacle blocks the view, it overrides the random walk, triggers a custom recovery maneuver, and triggers the hardware `/sound` service as an acoustic collision alert.

### Lab 2 Analysis: Odometry Verification & QoS Evaluation
* **Data Logging:** Tracked physical trajectories by recording `/odom` topics into a `ros2 bag`.
* **MATLAB Integration:** Extracted custom data fields via Python parsing scripts and mapped the 2D coordinate shifts directly into MATLAB (`showRandWalk.m`) for path verification against ground-truth parameters.
* **Theoretical Findings:** Documented key answers regarding laser scanner detection limits, coordinate index mapping, and why a `BEST_EFFORT` + `KEEP_LAST (depth=1)` QoS profile is strictly required for high-frequency sensor streams where old data is immediately obsolete.

### Lab 3: Multi-Robot Leader-Follower System (Burger Follower)
* **Environment:** Multi-agent coordination launched via `my_multi_robot.launch.py` inside a customized Gazebo world.
* **Core Algorithm:** **Jump Distance Algorithm (JDA)**.
* **Logic:** The follower robot tracks the leader by clustering incoming LiDAR reflection points. It computes sharp distance changes (jumps) in the data arrays to isolate the leader's dynamic profile from static background environments (e.g., laboratory walls). 
* **Control Loop:** Employs a Proportional (P) controller mapping distance error to forward velocity. It uses Kalman Filter prediction models and structural orientation metrics to prevent target tracking loss during sharp turns, eliminating wrap-around indexing issues.

---

## How to Run the Infrastructure

### 1. Bring up the Hardware (or Simulation)
If you are working on the physical TurtleBot3 via SSH:
```bash
ros2 launch turtlebot3_bringup robot.launch.py

```

Or launch the multi-robot simulation workspace in Gazebo:

```bash
ros2 launch ./my_multi_robot.launch.py

```

### 2. Execute the Autonomous Driving Package

To test the reactive, LiDAR-safe random walk node:

```bash
ros2 run lab1pkg randwalk_safe_turtlebot3

```

### 3. Record and Plot Odometry Trajectories

To capture data during runtime and analyze performance metrics:

```bash
# Record the bag
ros2 bag record -o randWalk_Dataset /odom

# After parsing, run the visualization script inside your MATLAB console
showRandWalk.m

```
