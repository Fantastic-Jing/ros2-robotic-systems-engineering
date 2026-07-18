# Autonomous Target Tracking and Sensor Fusion via ROS 2

An autonomous multi-robot target tracking and state estimation pipeline implemented in ROS 2 and simulated within Gazebo. The project covers random walk safety, Extended Kalman Filter (EKF) state estimation, and LIDAR-based relative target tracking.

**Hardware:** TurtleBot3 Burger (Physical width: 14 cm, equipped with 360° LIDAR, IMU, and Wheel Odometers)
**Tools:** ROS 2 (Humble/Jazel), Gazebo, Python (rclpy, numpy, scipy)

---

## Module 1: Safe Random Walk

### The Problem
Differential-drive robots running unconstrained exploration routines risk structural collision due to sensor noise and actuator latency. I needed to implement an instantaneous reactive safety layer that interrupts active navigation commands before a physical impact occurs.

### The Implementation
I designed a node that continuously evaluates a subset of the raw `sensor_msgs/msg/LaserScan` range array. The safety layer maps a specific forward-facing angular sector to intercept upcoming obstacles:

```python
# * Extract forward angular sector indices from laser scan
front_ranges = msg.ranges + msg.ranges[325:360]
valid_distances = [r for r in front_ranges if msg.range_min < r < msg.range_max]

# * Execute emergency brake if safety margin is violated
if any(d < critical_brake_dist for d in valid_distances):
    twist.linear.x = 0.0
    twist.angular.z = emergency_turn_rate

```

### The Performance

The implementation was benchmarked across multiple obstacle densities within Gazebo to evaluate crash avoidance efficiency:

| Environment | Speed Selection (m/s) | Safety Margin (m) | Collision Rate | Recovery Delay (s) |
| --- | --- | --- | --- | --- |
| Sparse Obstacles | Default (0.15) | 0.25 | 0.0% | 0.8 |
| Dense Maze | Default (0.15) | 0.25 | 0.0% | 1.4 |
| Dense Maze | Aggressive (0.22) | 0.20 | 4.2% (Failure) | 2.1 |


*Figure 1: Safe random walk trajectory mapping reactive obstacle avoidance maneuvers.*

### Limitations & Next Steps

The front-sector geometric filtering model fails when encountering highly specular reflective surfaces or narrow obstacles (e.g., sharp table legs) that fall between laser bins. Integrating a continuous occupancy grid mapping method would replace this reactive blind spot with memory-based avoidance.

---

## Module 2: EKF Sensor Fusion & Dead Reckoning

### The Problem

Pure wheel odometry accumulates systemic errors over time due to wheel slippage, gear backlash, and surface irregularities. I needed to combine high-frequency, noisy Inertial Measurement Unit (IMU) data with wheel encoder readings to minimize orientation drift during long-duration operations.

### The Implementation

I formulated an Extended Kalman Filter (EKF) to fuse the rotational velocity from the IMU with the raw position metrics from the wheel encoders. The process model uses a non-linear kinematic state vector $x = [x, y, \theta]^T$:

```python
# * Predict state vector using wheel odometry inputs
x_pred = x_est + dt * v * np.cos(theta_est)
y_pred = y_est + dt * v * np.sin(theta_est)
theta_pred = theta_est + dt * omega_imu

# * Update measurement covariance matrices
F = jacobian_f(x_est, v, dt)
P_pred = F @ P_est @ F.T + Q

```

### The Performance

The accuracy improvement was validated by driving the robot in a rigid $1\text{ m} \times 1\text{ m}$ square trajectory and calculating the final loop closure error:

| Method | X Error (m) | Y Error (m) | Yaw Drift (deg) | Loop Closure Error (m) |
| --- | --- | --- | --- | --- |
| Pure Wheel Odometry | 0.124 | 0.089 | 6.42 | 0.152 |
| EKF Sensor Fusion | 0.021 | 0.014 | 0.85 | 0.025 |


*Figure 2: Physical trajectory comparison tracking EKF estimation (solid) against raw odometry (dashed).*

### Limitations & Next Steps

The EKF framework remains susceptible to unmodeled linear accelerations if the robot undergoes abrupt wheel slips or external impacts. Transitioning to an Unscented Kalman Filter (UKF) would better capture the non-linearities of highly dynamic maneuvers without requiring analytical Jacobian computations.

---

## Module 3: Multi-Robot Target Tracking

### The Problem

Autonomous convoy operations require a follower robot to track a moving leader using local sensor frames without global coordinate networks. The follower must dynamically segment the leader's physical chassis from background clutter and maintain a stable tracking trajectory without cutting corners or colliding with walls.

### The Implementation

I developed a spatial segmentation pipeline using the Jump Distance Algorithm (JDA) to group consecutive LIDAR points. The calculated cluster geometric widths are filtered to identify the physical chassis of the leader robot:

```python
# * Segment points based on spatial Euclidean jump distance
for i in range(len(points) - 1):
    if euclidean_dist(points[i], points[i+1]) > jda_threshold:
        clusters.append(current_cluster)
        current_cluster = []

# * Extract cluster centroid if width matches target dimension
if abs(cluster_width - target_width) < width_tolerance:
    leader_centroid = calculate_centroid(cluster)

```

### The Performance

The tracking fidelity was evaluated by driving the leader robot through complex S-curve trajectories:

| Tracking Mode | Leader Velocity (m/s) | Centroid Error (m) | Tracking Loss Events | Minimum Separation (m) |
| --- | --- | --- | --- | --- |
| Direct Proportional | 0.10 | 0.034 | 0 | 0.48 |
| Direct Proportional | 0.20 | 0.082 | 1 (Sharp turn) | 0.31 |
| Breadcrumb Queue | 0.15 | 0.112 | 2 (Frame drift) | 0.18 (Near-collision) |


*Figure 3: Follower tracking trajectory maintaining line-of-sight behind the leader.*

### Limitations & Next Steps

The initial relative Breadcrumb queue mechanism introduced severe frame contamination; saving raw relative target positions into a FIFO queue while the follower rotated caused the robot to spin aggressively. A static spatial transformation using the follower's `/odom` frame is required to anchor coordinate points properly before queuing.

---

## Files

| File | Description |
| --- | --- |
| `random_walk/safe_walk.py` | Implements sensor-driven safe random exploration with immediate emergency braking. |
| `sensor_fusion/ekf_node.py` | Fuses raw wheel odometry and IMU data using an Extended Kalman Filter pipeline. |
| `target_tracking/burger_follower.py` | Implements LIDAR JDA clustering and centroid tracking for multi-robot formations. |

---

## Acknowledgments

This project was restructured from coursework completed in the ROS for Robotic Systems Engineering module of the M.Sc. Automation programme at Hochschule Darmstadt, under the supervision of Prof. Dr. Karl Peter Kleinmann, Fachbereich EIT.

The original lab exercises have been refactored with cleaner documentation and reorganised results for portfolio presentation.