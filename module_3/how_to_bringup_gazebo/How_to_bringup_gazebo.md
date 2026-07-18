### Phase 1: Launch Multi-Robot Simulation Environment

Open a new terminal, configure the TurtleBot3 hardware environment variable, and spin up the Gazebo empty world simulation containing 4 distinct robots:

```bash
export TURTLEBOT3_MODEL=burger
cd ~/ros2_ws
ros2 launch ./src/my_multi_robot.launch.py

```

### Phase 2: Purge Idle Entities from Simulation World

Open a second terminal. Call the Gazebo entity deletion service to remove the middle two idle units (`burger_2` and `burger_3`) to clear the LIDAR scanning horizon:

```bash
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'burger_2'}"
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'burger_3'}"

```

### Phase 3: Spin Up the Autonomous Follower Node

Open a third terminal. Run the tracking application to command the follower robot operating within the specified `/TB3_4` namespace:

```bash
cd ~/ros2_ws/src
python3 burger_follower.py

```

### Phase 4: Actuate Leader Node via Keyboard Teleoperation

Open a fourth terminal. Map the standard teleoperation package explicitly into the leader's `/TB3_1` routing namespace to drive the front target manually:

```bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r __ns:=/TB3_1

```
