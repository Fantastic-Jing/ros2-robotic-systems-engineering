
# 1️⃣ **Connect to the TurtleBot3**

### **On your laptop (BlueDevel, RedDevel, etc.)**
```bash
ping bluebot
```
If you get replies → OK.  
Stop with **CTRL+C**.

### **SSH into the robot**
```bash
ssh -l obv bluebot
```
Password:
```
turtle01
```

You are now inside the robot’s Raspberry Pi (SBC).

---

# 2️⃣ **Start the TurtleBot3 bringup**

Inside the robot (SSH shell):

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

Leave this running.  
Open a **second terminal** on your laptop.

---

# 3️⃣ **Create your ROS2 workspace (on your laptop)**

If you don’t have a workspace yet:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
```

Source it:

```bash
source install/setup.bash
```

---

# 4️⃣ **Create Lab1 package**

Inside your workspace:

```bash
cd ~/ros2_ws/src
ros2 pkg create lab1pkg --build-type ament_python --dependencies rclpy geometry_msgs sensor_msgs
```

This creates:

```
lab1pkg/
  setup.py
  package.xml
  lab1pkg/
    __init__.py
```

---

# 5️⃣ **Create your Python scripts**

Go into the package:

```bash
cd ~/ros2_ws/src/lab1pkg/lab1pkg
```

Create **stop_turtle.py**:

```bash
nano stop_turtle.py
```

Paste this minimal working template:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class StopTurtle(Node):
    def __init__(self):
        super().__init__('stop_turtle_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("StopTurtle started")

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        for _ in range(10):
            self.pub.publish(cmd)
            time.sleep(0.2)

def main(args=None):
    rclpy.init(args=args)
    node = StopTurtle()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Save: **CTRL+O**, ENTER  
Exit: **CTRL+X**

---

### Add executable to setup.py

Open:

```bash
nano ~/ros2_ws/src/lab1pkg/setup.py
```

Add inside `entry_points`:

```python
'console_scripts': [
    'stop_turtlebot3 = lab1pkg.stop_turtle:main',
],
```

Save & exit.

---

# 6️⃣ **Build the workspace**

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

# 7️⃣ **Run your node on the robot**

### **On your laptop (NOT inside SSH):**

```bash
ros2 run lab1pkg stop_turtlebot3
```

This publishes zero velocity to `/cmd_vel` and stops the robot.
