import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import random

class RandwalkSafeTurtle(Node):

    def __init__(self):
        super().__init__('randwalk_safe_turtle_node')
        
        # 1. Velocity Publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 2. Configure the specific QoS profile requested by the lab manual
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 3. Laser Scanner Subscriber
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        
        # 4. Timer for changing random velocities every 2 seconds
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        # Internal states
        self.obstacle_detected = False
        self.random_linear = 0.0
        self.random_angular = 0.0
        
        self.get_logger().info('RandwalkSafeTurtle Node has been started.')

    def timer_callback(self):
        # Only update random velocities when the path ahead is clear
        if not self.obstacle_detected:
            self.random_linear = random.uniform(0.06, 0.12)  # Slow forward
            self.random_angular = random.uniform(-0.4, 0.4)  # Slow turn

    def scan_callback(self, msg):
        # Extract ranges from the front window (e.g., 0 to 15 degrees and 345 to 360 degrees)
        # msg.ranges typically has 360 elements for a 360-degree LIDAR
        front_ranges = msg.ranges[0:15] + msg.ranges[345:360]
        
        # Filter out invalid readings (0.0 or inf) using range limits
        valid_ranges = [r for r in front_ranges if msg.range_min < r < msg.range_max]
        
        # Check if any valid obstacle distance is closer than 0.3 meters (30 cm)
        if valid_ranges and min(valid_ranges) < 0.3:
            if not self.obstacle_detected:
                self.get_logger().warn('Obstacle ahead! Executing safety stop and turn.')
            self.obstacle_detected = True
            
            # Emergency maneuver: Stop moving forward, spin on the spot to escape
            emergency_cmd = Twist()
            emergency_cmd.linear.x = 0.0
            emergency_cmd.angular.z = 0.5  # Spin left
            self.publisher_.publish(emergency_cmd)
        else:
            self.obstacle_detected = False
            # Path is clear, publish the regular random walk speeds
            normal_cmd = Twist()
            normal_cmd.linear.x = self.random_linear
            normal_cmd.angular.z = self.random_angular
            self.publisher_.publish(normal_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RandwalkSafeTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
