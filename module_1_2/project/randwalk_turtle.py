import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random
import time

class RandwalkTurtle(Node):

    def __init__(self):
        super().__init__("randwalk_turtle_node")
        self.get_logger().info("randwalk_turtle_node has been started :)")
        self.cmd_vel_randwalk_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)

        self.get_logger().info("randwalk_turtle_node has been ended :)")

    def timer_callback(self):
        self.get_logger().info("im here :)")

        cmd = Twist()
        cmd.linear.x = random.uniform(0.05, 0.12)
        cmd.angular.z = random.uniform(-0.4, 0.4)

        self.cmd_vel_randwalk_.publish(cmd)
        self.get_logger().info(f'randwalk linear: {cmd.linear.x:.2f}, angular:{cmd.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = RandwalkTurtle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()