import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class StopTurtle(Node):

    def __init__(self):
        super().__init__("stop_turtle_node")
        self.get_logger().info("StopTurtle Node has been started :)")
        self.cmd_vel_stop_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.stop_robot()

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        for n in range(1,10):
            self.cmd_vel_stop_.publish(cmd)
            self.get_logger().info(f'Publishing stop command: {n}/9')
            time.sleep(0.2)

def main(args=None):
    rclpy.init(args=args)
    node = StopTurtle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()