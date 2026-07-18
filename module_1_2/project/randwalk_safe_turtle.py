import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import random
import time


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

        # --- State machine ---
        # 'WALK'       -> normal random walk, watching for obstacles
        # 'BACKING_UP' -> moving straight back/forward away from the obstacle
        # 'TURNING'    -> spinning in place to face a new direction
        self.state = 'WALK'
        self.state_start_time = None

        # Durations for the escape maneuver (seconds)
        self.backup_duration = 1.5
        self.turn_duration = 1.5

        # Escape speeds
        self.backup_speed = 0.08        # magnitude, direction chosen at trigger time
        self.turn_speed = 0.6           # magnitude, direction randomized at trigger time
        self.escape_linear = 0.0        # signed value used while BACKING_UP
        self.escape_angular = 0.0       # signed value used while TURNING

        # Internal states for the normal random walk
        self.random_linear = 0.0
        self.random_angular = 0.0

        # Safety threshold
        self.safety_distance = 0.2

        self.get_logger().info('RandwalkSafeTurtle Node has been started.')

    # ------------------------------------------------------------------
    # Timer: only relevant while in WALK state. Picks a new random target
    # every 2 seconds so the robot wanders forward/backward and turns.
    # ------------------------------------------------------------------
    def timer_callback(self):
        if self.state == 'WALK':
            self.random_linear = random.uniform(-0.08, 0.08) #forward OR backward   12056
            self.random_angular = random.uniform(-0.5, 0.5)

    # ------------------------------------------------------------------
    # Helper: extract + clean a window of ranges from the scan.
    # ------------------------------------------------------------------
    def get_valid_min_distance(self, msg, center_index, half_width=15):
        n = len(msg.ranges)
        indices = [(center_index + i) % n for i in range(-half_width, half_width + 1)]
        window = [msg.ranges[i] for i in indices]
        valid = [r for r in window if msg.range_min < r < msg.range_max]
        return min(valid) if valid else None

    # ------------------------------------------------------------------
    # Scan callback: publishes /cmd_vel every time, based on current state.
    # ------------------------------------------------------------------
    def scan_callback(self, msg):
        now = time.time()

        if self.state == 'WALK':
            # Decide which side to watch based on the direction we are
            # currently commanding (sign of random_linear).
            # index 0 = front, index ~ n/2 = rear (180 degrees).
            n = len(msg.ranges)
            if self.random_linear >= 0:
                watch_index = 0           # moving forward -> watch front
            else:
                watch_index = n // 2      # moving backward -> watch rear

            min_dist = self.get_valid_min_distance(msg, watch_index)

            if min_dist is not None and min_dist < self.safety_distance:
                self.get_logger().warn(
                    f'Obstacle at {min_dist:.2f} m '
                    f'({"front" if watch_index == 0 else "rear"}). '
                    f'Starting backup maneuver.'
                )
                # Move away from whatever we were approaching
                self.escape_linear = -self.backup_speed if self.random_linear >= 0 else self.backup_speed
                self.state = 'BACKING_UP'
                self.state_start_time = now

                cmd = Twist()
                cmd.linear.x = self.escape_linear
                cmd.angular.z = 0.0
                self.publisher_.publish(cmd)
                return

            # Path is clear: publish the regular random walk speeds
            cmd = Twist()
            cmd.linear.x = self.random_linear
            cmd.angular.z = self.random_angular
            self.publisher_.publish(cmd)

        elif self.state == 'BACKING_UP':
            cmd = Twist()
            cmd.linear.x = self.escape_linear
            cmd.angular.z = 0.0
            self.publisher_.publish(cmd)

            if now - self.state_start_time >= self.backup_duration:
                # Done backing up -> pick a random turn direction and turn
                self.escape_angular = random.choice([-1.0, 1.0]) * self.turn_speed
                self.state = 'TURNING'
                self.state_start_time = now
                self.get_logger().info('Backup complete. Turning to a new direction.')

        elif self.state == 'TURNING':
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = self.escape_angular
            self.publisher_.publish(cmd)

            if now - self.state_start_time >= self.turn_duration:
                # Done turning -> resume normal random walk
                self.state = 'WALK'
                self.get_logger().info('Turn complete. Resuming random walk.')

    # ------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = RandwalkSafeTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()