import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import math

class BurgerFollower(Node):

    def __init__(self):
        super().__init__('burger_follower_node')


        # ROS 2 Communications Setup

        self.publisher_ = self.create_publisher(Twist, '/TB3_4/cmd_vel', 10)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/TB3_4/scan',
            self.scan_callback,
            qos_profile
        )


        # Hyperparameters & Tuning Knobs

        
        # * JDA Clustering Threshold
        # * Unit: meters
        # * Higher value: prevents cluster splitting at long ranges
        # * Lower value: isolates close objects but risks splitting the target
        self.jda_threshold = 0.45       

        # * Target Identification Geometry
        # * Unit: meters
        # * target_width: physical size of the Leader chassis (14 cm)
        # * width_tolerance: maximum allowed deformation of the target size
        # * Higher tolerance: easier tracking but risks picking up environmental noise
        self.target_width = 0.14        
        self.width_tolerance = 0.15     

        # * Safety Boundary
        # * Unit: meters
        # * The exact distance the Follower attempts to maintain behind the Leader
        self.desired_distance = 0.5     
        
        # * P-Controller Linear Gain (Kp)
        # * Higher value: faster response to distance changes, potential overshoot
        # * Lower value: smoother approach, sluggish tracking performance
        self.kp_linear = 0.6            
        
        # * P-Controller Angular Gain (Kp)
        # * Higher value: snappier turning adjustments, risks steering oscillations
        # * Lower value: stable path alignment, risks losing target on sharp turns
        self.kp_angular = 1.5           
        
        # * Hardware Velocity Constraints
        # * Clamped values strictly matching TurtleBot3 Burger hardware ceilings
        self.max_linear_speed = 0.21    
        self.max_angular_speed = 2.63   

        self.get_logger().info('Optimized BurgerFollower node initialized.')

    def wrap_to_pi(self, angle):
        # * Normalizes angles within the safe range [-pi, pi]
        # * Prevents erratic steering spinning when crossing the 180-degree boundary
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def preprocess_scan(self, msg):
        # * Rejects empty raw LIDAR inputs immediately
        if not msg.ranges:
            return None, None

        ranges = np.asarray(msg.ranges)
        angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment
        
        # * Filters out physical out-of-bound sensor returns (inf, nan)
        valid_mask = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        if not np.any(valid_mask):
            return None, None

        return ranges[valid_mask], angles[valid_mask]

    def polar_to_cartesian(self, ranges, angles):
        # * Converts Polar inputs to 2D Cartesian spatial coordinates
        x_points = ranges * np.cos(angles)
        y_points = ranges * np.sin(angles)
        return np.column_stack((x_points, y_points))

    def segment_clusters(self, points):
        # * Jump Distance Algorithm core grouping mechanism
        clusters = []
        current_cluster = [points[0]]

        for i in range(1, points.shape[0]):
            prev_p = points[i - 1]
            curr_p = points[i]
            
            # * Calculates Euclidean distance between consecutive scanning rays
            dist = math.sqrt((curr_p[0] - prev_p[0])**2 + (curr_p[1] - prev_p[1])**2)
            
            if dist < self.jda_threshold:
                current_cluster.append(curr_p)
            else:
                clusters.append(np.array(current_cluster))
                current_cluster = [curr_p]
        clusters.append(np.array(current_cluster))

        # * Handles the 360-degree wrap-around ring closure boundary
        if len(clusters) > 1:
            first_p = clusters[0][0]
            last_p = clusters[-1][-1]
            wrap_dist = math.sqrt((first_p[0] - last_p[0])**2 + (first_p[1] - last_p[1])**2)
            if wrap_dist < self.jda_threshold:
                clusters[0] = np.vstack((clusters[-1], clusters[0]))
                clusters.pop()

        return clusters

    def extract_leader_centroid(self, clusters):
        # * Isolates the Leader robot footprint via size filtering
        for cluster in clusters:
            # * Rejects small isolated noise sparks
            if cluster.shape[0] < 3:
                continue

            p_min = np.min(cluster, axis=0)
            p_max = np.max(cluster, axis=0)
            cluster_width = math.sqrt((p_max[0] - p_min[0])**2 + (p_max[1] - p_min[1])**2)

            # * Matches the tracked cluster against target width criteria
            if abs(cluster_width - self.target_width) <= self.width_tolerance:
                return np.mean(cluster, axis=0)
        return None

    def execute_control(self, centroid):
        # * Targets positional coordinates relative to the Follower's frame
        target_x = centroid[0]
        target_y = centroid[1]

        current_distance = math.sqrt(target_x**2 + target_y**2)
        target_angle = math.atan2(target_y, target_x)

        # * Evaluates distance and orientation tracking errors
        linear_error = current_distance - self.desired_distance
        angular_error = self.wrap_to_pi(target_angle)

        cmd = Twist()
        
        # * Applies proportional amplification and limits outputs to protect actuators
        linear_out = self.kp_linear * linear_error
        cmd.linear.x = max(min(linear_out, self.max_linear_speed), -self.max_linear_speed)

        angular_out = self.kp_angular * angular_error
        cmd.angular.z = max(min(angular_out, self.max_angular_speed), -self.max_angular_speed)

        self.publisher_.publish(cmd)

    def stop_robot(self):
        # * Safety state: publishes zero velocities immediately to prevent runaway crashes
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publisher_.publish(cmd)

    def scan_callback(self, msg):
        # * Pipeline Step 1: Data cleansing
        valid_ranges, valid_angles = self.preprocess_scan(msg)
        if valid_ranges is None:
            self.stop_robot()
            return

        # * Pipeline Step 2: Coordinate transformation
        points = self.polar_to_cartesian(valid_ranges, valid_angles)

        # * Pipeline Step 3: Spatial JDA segmentation
        clusters = self.segment_clusters(points)

        # * Pipeline Step 4: Geometric target filtering
        leader_centroid = self.extract_leader_centroid(clusters)

        # * Pipeline Step 5: Proportional law calculation
        if leader_centroid is not None:
            self.execute_control(leader_centroid)
        else:
            self.stop_robot()

def main(args=None):
    rclpy.init(args=args)
    node = BurgerFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()