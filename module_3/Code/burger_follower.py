import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import math
import os
import csv
import time
import atexit
from datetime import datetime

# * ============================================================
# * Algorithm Overview (Leader-Follower via LIDAR clustering)
# * ============================================================
# * 1. Raw LaserScan (polar: range + angle per ray) is cleaned of
# *    inf/nan and out-of-range returns (preprocess_scan).
# * 2. Each remaining ray is converted from polar (r, theta) to
# *    Cartesian (x, y) in the Follower's own sensor frame, where
# *    x points straight ahead and y points to the Follower's left
# *    (REP103 convention). This is required because clustering by
# *    Euclidean distance only makes sense in Cartesian space, not
# *    in raw angle/range space (polar_to_cartesian).
# * 3. Jump Distance Algorithm (JDA) groups consecutive rays into
# *    clusters: two neighboring points belong to the same physical
# *    object if the straight-line distance between them is below
# *    jda_threshold. A large gap means the beam moved from one
# *    object's edge to empty space or a different object
# *    (segment_clusters). Because the scan is a closed ring, the
# *    first and last rays are also checked against each other so a
# *    single object straddling the 0/360 degree seam is not split
# *    into two clusters.
# * 4. Among all clusters, the Leader is identified purely by its
# *    physical footprint: the TurtleBot3 Burger chassis is about
# *    0.14 m wide, so a cluster whose bounding-box diagonal is
# *    close to that value (within width_tolerance) is treated as
# *    the Leader; everything else (walls, noise, other robots) is
# *    ignored (extract_leader_centroid). The mean of the cluster's
# *    points is used as the Leader's estimated position.
# * 5. A P (proportional) controller converts the Leader's relative
# *    position into velocity commands: linear.x closes the gap
# *    between the current distance and desired_distance, angular.z
# *    turns the Follower to zero out the bearing angle to the
# *    Leader (execute_control). Both outputs are clamped to the
# *    Burger's physical speed limits.
# * 6. If no cluster matches the Leader's footprint, the Follower
# *    stops immediately (stop_robot) rather than guessing.

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

        # * Prints every tuning knob once at start-up so a run's exact
        # * configuration is always visible in the console/log, not just
        # * buried in the source file
        param_lines = [
            '=' * 50,
            'BurgerFollower Parameters',
            '=' * 50,
            f'  JDA threshold      : {self.jda_threshold:.2f} m',
            f'  Target width       : {self.target_width:.2f} m (tolerance +/- {self.width_tolerance:.2f} m)',
            f'  Desired distance   : {self.desired_distance:.2f} m',
            f'  Kp linear          : {self.kp_linear:.2f}',
            f'  Kp angular         : {self.kp_angular:.2f}',
            f'  Max linear speed   : {self.max_linear_speed:.2f} m/s',
            f'  Max angular speed  : {self.max_angular_speed:.2f} rad/s',
            '=' * 50,
        ]
        for line in param_lines:
            self.get_logger().info(line)

        self.get_logger().info('Optimized BurgerFollower node initialized.')

        # * ------------------------------------------------------
        # * Timestamped Data Logging (added for post-run analysis)
        # * ------------------------------------------------------
        # * Every scan_callback appends one row of runtime telemetry
        # * to a CSV file whose name is fixed at node start-up, so a
        # * full run's data lands in a single file for later plotting
        # * or debugging. This is purely observational: it does not
        # * feed back into any control decision above.
        # * The log directory sits next to this script file rather
        # * than under os.path.expanduser('~'), because the Follower
        # * may be launched under a different user/sudo context on
        # * real hardware, where '~' would silently resolve to a
        # * different home directory than the one being checked.
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'burger_follower_logs')
        os.makedirs(log_dir, exist_ok=True)
        run_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_path = os.path.join(log_dir, f'burger_follower_{run_timestamp}.csv')
        self.log_file = open(self.log_path, 'w', newline='')
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow([
            'timestamp', 'num_valid_points', 'num_clusters',
            'leader_found', 'reason', 'target_x', 'target_y',
            'distance', 'distance_error', 'angle_error_deg',
            'cmd_linear_x', 'cmd_angular_z'
        ])
        # * Flush right after the header too, not just after data rows,
        # * so the file is never left completely empty on disk
        self.log_file.flush()

        # * Safety net: closes the file on interpreter exit even if
        # * destroy_node() is skipped by an unexpected shutdown path
        atexit.register(self._close_log_file)

        self.get_logger().info(f'Logging telemetry to {self.log_path}')
        print(f'[BurgerFollower] Logging telemetry to {self.log_path}')

        # * Bookkeeping updated each scan_callback so execute_control /
        # * stop_robot can log the frame's clustering stats alongside
        # * their own control-specific numbers
        self.last_num_valid_points = 0
        self.last_num_clusters = 0

        # * Real-Time Console Feedback
        # * last_state: tracks the previous frame's outcome so a
        # *   one-line event message only fires on a transition
        # *   (e.g. tracking -> lost), not on every single frame
        # * print_interval: minimum seconds between throttled status
        # *   lines while actively tracking, to stay readable at 10 Hz
        self.last_state = None
        self.print_interval = 1.0
        self.last_print_time = 0.0
        self.last_env_print_time = 0.0

    def wrap_to_pi(self, angle):
        # * Normalizes angles within the safe range [-pi, pi]
        # * Prevents erratic steering spinning when crossing the 180-degree boundary
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def announce_state(self, state, message):
        # * Prints a single readable line only when the state actually
        # * changes (tracking -> lost, lost -> tracking, etc.), so the
        # * console stays readable instead of repeating every frame
        if state != self.last_state:
            self.get_logger().info(f'[EVENT] {message}')
            self.last_state = state

    def maybe_print_status(self, distance, distance_error, angle_error_deg, cmd):
        # * Throttled real-time status line for observing the run
        # * without flooding the console at the scan's native rate
        now = time.monotonic()
        if now - self.last_print_time >= self.print_interval:
            self.get_logger().info(
                f'[STATUS] dist={distance:.2f}m derr={distance_error:+.2f}m '
                f'aerr={angle_error_deg:+.1f}deg lin={cmd.linear.x:+.2f} '
                f'ang={cmd.angular.z:+.2f}'
            )
            self.last_print_time = now

    def print_environment_snapshot(self, clusters):
        # * Purely observational: reports every cluster the LIDAR
        # * currently sees (how many points, how wide, which bearing),
        # * whether or not it matched the Leader. This lets you confirm
        # * the sensor and clustering are actually working even during
        # * stretches where no tracking/lost event fires. Nothing here
        # * is written to the CSV log.
        now = time.monotonic()
        if now - self.last_env_print_time < self.print_interval:
            return
        self.last_env_print_time = now

        if not clusters:
            self.get_logger().info('[ENV] no clusters detected')
            return

        for idx, cluster in enumerate(clusters):
            p_min = np.min(cluster, axis=0)
            p_max = np.max(cluster, axis=0)
            width = math.sqrt((p_max[0] - p_min[0])**2 + (p_max[1] - p_min[1])**2)
            centroid = np.mean(cluster, axis=0)
            bearing_deg = math.degrees(math.atan2(centroid[1], centroid[0]))
            self.get_logger().info(
                f'[ENV] cluster{idx}: pts={cluster.shape[0]} '
                f'width={width:.2f}m bearing={bearing_deg:+.1f}deg'
            )

    def log_telemetry(self, num_valid_points, num_clusters, leader_found,
                       reason='', target_x=None, target_y=None, distance=None,
                       distance_error=None, angle_error_deg=None, cmd=None):
        # * Writes one CSV row of runtime data, timestamped to the
        # * millisecond, for later plotting or exam-demo evidence.
        # * All fields are optional except the three that are always
        # * known (point/cluster counts, whether the Leader was found).
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        self.log_writer.writerow([
            timestamp,
            num_valid_points,
            num_clusters,
            leader_found,
            reason,
            target_x if target_x is not None else '',
            target_y if target_y is not None else '',
            distance if distance is not None else '',
            distance_error if distance_error is not None else '',
            angle_error_deg if angle_error_deg is not None else '',
            cmd.linear.x if cmd is not None else '',
            cmd.angular.z if cmd is not None else ''
        ])
        self.log_file.flush()

    def _close_log_file(self):
        # * Idempotent close so it is safe to call from both
        # * destroy_node() and the atexit safety net
        if hasattr(self, 'log_file') and not self.log_file.closed:
            self.log_file.close()

    def destroy_node(self):
        # * Ensures the telemetry CSV is closed cleanly on shutdown
        self._close_log_file()
        super().destroy_node()

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
        # * ----------------------------------------------------------
        # * Principle: Jump Distance Algorithm (JDA)
        # * A LIDAR scan sees the world as one continuous ring of rays.
        # * Points that belong to the same physical object sit close to
        # * each other in Cartesian space because the object's surface
        # * is continuous; points from different objects (or an object
        # * and empty background) are separated by a visible "jump" in
        # * distance. Scanning once through the ring and cutting a new
        # * cluster wherever that jump exceeds jda_threshold recovers
        # * the objects in a single O(N) pass, with no prior knowledge
        # * of how many objects are present.
        # * ----------------------------------------------------------
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
        # * ----------------------------------------------------------
        # * Principle: Identification by physical footprint, not by
        # * position or motion. The Follower has no prior belief about
        # * where the Leader is, so it cannot rely on "closest cluster"
        # * or "cluster that moved". Instead it exploits a fixed,
        # * known fact: the Leader is a TurtleBot3 Burger, roughly
        # * 0.14 m across. Any cluster whose bounding-box diagonal
        # * falls within that width band is accepted as the Leader;
        # * this is why walls (too wide) and sensor noise specks (too
        # * few points, filtered separately) are naturally rejected.
        # * ----------------------------------------------------------
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
        # * ----------------------------------------------------------
        # * Principle: Proportional (P) control on two independent
        # * errors. Distance error (how far the Follower is from its
        # * desired_distance behind the Leader) drives linear.x;
        # * bearing error (how far off-center the Leader appears in
        # * the Follower's frame) drives angular.z. Because the two
        # * errors are computed from the same centroid but act on
        # * different actuators, the Follower can simultaneously close
        # * the gap and turn to face the Leader, converging on both at
        # * once rather than sequentially.
        # * ----------------------------------------------------------
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

        # * Real-time console feedback: one-shot event on (re)acquiring
        # * the Leader, plus a throttled ongoing status line
        self.announce_state('tracking', 'Leader acquired -> tracking')
        self.maybe_print_status(current_distance, linear_error, math.degrees(angular_error), cmd)

        # * Records this control cycle's numbers for the telemetry log
        self.log_telemetry(
            num_valid_points=self.last_num_valid_points,
            num_clusters=self.last_num_clusters,
            leader_found=True,
            reason='tracking',
            target_x=target_x,
            target_y=target_y,
            distance=current_distance,
            distance_error=linear_error,
            angle_error_deg=math.degrees(angular_error),
            cmd=cmd
        )

    def stop_robot(self, reason='no_leader_match'):
        # * Safety state: publishes zero velocities immediately to prevent runaway crashes
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publisher_.publish(cmd)

        # * Real-time console feedback: one-shot event line, keyed by
        # * reason so "no leader" and "bad scan" read differently
        if reason == 'empty_or_invalid_scan':
            self.announce_state('invalid_scan', 'Invalid/empty scan -> stopping')
        else:
            self.announce_state('lost_no_match', 'Leader lost -> stopping')

        # * Records the stop event so the log shows exactly when and
        # * why the Follower went idle
        self.log_telemetry(
            num_valid_points=self.last_num_valid_points,
            num_clusters=self.last_num_clusters,
            leader_found=False,
            reason=reason,
            cmd=cmd
        )

    def scan_callback(self, msg):
        # * Pipeline Step 1: Data cleansing
        valid_ranges, valid_angles = self.preprocess_scan(msg)
        if valid_ranges is None:
            self.last_num_valid_points = 0
            self.last_num_clusters = 0
            self.stop_robot(reason='empty_or_invalid_scan')
            return

        # * Pipeline Step 2: Coordinate transformation
        points = self.polar_to_cartesian(valid_ranges, valid_angles)

        # * Pipeline Step 3: Spatial JDA segmentation
        clusters = self.segment_clusters(points)
        self.last_num_valid_points = points.shape[0]
        self.last_num_clusters = len(clusters)

        # * Environment awareness: reports what the sensor currently
        # * sees regardless of whether it matches the Leader or not
        self.print_environment_snapshot(clusters)

        # * Pipeline Step 4: Geometric target filtering
        leader_centroid = self.extract_leader_centroid(clusters)

        # * Pipeline Step 5: Proportional law calculation
        if leader_centroid is not None:
            self.execute_control(leader_centroid)
        else:
            self.stop_robot(reason='no_leader_match')

def main(args=None):
    rclpy.init(args=args)
    node = BurgerFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # * Ctrl+C: fall through to the finally block so the log file
        # * is still closed properly instead of being skipped
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()