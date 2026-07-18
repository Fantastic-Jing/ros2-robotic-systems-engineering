import logging
from dataclasses import dataclass, field
import math
import time
from laser_processor import LaserProcessor

# CONFIGURE LOGGING HERE (Only once!)
logging.basicConfig(
    # DEBUG or INFO or WARNING or ERROR or CRITICAL
    level=logging.DEBUG, 
    format='%(asctime)s [%(levelname)s] (%(filename)s:%(lineno)d): %(message)s',
    datefmt='%H:%M:%S'
)

# Define the ROS 2 Header structure from scratch
@dataclass
class Header:
    stamp: dict = field(default_factory=lambda: {"sec": 0, "nanosec": 0})
    frame_id: str = "base_scan"

# Define the LaserScan structure mimicking sensor_msgs/msg/LaserScan
@dataclass
class LaserScan:
    header: Header = field(default_factory=Header)
    angle_min: float = 0.0
    angle_max: float = 0.0
    angle_increment: float = 0.0
    time_increment: float = 0.0
    scan_time: float = 0.0
    range_min: float = 0.0
    range_max: float = 0.0
    ranges: list = field(default_factory=list)
    intensities: list = field(default_factory=list)

def construct_minimal_scan():
    # Instantiate the blank object
    msg = LaserScan()
    
    # Get current timestamp (simulating ROS time)
    current_time = time.time()
    msg.header.stamp = {
        "sec": int(current_time),
        "nanosec": int((current_time - int(current_time)) * 1e9)
    }
    msg.header.frame_id = "base_scan"  # TurtleBot3 Burger scanner frame
    
    # Geometry metadata for 8 points (45 degree increments)
    msg.angle_min = 0.0
    msg.angle_increment = math.pi / 4  # 45 degrees in radians
    msg.angle_max = 7 * msg.angle_increment  # 7 steps = 8 points total
    
    # Laser properties
    msg.time_increment = 0.0
    msg.scan_time = 0.2
    msg.range_min = 0.12  # TurtleBot3 minimum LDS range (meters)
    msg.range_max = 3.5   # TurtleBot3 maximum LDS range (meters)
    
    # 8 raw simulated ranges (0°, 45°, 90°, 135°, 180°, 225°, 270°, 315°)
    msg.ranges = [1.0, 1.5, 2.0, 0.1, 1.0, 0.8, 0.5, float('nan')]
    msg.intensities = [0.0] * 8
    
    return msg

# --- Execution  ---
if __name__ == "__main__":
    
    logging.info("Starting the laser simulation application.")

    # Generate the raw scan message object
    scan_msg = construct_minimal_scan()
    
    # Check: Print individual metadata elements
    # print(f"Frame ID: {scan_msg.header.frame_id}")
    # print(f"Angle Increment: {scan_msg.angle_increment:.4f} rad")
    # print(f"Constructed Ranges: {scan_msg.ranges}")
    # print(f"Total Range Elements: {len(scan_msg.ranges)}")
    # print(f"{scan_msg=}")

    # Create an instance of your LaserProcessor class
    processor = LaserProcessor()
    
    # Call get_sector() and pass the scan object into it
    start_deg = -60.0
    end_deg = 60.0
    sector_result = processor.get_sector(scan_msg, start_deg, end_deg, mode="min")
    
    # Use or print the output
    #print("Successfully processed the scan!")
    print(f"Minimum distance between {start_deg}° and {end_deg}°: {sector_result}")

