import rclpy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from geographiclib.geodesic import Geodesic
import math
from simple_pid import PID

# Initialize the current heading and waypoint
current_heading = 0.0
current_waypoint = None

# Initialize the PID controller
pid = PID(1.0, 0.1, 0.05, setpoint=0)

def waypoint_callback(msg: NavSatFix):
    global current_waypoint
    current_waypoint = msg

def gps_callback(msg: NavSatFix):
    global current_waypoint
    if current_waypoint is None:
        return

    # Calculate the bearing to the target
    bearing = Geodesic.WGS84.Inverse(msg.latitude, msg.longitude, current_waypoint.latitude, current_waypoint.longitude)['azi1']

    # Adjust the bearing to match the robot's heading system
    bearing = (90 - bearing) % 360

    # Calculate the error
    error = bearing - current_heading

    # Set a constant turn speed
    turn_speed = 0.2 if error > 0 else -0.2

    # Publish a turn command
    twist = Twist()
    twist.angular.z = turn_speed
    cmd_vel_publisher.publish(twist)

    # If the robot is facing the waypoint, drive forward
    if abs(error) < 10.0:  # Allow for a small error
        twist = Twist()
        twist.linear.x = 0.5  # Adjust drive speed as needed
        cmd_vel_publisher.publish(twist)
        
def heading_callback(msg: Float64):
    global current_heading
    current_heading = msg.data

rclpy.init()
node = rclpy.create_node('basic_waypoint_follower')

# Create the publishers and subscribers
cmd_vel_publisher = node.create_publisher(Twist, '/cmd_vel', 10)
waypoint_subscription = node.create_subscription(NavSatFix, '/cur_waypoint', waypoint_callback, 10)
gps_subscription = node.create_subscription(NavSatFix, '/gps/fix', gps_callback, 10)
heading_subscription = node.create_subscription(Float64, '/heading_deg', heading_callback, 10)

rclpy.spin(node)