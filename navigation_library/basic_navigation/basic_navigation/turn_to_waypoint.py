import rclpy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from geographiclib.geodesic import Geodesic
import math

# Set the target GPS coordinate
target_lat = 38.161482658103715
target_lon = -122.45456117956125

def gps_callback(msg: NavSatFix):
    # Calculate the bearing to the target
    bearing = Geodesic.WGS84.Inverse(msg.latitude, msg.longitude, target_lat, target_lon)['azi1']

    # Adjust the bearing to match the robot's heading system
    bearing = -bearing

    # If the current heading does not match the bearing, publish a turn command
    if abs(bearing - current_heading) > 1.0:  # Allow for a small error
        twist = Twist()
        twist.angular.z = math.radians(bearing - current_heading)  # Adjust turn speed as needed
        cmd_vel_publisher.publish(twist)

def heading_callback(msg: Float64):
    global current_heading
    current_heading = msg.data

rclpy.init()
node = rclpy.create_node('turn_to_waypoint')

# Create the publishers and subscribers
cmd_vel_publisher = node.create_publisher(Twist, '/cmd_vel', 10)
gps_subscription = node.create_subscription(NavSatFix, '/gps/fix', gps_callback, 10)
heading_subscription = node.create_subscription(Float64, '/heading_deg', heading_callback, 10)

rclpy.spin(node)