import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from pyquaternion import Quaternion
import math

def odom_callback(msg: Odometry):
    # Get the orientation quaternion
    orientation_q = msg.pose.pose.orientation
    # Convert the quaternion to Euler angles
    quaternion = Quaternion(orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z)
    yaw, pitch, roll = quaternion.yaw_pitch_roll

    # Convert yaw to degrees
    yaw_degrees = math.degrees(yaw)

    # Publish the headings
    heading_deg_publisher.publish(Float64(data=yaw_degrees))
    heading_rad_publisher.publish(Float64(data=yaw))

rclpy.init()
node = rclpy.create_node('odom_listener')

# Create the publishers
heading_deg_publisher = node.create_publisher(Float64, '/heading_deg', 10)
heading_rad_publisher = node.create_publisher(Float64, '/heading_rad', 10)

# Create the subscription
subscription = node.create_subscription(Odometry, '/odom', odom_callback, 10)

rclpy.spin(node)