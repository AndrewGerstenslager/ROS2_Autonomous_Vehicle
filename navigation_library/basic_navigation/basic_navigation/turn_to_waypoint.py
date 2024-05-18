import rclpy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from geographiclib.geodesic import Geodesic
import math
from rclpy.node import Node

class TurnToWaypointNode(Node):
    def __init__(self):
        super().__init__('turn_to_waypoint')

        # Create the publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bearing_subscription = self.create_subscription(Float64, '/bearing_deg', self.bearing_callback, 10)
        self.heading_subscription = self.create_subscription(Float64, '/heading_deg', self.heading_callback, 10)
        self.waypoint_subscription = self.create_subscription(NavSatFix, '/cur_waypoint', self.waypoint_callback, 10)

        self.current_heading = 0.0
        self.current_bearing = 0.0
        self.current_waypoint = None

    def waypoint_callback(self, msg: NavSatFix):
        self.current_waypoint = msg

    def bearing_callback(self, msg: Float64):
        self.current_bearing = msg.data
        self.turn_to_waypoint()

    def heading_callback(self, msg: Float64):
        self.current_heading = msg.data

    def turn_to_waypoint(self):
        if self.current_waypoint is None:
            self.get_logger().info(f'Waypoint is None')
            return

        # Set the turning speed proportional to the bearing
        divisor = 15
        turn_speed = math.sqrt(abs(self.current_bearing) / divisor) if self.current_bearing > self.current_heading else -math.sqrt(abs(self.current_bearing) / divisor)

        # Set the forward speed to a constant value
        forward_speed = 0.5

        twist = Twist()
        twist.angular.z = turn_speed
        twist.linear.x = forward_speed
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    node = TurnToWaypointNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()