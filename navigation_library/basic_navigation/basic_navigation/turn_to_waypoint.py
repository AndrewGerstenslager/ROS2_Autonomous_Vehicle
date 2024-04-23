import rclpy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from geographiclib.geodesic import Geodesic
import math
from rclpy.node import Node
from simple_pid import PID


class TurnToWaypointNode(Node):
    def __init__(self):
        super().__init__('turn_to_waypoint')

        # Create the publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gps_subscription = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.heading_subscription = self.create_subscription(Float64, '/heading_deg', self.heading_callback, 10)
        self.waypoint_subscription = self.create_subscription(NavSatFix, '/cur_waypoint', self.waypoint_callback, 10)

        self.current_heading = 0.0
        self.current_waypoint = None

        # Initialize the PID controller
        self.pid = PID(1.0, 0.1, 0.05, setpoint=0)

        
    def waypoint_callback(self, msg: NavSatFix):
        self.current_waypoint = msg

    def gps_callback(self, msg: NavSatFix):
        if self.current_waypoint is None:
            return

        # Calculate the bearing to the target
        bearing = Geodesic.WGS84.Inverse(msg.latitude, msg.longitude, self.current_waypoint.latitude, self.current_waypoint.longitude)['azi1']

        # Adjust the bearing to match the robot's heading system
        bearing = -bearing

        # Set the setpoint of the PID controller to the desired bearing
        self.pid.setpoint = bearing

        # Use the output of the PID controller to control the turning speed
        turn_speed = self.pid(self.current_heading)

        # Limit the top turning speed to 0.5
        if turn_speed > 0.5:
            turn_speed = 0.5
        elif turn_speed < -0.5:
            turn_speed = -0.5

        twist = Twist()
        twist.angular.z = turn_speed
        self.cmd_vel_publisher.publish(twist)

        # If the robot is facing the goal, publish a forward speed
        if abs(bearing - self.current_heading) <= 10.0:  # Allow for a small error
            twist = Twist()
            twist.linear.x = 0.5  # Adjust forward speed as needed
            self.cmd_vel_publisher.publish(twist)

    def heading_callback(self, msg: Float64):
        self.current_heading = msg.data

def main(args=None):
    rclpy.init(args=args)

    node = TurnToWaypointNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()