import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from geographiclib.geodesic import Geodesic
import numpy as np

class BearingPublisher(Node):
    def __init__(self):
        super().__init__('bearing_publisher')
        self.gps_subscription = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.waypoint_subscription = self.create_subscription(NavSatFix, '/cur_waypoint', self.waypoint_callback, 10)
        self.bearing_pub = self.create_publisher(Float64, '/bearing_deg', 10)
        self.current_location = None
        self.current_waypoint = None

    def waypoint_callback(self, msg: NavSatFix):
        self.current_waypoint = msg

    def gps_callback(self, msg: NavSatFix):
        self.current_location = msg
        self.publish_bearing()

    def publish_bearing(self):
        if self.current_location and self.current_waypoint:
            result = Geodesic.WGS84.Inverse(self.current_location.latitude, self.current_location.longitude, self.current_waypoint.latitude, self.current_waypoint.longitude)
            bearing = result['azi1']
            bearing = -bearing  # Adjust the bearing to match the robot's heading system
            msg = Float64()
            msg.data = bearing
            self.bearing_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    bearing_publisher = BearingPublisher()
    rclpy.spin(bearing_publisher)

if __name__ == '__main__':
    main()