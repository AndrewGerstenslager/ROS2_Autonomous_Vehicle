import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geographiclib.geodesic import Geodesic
import json
import os
import ament_index_python

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, 'cur_waypoint', 10)
        self.subscription = self.create_subscription(
            NavSatFix,
            'gps/fix',
            self.gps_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Load waypoints
        package_path = ament_index_python.get_package_share_directory('basic_navigation')
        with open(os.path.join(package_path, 'waypoints', 'waypoints.json')) as f:
            self.waypoints = json.load(f)['waypoints']
        self.current_waypoint_index = 0

        # Publish the initial waypoint
        self.publish_current_waypoint()

        # Create a timer to publish the current waypoint every second
        self.timer = self.create_timer(0.1, self.publish_current_waypoint)

    def publish_current_waypoint(self):
        if self.current_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint_index]
            waypoint_msg = NavSatFix()
            waypoint_msg.latitude = waypoint[0]
            waypoint_msg.longitude = waypoint[1]
            self.publisher_.publish(waypoint_msg)
            self.get_logger().info(f'Published waypoint {self.current_waypoint_index}: {waypoint_msg.latitude}, {waypoint_msg.longitude}')

    def gps_callback(self, msg):
        if self.current_waypoint_index >= len(self.waypoints):
            return

        # Calculate the distance to the current waypoint
        waypoint = self.waypoints[self.current_waypoint_index]
        distance = Geodesic.WGS84.Inverse(msg.latitude, msg.longitude, waypoint[0], waypoint[1])['s12']

        # If the robot is within 0.2 meters of the waypoint, move to the next waypoint
        if distance < 2:
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                self.get_logger().info(f'Moving to waypoint {self.current_waypoint_index}')

def main(args=None):
    rclpy.init(args=args)

    waypoint_publisher = WaypointPublisher()

    rclpy.spin(waypoint_publisher)

    waypoint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()