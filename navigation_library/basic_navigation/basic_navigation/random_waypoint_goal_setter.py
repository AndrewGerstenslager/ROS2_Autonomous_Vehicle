import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geographiclib.geodesic import Geodesic
import json
import os
import ament_index_python
import random

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
        self.goal_met = True

        # Load bounding box
        package_path = ament_index_python.get_package_share_directory('basic_navigation')
        with open(os.path.join(package_path, 'waypoints', 'bounding_points.json')) as f:
            bounding_points = json.load(f)['bounding_points']
        self.min_lat, self.min_lon = bounding_points[0]
        self.max_lat, self.max_lon = bounding_points[1]

        # Generate and publish the initial waypoint
        self.current_goal = NavSatFix()
        self.generate_and_publish_waypoint()

        # Create a timer that calls generate_and_publish_waypoint every second
        self.timer = self.create_timer(1.0, self.generate_and_publish_waypoint)

    def generate_and_publish_waypoint(self):
        # Only generate a new waypoint if the current goal was met
        if self.goal_met:
            self.current_goal.latitude = random.uniform(self.min_lat, self.max_lat)
            self.current_goal.longitude = random.uniform(self.min_lon, self.max_lon)
            self.publisher_.publish(self.current_goal)
            self.get_logger().info(f'Published waypoint: {self.current_goal.latitude}, {self.current_goal.longitude}')
            self.goal_met = False

    def gps_callback(self, msg):
        # Calculate the distance to the current waypoint
        distance = Geodesic.WGS84.Inverse(msg.latitude, msg.longitude, self.current_goal.latitude, self.current_goal.longitude)['s12']

        # If the robot is within 0.3 meters of the waypoint, set goal_met to True
        if distance < 0.3:
            self.goal_met = True
            self.get_logger().info('Goal met, ready for new waypoint')


def main(args=None):
    rclpy.init(args=args)

    waypoint_publisher = WaypointPublisher()

    rclpy.spin(waypoint_publisher)

    waypoint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()