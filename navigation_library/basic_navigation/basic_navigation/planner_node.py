import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
import json
import os
from ament_index_python.packages import get_package_share_directory
from geopy.distance import geodesic

class WaypointPlanner(Node):
    def __init__(self):
        super().__init__('waypoint_planner')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10)
        self.publisher = self.create_publisher(
            PointStamped,
            '/goal_point',
            10)
        self.waypoints = self.load_waypoints()
        self.current_waypoint_index = 0
        self.goal_reached_threshold = 2.0  # meters
        
        # Timer to continuously publish the current waypoint
        self.publish_timer = self.create_timer(1.0, self.publish_current_waypoint)  # adjust the interval as needed

    def gps_callback(self, msg):
        if self.current_waypoint_index < len(self.waypoints):
            current_waypoint = self.waypoints[self.current_waypoint_index]
            current_position = (msg.latitude, msg.longitude)
            distance = geodesic(current_position, (current_waypoint['latitude'], current_waypoint['longitude'])).meters
            
            if distance < self.goal_reached_threshold:  
                self.get_logger().info(f'Waypoint {current_waypoint["name"]} reached.')
                self.current_waypoint_index += 1
                if self.current_waypoint_index == len(self.waypoints):
                    self.get_logger().info('Final waypoint reached. Stopping waypoint publisher.')
                    self.publish_timer.cancel()  # Stop the timer if all waypoints have been reached

    def load_waypoints(self):
        package_share_directory = get_package_share_directory('basic_navigation')
        waypoints_file_path = os.path.join(package_share_directory, 'waypoints.json')
        
        with open(waypoints_file_path, 'r') as f:
            waypoints = json.load(f)
        self.get_logger().info(f'Loaded {len(waypoints)} waypoints.')
        return waypoints

    def publish_current_waypoint(self):
        if self.current_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint_index]
            goal_point_msg = PointStamped()
            goal_point_msg.header.frame_id = "gps"  # frame_id can be set to "gps" or "map" based on your TF configuration
            goal_point_msg.header.stamp = self.get_clock().now().to_msg()
            
            # Set the longitude and latitude
            goal_point_msg.point.x = waypoint['longitude']
            goal_point_msg.point.y = waypoint['latitude']
            goal_point_msg.point.z = 0.0  # Altitude can be set if needed
            
            self.publisher.publish(goal_point_msg)
            #self.get_logger().info(f'Publishing waypoint: {waypoint["name"]} with longitude: {waypoint["longitude"]} and latitude: {waypoint["latitude"]}.')

def main(args=None):
    rclpy.init(args=args)
    waypoint_planner = WaypointPlanner()
    rclpy.spin(waypoint_planner)
    waypoint_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
