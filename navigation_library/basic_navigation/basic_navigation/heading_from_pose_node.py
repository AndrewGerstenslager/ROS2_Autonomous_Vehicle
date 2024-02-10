import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import NavSatFix
from math import atan2, radians, degrees, pi

class WaypointExecutor(Node):
    def __init__(self):
        super().__init__('waypoint_executor')
        self.goal_subscriber = self.create_subscription(
            PointStamped,
            '/goal_point',
            self.goal_callback,
            10)
        self.position_subscriber = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.position_callback,
            10)
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        self.current_goal = None
        self.current_position = None
        self.target_heading = None  # Degrees

        self.create_timer(0.1, self.calculate_and_publish_cmd_vel)

    def goal_callback(self, msg):
        self.current_goal = (msg.point.y, msg.point.x)  # Latitude, Longitude

    def position_callback(self, msg):
        self.current_position = (msg.latitude, msg.longitude)  # Latitude, Longitude
        # Calculate target heading when we get a new position
        if self.current_goal is not None:
            self.target_heading = self.calculate_target_heading()

    def calculate_target_heading(self):
        # Convert current position and goal to radians for calculation
        lat1, lon1 = radians(self.current_position[0]), radians(self.current_position[1])
        lat2, lon2 = radians(self.current_goal[0]), radians(self.current_goal[1])

        # Calculate goal bearing in radians
        y = atan2(lon2 - lon1, lat2 - lat1)

        # Convert goal bearing from radians to degrees
        target_heading_degrees = degrees(y) % 360

        return target_heading_degrees

    def calculate_and_publish_cmd_vel(self):
        if self.current_goal is None or self.current_position is None or self.target_heading is None:
            return  # Do nothing if we don't have goal, position, or target heading

        # Calculate difference between current heading and goal bearing
        angle_diff = (self.target_heading - self.current_heading + 360) % 360

        # Determine the shortest direction to turn (left or right)
        if angle_diff > 180:
            angle_diff -= 360

        # Apply proportional control to determine turn rate
        angular_velocity = 0.005 * -angle_diff  # Tune this gain

        # Create and publish Twist message
        cmd_vel = Twist()
        cmd_vel.angular.z = angular_velocity
        # Reduce speed as the robot aligns with the goal bearing
        cmd_vel.linear.x = max(0.2, 0.5 - abs(angular_velocity) / 2)  # Tune these values

        self.cmd_vel_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    waypoint_executor = WaypointExecutor()
    rclpy.spin(waypoint_executor)
    waypoint_executor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()