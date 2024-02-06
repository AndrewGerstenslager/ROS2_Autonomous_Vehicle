import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from math import atan2, sqrt

class WaypointExecutor(Node):
    def __init__(self):
        super().__init__('waypoint_executor')
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10)
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        self.current_goal = None

    def goal_callback(self, msg):
        # Set the current goal
        self.current_goal = msg.pose

    def calculate_and_publish_cmd_vel(self):
        # Calculate the required turn and publish to /cmd_vel
        pass

def main(args=None):
    rclpy.init(args=args)
    waypoint_executor = WaypointExecutor()
    rclpy.spin(waypoint_executor)
    waypoint_executor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
