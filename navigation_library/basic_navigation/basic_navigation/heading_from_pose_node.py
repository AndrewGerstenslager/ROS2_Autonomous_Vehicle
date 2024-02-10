import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf_transformations import euler_from_quaternion

class OdomHeadingNode(Node):
    def __init__(self):
        super().__init__('odom_heading_node')
        # Subscriber for the odometry
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Publisher for the heading
        self.publisher_ = self.create_publisher(Float64, 'heading', 10)

    def odom_callback(self, msg):
        # Extract the quaternion from the odometry message
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        # Convert the quaternion to Euler angles
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        # Convert yaw to degrees
        yaw_degrees = yaw * (180.0 / 3.141592653589793)
        heading = (yaw_degrees + 360) % 360  # Convert yaw to 0 - 360 degrees

        # Create a Float64 message and publish the heading
        heading_msg = Float64()
        heading_msg.data = heading
        self.publisher_.publish(heading_msg)
        #self.get_logger().info('Published Heading: {:.2f} degrees'.format(heading))

def main(args=None):
    rclpy.init(args=args)
    odom_heading_node = OdomHeadingNode()
    rclpy.spin(odom_heading_node)
    odom_heading_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
