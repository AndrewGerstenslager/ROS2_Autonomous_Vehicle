import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from std_srvs.srv import Empty
from std_msgs.msg import String, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf_transformations
import tf2_ros
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        
        # Parameters
        self.declare_parameter('wheel_base', 0.5207)  # Distance between wheels in meters
        self.declare_parameter('wheel_diameter', 0.318)  # Diameter of the wheels in meters

        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.wheel_diameter = self.get_parameter('wheel_diameter').get_parameter_value().double_value
        self.circumference = self.wheel_diameter * math.pi

        # Odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left_wheel_pos = 0.0
        self.prev_right_wheel_pos = 0.0

        # Create publisher and subscriber
        self.raw_data_sub = self.create_subscription(String, 'raw_data', self.raw_data_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', QoSProfile(depth=10))
        self.wheel_pos_pub = self.create_publisher(Float32MultiArray, 'wheel_positions', 10)
        
        # Create tf broadcaster
        self.odom_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Create reset service
        self.create_service(Empty, 'reset_odometry', self.reset_odometry_callback)
        
        self.get_logger().info("Odometry Node has been started")

    def raw_data_callback(self, msg):
        data = msg.data.split(';')
        if len(data) == 4:
            try:
                d_pos = float(data[0])
                t_pos = float(data[2])
            except ValueError:
                self.get_logger().error("Invalid data format received: " + msg.data)
                return
            
            left_wheel_pos = d_pos - t_pos / 2
            right_wheel_pos = d_pos + t_pos / 2
            
            delta_left = left_wheel_pos - self.prev_left_wheel_pos
            delta_right = right_wheel_pos - self.prev_right_wheel_pos
            
            self.prev_left_wheel_pos = left_wheel_pos
            self.prev_right_wheel_pos = right_wheel_pos
            
            delta_distance = (delta_left + delta_right) / 2.0
            delta_theta = (delta_right - delta_left) / self.wheel_base
            
            if delta_theta != 0:
                radius = delta_distance / delta_theta
                delta_x = radius * math.sin(delta_theta)
                delta_y = radius * (1 - math.cos(delta_theta))
            else:
                delta_x = delta_distance
                delta_y = 0.0
            
            self.x += delta_x * math.cos(self.theta) - delta_y * math.sin(self.theta)
            self.y += delta_x * math.sin(self.theta) + delta_y * math.cos(self.theta)
            self.theta += delta_theta
            
            self.publish_odometry()
            self.publish_wheel_positions(left_wheel_pos, right_wheel_pos)

    def publish_odometry(self):
        current_time = self.get_clock().now()
        
        # Create the odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Set the position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.create_quaternion_from_yaw(self.theta)
        
        # Set the velocity (optional, using the same x, y, and theta values for simplicity)
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0
        
        self.odom_pub.publish(odom)
        
        # Broadcast the transform
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = odom.pose.pose.orientation
        
        self.odom_broadcaster.sendTransform(transform)
    
    def publish_wheel_positions(self, left_wheel_pos, right_wheel_pos):
        wheel_positions = Float32MultiArray()
        wheel_positions.data = [left_wheel_pos, right_wheel_pos]
        self.wheel_pos_pub.publish(wheel_positions)
        self.get_logger().info(f"Published wheel positions: left={left_wheel_pos}, right={right_wheel_pos}")

    def reset_odometry_callback(self, request, response):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left_wheel_pos = 0.0
        self.prev_right_wheel_pos = 0.0
        self.get_logger().info("Odometry reset to zero")
        return response

    def create_quaternion_from_yaw(self, yaw):
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        return TransformStamped().transform.rotation.__class__(x=q[0], y=q[1], z=q[2], w=q[3])

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
