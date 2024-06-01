import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf_transformations
import numpy as np

class IMUOdometryNode(Node):

    def __init__(self):
        super().__init__('imu_odometry_node')
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.imu_subscriber = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.timer = self.create_timer(0.1, self.publish_odometry)
        
        self.last_time = self.get_clock().now()
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # Quaternion (x, y, z, w)

        self.accel = np.array([0.0, 0.0, 0.0])
        self.gyro = np.array([0.0, 0.0, 0.0])

    def imu_callback(self, msg: Imu):
        self.accel[0] = msg.linear_acceleration.x
        self.accel[1] = msg.linear_acceleration.y
        self.accel[2] = msg.linear_acceleration.z

        self.gyro[0] = msg.angular_velocity.x
        self.gyro[1] = msg.angular_velocity.y
        self.gyro[2] = msg.angular_velocity.z

    def publish_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds

        # Update velocity and position using acceleration
        self.velocity += self.accel * dt
        self.position += self.velocity * dt

        # Update orientation using gyroscope data
        delta_angle = self.gyro * dt
        delta_quat = tf_transformations.quaternion_from_euler(delta_angle[0], delta_angle[1], delta_angle[2])
        self.orientation = tf_transformations.quaternion_multiply(self.orientation, delta_quat)

        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set position
        odom_msg.pose.pose.position.x = self.position[0]
        odom_msg.pose.pose.position.y = self.position[1]
        odom_msg.pose.pose.position.z = self.position[2]

        # Set orientation
        odom_msg.pose.pose.orientation.x = self.orientation[0]
        odom_msg.pose.pose.orientation.y = self.orientation[1]
        odom_msg.pose.pose.orientation.z = self.orientation[2]
        odom_msg.pose.pose.orientation.w = self.orientation[3]

        # Set velocity
        odom_msg.twist.twist.linear.x = self.velocity[0]
        odom_msg.twist.twist.linear.y = self.velocity[1]
        odom_msg.twist.twist.linear.z = self.velocity[2]

        # Publish the odometry message
        self.odom_publisher.publish(odom_msg)
        
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    imu_odometry_node = IMUOdometryNode()
    rclpy.spin(imu_odometry_node)
    imu_odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
