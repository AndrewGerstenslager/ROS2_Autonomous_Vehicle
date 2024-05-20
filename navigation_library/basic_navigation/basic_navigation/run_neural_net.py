import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import tensorflow as tf
import math
import numpy as np

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.heading_sub = self.create_subscription(Float64, '/heading_deg', self.heading_callback, 10)
        self.bearing_sub = self.create_subscription(Float64, '/bearing_deg', self.bearing_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nn = tf.keras.models.Sequential([
            tf.keras.layers.Dense(10, activation='relu', input_shape=(4,)),
            tf.keras.layers.Dense(2)
        ])
        self.nn.compile(optimizer='adam', loss='mean_squared_error')
        self.nn.load_weights('/home/andrew/repo/dokalman/navigation_library/basic_navigation/tf_models/weights.h5')
        self.bearing_sin = 0
        self.bearing_cos = 0
        self.heading_sin = 0
        self.heading_cos = 0

    def heading_callback(self, msg):
        # Update heading based on msg
        self.heading_sin = math.sin(msg.data)
        self.heading_cos = math.cos(msg.data)
        self.publish_cmd_vel()

    def bearing_callback(self, msg):
        # Update bearing based on msg
        self.bearing_sin = math.sin(msg.data)
        self.bearing_cos = math.cos(msg.data)
        self.publish_cmd_vel()

    def publish_cmd_vel(self):
        # Predict the command velocity using the neural network
        cmd_vel = self.nn.predict(tf.constant([[self.bearing_sin, self.bearing_cos, self.heading_sin, self.heading_cos]]))

        # Create a Twist message and publish it
        twist = Twist()
        twist.angular.z = float(cmd_vel[0][0])
        twist.linear.x = float(cmd_vel[0][1])
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)
    waypoint_follower.on_shutdown()
    waypoint_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()