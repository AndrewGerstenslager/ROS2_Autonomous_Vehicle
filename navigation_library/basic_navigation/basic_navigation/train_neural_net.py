import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import tensorflow as tf
import math

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.heading_sub = self.create_subscription(Float64, '/heading_deg', self.heading_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nn = tf.keras.models.Sequential([
            tf.keras.layers.Dense(10, activation='relu', input_shape=(2,)), 
            tf.keras.layers.Dense(2)
        ])
        self.nn.compile(optimizer='sgd', loss='mean_squared_error')
        self.nn.load_weights('/home/andrew/repo/dokalman/navigation_library/basic_navigation/tf_models/weights.h5')
        self.bearing_sin = 0
        self.bearing_cos = 0
        self.angular_z = 0
        self.linear_x = 0

    def gps_callback(self, msg):
        # Calculate bearing and convert to sine and cosine
        # This is a placeholder, replace with actual calculation
        self.bearing_sin = math.sin(msg.latitude)
        self.bearing_cos = math.cos(msg.longitude)

    def heading_callback(self, msg):
        # Update bearing based on heading
        # This is a placeholder, replace with actual calculation
        self.bearing_sin = math.sin(msg.data)
        self.bearing_cos = math.cos(msg.data)

    def cmd_vel_callback(self, msg):
        self.angular_z = msg.angular.z
        self.linear_x = msg.linear.x

        # Train the neural network
        history = self.nn.train_on_batch(
            x=tf.constant([[self.bearing_sin, self.bearing_cos]]),
            y=tf.constant([[self.angular_z, self.linear_x]])
        )

        print(f"Training loss: {history}")

    def on_shutdown(self):
        # Save the weights of the neural network
        self.nn.save_weights('/home/andrew/repo/dokalman/navigation_library/basic_navigation/tf_models/weights.h5')

import signal

def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()

    def sigint_handler(sig, frame):
        # Save weights and shutdown on Ctrl+C
        waypoint_follower.on_shutdown()
        waypoint_follower.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, sigint_handler)

    rclpy.spin(waypoint_follower)
if __name__ == '__main__':
    main()