# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

class CameraSubscriber(Node):
    def __init__(self):
        
        # calls the parent class's constructor
        # sets the name of the node
        super().__init__('cam_sub')

        
        # The node subscribes to messages of type Image, 
        # over a topic named: video_frames
        # The callback function is called as soon as a message is received.
        # The maximum number of queued messages is 10.
        self.subscription = self.create_subscription(
            Image, 
            'video_frames',
            self.listener_callback,
            10)
        
        # initiallizing the CvBridge object
        self.bridge = CvBridge()

    def listener_callback(self, data):
        
        # Display the message on the console
        self.get_logger().info('Receiving Video Frames: "%s"' % data)

        # convert ROS image to CV image
        current_frame  = self.bridge.imgmsg_to_cv2(data)

        # displays the received camera image
        cv2.imshow('camera_feed', current_frame)

        cv2.waitKey(1)

def main(args=None):
    # initailize the rcply library
    rclpy.init(args=args)

    # create the node
    cam_sub = CameraSubscriber()

    # spin the node so that the callback function is called
    rclpy.spin(cam_sub)

    # destroy the node explicitly
    cam_sub.destroy_node()

    # shutdown the ROS client lib for python
    rclpy.shutdown()

if __name__ == '__main__':
    main()
