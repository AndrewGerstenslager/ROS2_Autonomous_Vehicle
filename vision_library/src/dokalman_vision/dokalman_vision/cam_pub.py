# based on this tutorial: https://automaticaddison.com/create-a-basic-publisher-and-subscriber-python-ros2-foxy/
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library


# Create a CameraPublisher class, which is a subclass of the Node class.
class CameraPublisher(Node):

    def __init__(self):
        # calls the parent class's constructor
        # sets the name of the node
        super().__init__('cam_pub')

        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)

        # We will publish a message every 0.1 seconds
        timer_period = 0.1  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        self.cap = cv2.VideoCapture(0)
        # Sets resolution of the frame to 480x270
        self.cap.set(3,480)
        self.cap.set(4,270)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def timer_callback(self):

        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, frame = self.cap.read()

        if ret == True:
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS 2 image message
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))

            # Display the message on the console
            self.get_logger().info('Publishing video frame')

def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    cam_pub = CameraPublisher()

    # Spin the node so the callback function is called.
    rclpy.spin(cam_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cam_pub.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()