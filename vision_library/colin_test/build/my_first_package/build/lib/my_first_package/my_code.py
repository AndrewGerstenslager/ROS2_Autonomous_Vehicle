import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
# Import the definition for the String message type
from std_msgs.msg import String

class BasicNode(Node):
    def __init__(self):
        super().__init__("basic_node")

        

        # Each component (publisher, subscriber, server, client, etc) is added as a member
        # of the Node. Rclpy provides methods for creating these components.
        # Create a publisher component as a member of the node
        self.publisher = self.create_publisher(Image, "image_frames", 10)

        # Create a subscriber for our topic (yes, a node can subscribe to itself). 
        self.string_subscriber = self.create_subscription(Image, "~/image_topic", self.sub_callback, 10)

        # The publisher won't do anything on its own, so we also create a timer
        # that will call a function to publish messages. The timer member will 
        # call the given function (self.timer_callback) at a given rate, which
        # in this example is every 1 second. 
        self.timer = self.create_timer(1, self.timer_callback)

    # Function that is called whenever the subscriber receives a message
    def sub_callback(self, message):
        # Print out the message's data field
        print("I received an image")
        # Log the message's data field
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(message.data, "bgr8")
        cv2.imwrite("/home/2boss2ros/my_first_package/my_first_package/TestImage2.jpg", orig)
        self.get_logger().info("Saved the image")

    # Function to publish messages whenever the timer triggers
    def timer_callback(self):
        
        img = cv2.imread("/home/2boss2ros/my_first_package/my_first_package/TestImage.jpg")
        cv2.imwrite("/home/2boss2ros/my_first_package/my_first_package/TestImage2.jpg", img)

        bridge = CvBridge()
        imgMsg = bridge.cv2_to_imgmsg(img, "bgr8")
        message= Image()
        
        message.data = imgMsg
        self.publisher.publish(message)

def main(args=None):
    rclpy.init(args=args)
    image_node = BasicNode()
    rclpy.spin(image_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
