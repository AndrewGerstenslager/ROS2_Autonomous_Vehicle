import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Image
import std_msgs.msg as std_msgs
from cv_bridge import CvBridge
import numpy as np
import cv2

class ImageToPCDPublisher(Node):
    """
    A class that converts an image to a PointCloud2 message and publishes it.
    """

    def __init__(self):
        super().__init__('image_to_pcd_publisher')
        self.declare_parameter('scale', 1/255)  # Declare a parameter for scaling
        
        # Initialize the CvBridge
        self.bridge = CvBridge()
        
        # Create a subscription to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/img_processed',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Create a publisher for the PointCloud2 data
        self.pcd_publisher = self.create_publisher(PointCloud2, 'pcd', 10)

    def image_callback(self, msg):
        """
        Callback function for the image topic subscription.
        Converts the received image to a PointCloud2 message and publishes it.
        
        Parameters:
            msg (sensor_msgs.msg.Image): The ROS Image message containing the image data.
        """
        # Convert the ROS Image message to a CV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        # Assume cv_image is binary (0 or 255). Find coordinates of all white pixels
        y_coords, x_coords = np.where(cv_image == 255)
        
        # Get scale parameter
        scale = self.get_parameter('scale').get_parameter_value().double_value

        # Scale x and y coordinates
        x_coords = x_coords * scale
        y_coords = y_coords * scale
        
        # Create 3D points, setting z = 0
        points = np.vstack((x_coords, y_coords, np.zeros_like(x_coords))).astype(np.float32).T

        # Convert points to PointCloud2
        pcd = self.create_point_cloud(points, 'base_link')
        
        # Publish the PointCloud2
        self.pcd_publisher.publish(pcd)

    def create_point_cloud(self, points, parent_frame) -> PointCloud2:
        """
        Function to create a PointCloud2 message from a numpy array of points.
        
        Parameters:
            points (numpy.ndarray): The array of 3D points.
            parent_frame (str): The frame ID of the parent coordinate frame.
        
        Returns:
            sensor_msgs.msg.PointCloud2: The PointCloud2 message.
        """
        header = std_msgs.Header(frame_id=parent_frame)
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
        return PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(np.float32().itemsize * 3),
            row_step=(np.float32().itemsize * 3 * len(points)),
            data=np.asarray(points, np.float32).tobytes()
        )

def main(args=None):
    rclpy.init(args=args)
    node = ImageToPCDPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
