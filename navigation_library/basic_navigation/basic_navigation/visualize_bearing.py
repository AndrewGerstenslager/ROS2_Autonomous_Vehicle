import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
import numpy as np

class BearingSubscriber(Node):
    def __init__(self):
        super().__init__('bearing_subscriber')
        self.bearing_sub = self.create_subscription(Float64, '/bearing_deg', self.bearing_callback, 10)
        self.heading_sub = self.create_subscription(Float64, '/heading_deg', self.heading_callback, 10)
        self.fig, self.axs = plt.subplots(2, subplot_kw={'projection': 'polar'})

        # Adjust the spacing between the subplots
        plt.subplots_adjust(hspace=0.5)

        self.arrow_bearing = self.axs[0].arrow(0, 0, 0, 1, alpha = 0.5, width = 0.015,
                                   edgecolor = 'black', facecolor = 'green', lw = 2, zorder = 5)
        self.arrow_heading = self.axs[1].arrow(0, 0, 0, 1, alpha = 0.5, width = 0.015,
                                   edgecolor = 'black', facecolor = 'blue', lw = 2, zorder = 5)

    def bearing_callback(self, msg):
        # Convert degrees to radians and adjust so that arrow is facing up when bearing is directly ahead
        bearing_rad = np.deg2rad(msg.data) + np.pi/2

        # Remove the old arrow
        self.arrow_bearing.remove()

        # Plot the new arrow
        self.arrow_bearing = self.axs[0].arrow(bearing_rad, 0, 0, 1, alpha = 0.5, width = 0.015,
                                   edgecolor = 'black', facecolor = 'green', lw = 2, zorder = 5)
        plt.pause(0.001)

    def heading_callback(self, msg):
        # Convert degrees to radians and adjust so that arrow is facing up when heading is directly ahead
        heading_rad = np.deg2rad(msg.data) + np.pi/2

        # Remove the old arrow
        self.arrow_heading.remove()

        # Plot the new arrow
        self.arrow_heading = self.axs[1].arrow(heading_rad, 0, 0, 1, alpha = 0.5, width = 0.015,
                                   edgecolor = 'black', facecolor = 'blue', lw = 2, zorder = 5)
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    bearing_subscriber = BearingSubscriber()
    rclpy.spin(bearing_subscriber)

if __name__ == '__main__':
    main()