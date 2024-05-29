import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import pandas as pd
import signal
import os

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        self.heading_sub = self.create_subscription(Float64, '/heading_deg', self.heading_callback, 10)
        self.bearing_sub = self.create_subscription(Float64, '/bearing_deg', self.bearing_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.data = pd.DataFrame(columns=['heading', 'bearing', 'angular_z', 'linear_x'])
        self.latest_heading = None
        self.latest_bearing = None
        self.latest_cmd_vel = None
        self.timer = self.create_timer(0.1, self.timer_callback)

    def heading_callback(self, msg):
        self.latest_heading = msg.data

    def bearing_callback(self, msg):
        self.latest_bearing = msg.data

    def cmd_vel_callback(self, msg):
        self.latest_cmd_vel = {
            'angular_z': msg.angular.z,
            'linear_x': msg.linear.x
        }

    def timer_callback(self):
        if self.latest_heading is not None and self.latest_bearing is not None and self.latest_cmd_vel is not None:
            self.data.loc[len(self.data)] = {'heading': self.latest_heading, 'bearing': self.latest_bearing, **self.latest_cmd_vel}
            print(f"Current dataset size: {len(self.data)}")
        else:
            print('something wrong is occurring')
            print(self.latest_heading, self.latest_bearing)
            if self.latest_cmd_vel is not None:
                print(self.latest_cmd_vel['angular_z'], self.latest_cmd_vel['linear_x'])
            else:
                print('cmd_vel is None')
                
    def on_shutdown(self):
        # Save the DataFrame to a CSV file
        self.data.to_csv(os.path.join('/home/andrew/repo/dokalman/navigation_library/basic_navigation/tf_models', 'data.csv'), index=False)
        print(f"Final dataset size: {len(self.data)}")
        
def main(args=None):
    print('starting')
    rclpy.init(args=args)
    data_collector = DataCollector()

    def sigint_handler(sig, frame):
        # Save data and shutdown on Ctrl+C
        data_collector.on_shutdown()
        data_collector.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, sigint_handler)

    rclpy.spin(data_collector)

if __name__ == '__main__':
    main()