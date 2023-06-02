import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import serial.tools.list_ports


class SparkfunImuNode(Node):
    def __init__(self):
        super().__init__('sparkfun_imu_node')
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.timer_period = 0.01  # 100Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.ser = None
        self.find_9dof_port()

    def find_9dof_port(self):
        # Get a list of all available ports
        ports = serial.tools.list_ports.comports()

        # Iterate through each port and check if the description matches the 9DOF sensor
        for port in ports:
            if "9DOF" in port.description:
                self.get_logger().info("Found 9DOF port: {}".format(port.device))
                self.ser = serial.Serial(port.device, 9600)
                return

        self.get_logger().warn("No 9DOF port found.")

    def timer_callback(self):
        if self.ser is not None:
            line = self.ser.readline().decode().strip()
            self.get_logger().info("Received data: {}".format(line))

            values = line.split(',')

            if len(values) == 10: # We expect 10 values
                imu_msg = Imu()
                
                # Linear acceleration [m/s^2]
                imu_msg.linear_acceleration.x = float(values[1])
                imu_msg.linear_acceleration.y = float(values[2])
                imu_msg.linear_acceleration.z = float(values[3])
                
                # Angular velocity [rad/s]
                imu_msg.angular_velocity.x = float(values[4])
                imu_msg.angular_velocity.y = float(values[5])
                imu_msg.angular_velocity.z = float(values[6])
                
                self.publisher_.publish(imu_msg)



def main(args=None):
    rclpy.init(args=args)
    sparkfun_imu_node = SparkfunImuNode()
    rclpy.spin(sparkfun_imu_node)
    sparkfun_imu_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
