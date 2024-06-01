import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial

class IMUSerialNode(Node):

    def __init__(self):
        super().__init__('imu_serial_node')
        self.publisher_ = self.create_publisher(Imu, '/imu', 20)
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Update with your serial port and baud rate
        self.timer = self.create_timer(0.1, self.publish_imu_data)  # Publish at 10 Hz

    def publish_imu_data(self):
        try:
            # Clear the serial buffer to get the latest data
            self.serial_port.reset_input_buffer()

            line = self.serial_port.readline().decode('utf-8').strip()
            if line:
                data = line.split(',')
                if len(data) >= 6:
                    imu_msg = Imu()
                    
                    # Parse the accelerometer data
                    imu_msg.linear_acceleration.x = float(data[0])
                    imu_msg.linear_acceleration.y = float(data[1])
                    imu_msg.linear_acceleration.z = float(data[2])
                    
                    # Parse the gyroscope data
                    imu_msg.angular_velocity.x = float(data[3])
                    imu_msg.angular_velocity.y = float(data[4])
                    imu_msg.angular_velocity.z = float(data[5])
                    
                    # The quaternion and covariance are not set in this example
                    # You may need to adjust this part according to your data format
                    
                    self.publisher_.publish(imu_msg)
                    self.get_logger().info(f'Publishing IMU data: {line}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial exception: {e}')
        except ValueError as e:
            self.get_logger().error(f'Value error: {e}')

def main(args=None):
    rclpy.init(args=args)
    imu_serial_node = IMUSerialNode()
    rclpy.spin(imu_serial_node)
    imu_serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

