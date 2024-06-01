import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import time

class SerialCommNode(Node):
    def __init__(self):
        super().__init__('serial_comm_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = 9600
        
        # Initialize serial connection
        self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        time.sleep(2)  # Wait for serial connection to initialize
        
        # Initialize self-drive state
        self.self_drive_enabled = False
        
        # Create services
        self.create_service(Empty, 'start_self_drive', self.start_self_drive_callback)
        self.create_service(Empty, 'stop_self_drive', self.stop_self_drive_callback)
        self.create_service(Empty, 'emergency_stop', self.emergency_stop_callback)
        
        # Create subscriber for cmd_vel
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Create publishers for status and raw data
        self.status_pub = self.create_publisher(String, 'arduino_status', 10)
        self.raw_data_pub = self.create_publisher(String, 'raw_data', 10)
        
        # Create a timer to periodically read data from the Arduino
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        self.get_logger().info("Serial Communication Node has been started")
    
    def start_self_drive_callback(self, request, response):
        self.send_serial_command('b')
        self.self_drive_enabled = True
        self.get_logger().info("Sent 'b' to start self drive")
        return response
    
    def stop_self_drive_callback(self, request, response):
        self.send_serial_command('e')
        self.self_drive_enabled = False
        self.get_logger().info("Sent 'e' to stop self drive")
        return response
    
    def emergency_stop_callback(self, request, response):
        self.send_serial_command('s')
        self.self_drive_enabled = False
        self.get_logger().info("Sent 's' to emergency stop")
        return response
    
    def cmd_vel_callback(self, msg):
        if self.self_drive_enabled:
            drive_cmd = f"d,s{int(msg.linear.x * 100)}"
            turn_cmd = f"t,s{int(msg.angular.z * 100)}"
            self.send_serial_command(drive_cmd)
            self.send_serial_command(turn_cmd)
            self.get_logger().info(f"Sent drive command: {drive_cmd}")
            self.get_logger().info(f"Sent turn command: {turn_cmd}")
    
    def timer_callback(self):
        if self.serial_conn.in_waiting == 0:
            self.send_serial_command("p")
        
        if self.serial_conn.in_waiting > 0:
            raw_data = self.serial_conn.read_until().decode().strip()
            if raw_data.startswith("STATUS:"):
                status_message = raw_data[len("STATUS:"):].strip()
                self.status_pub.publish(String(data=status_message))
                self.get_logger().info(f"Published status: {status_message}")
            elif raw_data.startswith("DEBUG:"):
                debug_message = raw_data[len("DEBUG:"):].strip()
                self.get_logger().debug(f"Debug: {debug_message}")
            else:
                parts = raw_data.split(';')
                if len(parts) == 4 and all(part.strip() for part in parts):
                    self.raw_data_pub.publish(String(data=raw_data))
                    self.get_logger().info(f"Published raw data: {raw_data}")
    
    def send_serial_command(self, command):
        if self.serial_conn.is_open:
            self.serial_conn.write((command + '\n').encode())
            start_time = time.time()
            while self.serial_conn.in_waiting == 0:
                if time.time() - start_time > 1:  # Timeout after 1 second
                    self.get_logger().warning(f"Timeout waiting for response to command: {command}")
                    break
            self.get_logger().info(f"Sent command: {command}")
    
    def close_serial(self):
        if self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info("Serial connection closed")

def main(args=None):
    rclpy.init(args=args)
    node = SerialCommNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.close_serial()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
