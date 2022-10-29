from asyncio.log import logger
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from libraries.serial_utils import *
from serial import Serial


class Motor_Node(Node):
    '''
        This class is a ros2 node that communicates to the arduino    
    '''
    def __init__(self, port):
        super().__init__('Motor_Node')
        
        #self.serial_port = self.create_serial_port()
        self.serial_port = port
        #subscribe to the turn message
        self.subscription = self.create_subscription(
            String,
            'turn_speed',
            self.turn_callback,
            10)

        #subscribe to the drive message
        self.subscription = self.create_subscription(
            String,
            'drive_speed',
            self.drive_callback,
            10)

    def create_serial_port(self) -> Serial:
        '''Creates a serial port for this class'''
        serial_port = get_arduino_port()
        if (serial_port is None):
            self.get_logger().info("No Arduino detected")
        wait_for_arduino(serial_port)
        self.get_logger().info("Arduino detected")

    def turn_callback(self, msg):
        '''
            Creates the turn message for the Arduino to
            send to the Kangaroo Motor Controller. \n
            Message is from the "turn_speed" topic
        '''
        turn_msg = f"t,s{msg.data}"
        self.get_logger().info(f"Sent: '{turn_msg}' to Arduino")
        write_without_response(turn_msg, self.serial_port)

    def drive_callback(self, msg):
        '''
            Creates the drive message for the Arduino to
            send to the Kangaroo Motor Controller.\n
            Message is from the "drive_speed" topic
        '''
        drive_msg = f"d,s{msg.data}"
        self.get_logger().info(f"Sent: '{drive_msg}' to Arduino")
        write_without_response(drive_msg, self.serial_port)

    #TODO: Add a subscriber that can call a reset to the arduino to restablish the serial connection

    #We want to be able to handle
        #Status Examples: Online, Offline
        #Drive State Example: Self Drive, RC Control

def main(args=None):
    rclpy.init(args=args)

    arduino_port = get_arduino_port()
    wait_for_arduino(arduino_port)
    motor_node = Motor_Node(arduino_port)

    rclpy.spin(motor_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()