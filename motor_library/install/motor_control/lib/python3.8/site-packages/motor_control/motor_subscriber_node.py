import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Motor_Node(Node):
    '''
        This class is a ros2 node that communicates to the arduino    
    '''
    def __init__(self):
        super().__init__('Motor_Node')
        
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

        self.subscription  # prevent unused variable warning

    def turn_callback(self, msg):
        '''
            Creates the turn message for the Arduino to
            send to the Kangaroo Motor Controller. \n
            Message is from the "turn_speed" topic
        '''
        turn_msg = f"t,s{msg.data}"
        self.get_logger().info(f"Sent: '{turn_msg}' to Arduino")
        #TODO: Add code to utilize serial library

    def drive_callback(self, msg):
        '''
            Creates the drive message for the Arduino to
            send to the Kangaroo Motor Controller.\n
            Message is from the "drive_speed" topic
        '''
        drive_msg = f"d,s{msg.data}"
        self.get_logger().info(f"Sent: '{drive_msg}' to Arduino")
        #TODO: Add code to utilize serial library

    #TODO: Add a subscriber that can call a reset to the arduino to restablish the serial connection

def main(args=None):
    rclpy.init(args=args)

    motor_node = Motor_Node()

    rclpy.spin(motor_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()