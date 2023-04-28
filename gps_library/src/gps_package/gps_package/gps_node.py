import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from libraries.gps_libry_utils import * 

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')
        self.latitude_longitude_publisher = self.create_publisher(NavSatFix, 'gps', 10)
        self.heading_publisher = self.create_publisher(Float64, 'heading', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global lat_lon_global, heading_global

        lat_lon_msg = NavSatFix()
        heading_msg = Float64()

        #READ FROM DEVICE 1 FOR HEADING
        heading2a = read_port_formatted(device1)
        if heading2a and "HEADING2A" in heading2a:
            heading_global = heading2a.split(",")[12]
            
        #READ FROM DEVICE 2 FOR POSITION
        position = read_port_formatted(device2)
        if position:
            lat_lon_dict = parse_gps_message(position)
            if lat_lon_dict:
                lat_lon_global = lat_lon_dict

        #GET AND PUBLISH LON_LAT
        if lat_lon_global:
            lat_lon_msg.latitude = lat_lon_global['latitude']
            lat_lon_msg.longitude = lat_lon_global['longitude']
            self.latitude_longitude_publisher.publish(lat_lon_msg)
        
        #GET AND PUBLISH HEADING
        if heading_global:
            heading_msg.data = heading_global
            self.heading_publisher.publish(heading_msg)


def main(args=None):
    #DEFINE GLOBAL DATA TO STORE LAST RECORDED POSITION
    global device1,device2

    rclpy.init(args=args)
    
    ports = get_gps_ports()

    # Print out the device names (if found)
    if not ports:
        print("NO GPS DETECTED")
        rclpy.shutdown()
        return

    device1,device2 = ports
    
    print("Connected to Novatel device 1 on", device1.name)
    print("Connected to Novatel device 2 on", device2.name)

    write_without_response("log heading2a onchanged\r\n", port=device1)
    write_without_response("log bestpos ontime 0.25\r\n", port=device2)

    
    

    publisher = MyPublisher()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
