import os
import serial

Vendor=0x09d7 
ProdID=0x0100 
Rev=01.01



def read_port_formatted(port : serial.Serial) -> str:
    '''
        Reads the serial port for its current value\n
        Additionally removes last escape characters from string
    '''
    if port is None: raise Exception("Port undefined")
    
    resp = port.readline().decode('utf-8')
    return resp[:len(resp) - 2]


def write_without_response(data : str, port : serial.Serial):
    '''
        Writes a string to the serial port without waiting for response
    '''
    if port is None: raise Exception("Port undefined")
    port.write(data.encode('utf-8'))

def main():
    print('Hi from gps_package.')

    with serial.Serial(port='/dev/ttyUSB3', baudrate=115200, timeout=0.25) as s:
        try:
            write_without_response("log bestpos ontime 1", port=s)
            while True:
                line = s.readline().decode('utf-8')
                if 'BESTPOS' in line:
                    print(line)
        #except:
            #print("exit")
            #pass

        finally:
            s.close()
        pass


if __name__ == '__main__':
    main()

'''CHATGPT RESPONSE TO SET UP TOPICS

Yes, the gps_navigation package in ROS2 expects GPS data to be published on specific topics in a standardized ROS2 message format. Specifically, it expects GPS data to be published on the /fix topic using the sensor_msgs/NavSatFix message type.

The NavSatFix message type contains the latitude, longitude, and altitude of the GPS fix, as well as information about the quality of the fix (e.g., number of satellites used, horizontal/vertical dilution of precision, etc.). Here is an example of how to create a custom message for GPS data in ROS2 using the sensor_msgs/NavSatFix message:

bash

# Example custom message for GPS data
# Filename: MyNavSatFix.msg

Header header        # Header information
sensor_msgs/NavSatStatus status  # Status of the GPS fix
float64 latitude     # Latitude in degrees
float64 longitude    # Longitude in degrees
float64 altitude     # Altitude in meters above sea level
float64 position_covariance[9] # Position covariance matrix (optional)

Once you have defined your custom message, you can use it to publish GPS data on the /fix topic, which gps_navigation will use for navigation.

To summarize, in order to set up GPS node properly for gps_navigation package, you need to publish GPS data on the /fix topic using the sensor_msgs/NavSatFix message type. You can either use the existing sensor_msgs/NavSatFix message type or define your own custom message that conforms to this message type.

--------------------------------------------------------------------------------------------------------------------------------------------------------------


Yes, the gps_navigation package in ROS2 can take in heading information in addition to GPS data. The package uses GPS data (published on the /fix topic using the sensor_msgs/NavSatFix message type) for localization and navigation, and heading information (published on the /imu topic using the sensor_msgs/Imu message type) for orientation estimation.

The Imu message type contains information about the orientation and angular velocity of the device, including the heading (i.e., yaw) angle. Here is an example of how to create a custom message for IMU data in ROS2 using the sensor_msgs/Imu message:

bash

# Example custom message for IMU data
# Filename: MyImu.msg

Header header        # Header information
geometry_msgs/Quaternion orientation # Orientation as a quaternion
float64[9] orientation_covariance # Orientation covariance matrix
geometry_msgs/Vector3 angular_velocity # Angular velocity in radians/s
float64[9] angular_velocity_covariance # Angular velocity covariance matrix
geometry_msgs/Vector3 linear_acceleration # Linear acceleration in m/s^2
float64[9] linear_acceleration_covariance # Linear acceleration covariance matrix

Once you have defined your custom message, you can use it to publish heading information on the /imu topic, which gps_navigation will use for orientation estimation.

To summarize, in order to use heading information with gps_navigation package, you need to publish the heading information on the /imu topic using the sensor_msgs/Imu message type, in addition to publishing GPS data on the /fix topic using the sensor_msgs/NavSatFix message type.


'''