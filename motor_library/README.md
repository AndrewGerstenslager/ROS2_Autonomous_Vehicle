### [go to main page](../README.md)

# __MOTOR CONTROL LIBRARY__
  
## __Requirements for this Library__

- ros2 - Foxy
- Python
    - pyserial
- Arduino Ide

### We have two sections, the Python and the Arduino section:

## __PYTHON:__
The python section has a ROS2 node that communicates to and from the ROS2 network.

This code is located at /src/motor_contol/motor_control/motor_subscriber_node.py

The python section also uses serial communication to talk to and from the Arduino
over its Serial port (USB).

This library is located at /src/motor_contol/motor_control/libraries/serial_utils.py

***NOTE: We did this because ROS2 developers at this time are terrible and the ROS2 Arduino libraries are all broken, suck, and don't support basic Arduino Mega/Uno boards***

## __ARDUINO:__
The Arduino section is listening on the USB port for commands from python.
When a serial connection is established or reconnected, the arduino restarts.
When we connect with python and listen to the serial port, we should always be able to 
read in "Ready" from the serial port. That way we know the setup method has ran all the
way before we start sending commands before the Arduino is fully initialized.

The python library should be os-agnostic in terms of finding the serial port and connecting
to the Arduino by the way we check the port.description to contain "Arduino". 

***NOTE: This library will only connect to one arduino at a time at the moment***

***NOTE: Serial.setTimeout(10); is INCREDIBLY IMPORTANT. If not in setup(), the arduino will take 1 second each time it calls Serial.readString(); which is a huge delay. It usually takes 2ms otherwise to send and receive data***

## __ROS2 Executables__

-   arduino_node
    - _Description: Takes in images from left and right images and publishes raw images to ROS2_
    - sub: turn_speed, drive_speed


