# MOTOR CONTROL LIBRARY
  
### This is the readme explaining the motor control library.


### We have two sections, the Python and the Arduino section:

## PYTHON:
The python section has a ROS2 node that communicates to and from the ROS2 network.

This code is located at /src/motor_contol/motor_control/motor_subscriber_node.py

The python section also uses serial communication to talk to and from the Arduino
over its Serial port (USB).

This library is located at /src/motor_contol/motor_control/libraries/serial_utils.py

***NOTE: We did this because ROS2 developers at this time are terrible and the ROS2 Arduino libraries are all broken, suck, and don't support basic Arduino Mega/Uno boards***

## ARDUINO:
The Arduino section is listening on the USB port for commands from python.
When a serial connection is established or reconnected, the arduino restarts.
When we connect with python and listen to the serial port, we should always be able to 
read in "Ready" from the serial port. That way we know the setup method has ran all the
way before we start sending commands before the Arduino is fully initialized.

The python library should be os-agnostic in terms of finding the serial port and connecting
to the Arduino by the way we check the port.description to contain "Arduino". 

***NOTE: This library will only connect to one arduino at a time at the moment***

***NOTE: Serial.setTimeout(10); is INCREDIBLY IMPORTANT. If not in setup(), the arduino will take 1 second each time it calls Serial.readString(); which is a huge delay. It usually takes 2ms otherwise to send and receive data***

## To use this library:

1. Install arduino_main.ino or any similar file to the Arduino using the Arduino IDE

    ***NOTE: If Serial1 is not recognized, make sure you have "Arduino Mega" selected as the target board type***

2. Build and source the package using ". build_and_source.sh" command in "/dokalman/motor_library" directory


4. Run "ros2 pkg list | grep motor" to verify that motor_control comes up

5. Finally Run "ros2 run motor_control arduino_node" to start the package

 

