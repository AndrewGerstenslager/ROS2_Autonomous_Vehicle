# MOTOR CONTROL LIBRARY
  
### This is the readme explaining the motor control library.


### We have two sections, the python and the arduino section:

## PYTHON:
The python section has a ROS2 node that communicates to and from the ROS2 network.
The python section also uses serial communication to talk to and from the Arduino
over its Serial port (USB).

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


## To use this library:

Install arduino_main.ino to the Arduino using the Arduino IDE
***NOTE: If Serial1 is not recognized, make sure you have Arduino mega selected as the target board type***

Run the python code from a computer connected via USB to the Arduino

