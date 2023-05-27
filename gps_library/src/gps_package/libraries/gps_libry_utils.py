from typing import Union
import serial
import serial.tools.list_ports
import re

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

def is_valid_decimal(value):
    return re.match(r'^-?\d+(\.\d+)?$', value)

def parse_gps_message(msg):
    #data = read_port_formatted(port)
    msg_split = msg.split(" ")

    if len(msg_split) < 9 or not (is_valid_decimal(msg_split[7]) and is_valid_decimal(msg_split[8])):
        return None

    msg_dict = {'longitude': msg_split[8], 'latitude': msg_split[7]}
    return msg_dict

def get_gps_ports():
    # Get a list of all available serial ports
    available_ports = serial.tools.list_ports.comports()
    device1=device2 = None
    # Define the pattern to search for in the port description
    pattern = "Novatel"
    # Loop through the list of available ports and look for matching descriptions
    for port in available_ports:
        if pattern in port.description:
            if device1 is None:
                device1 = serial.Serial(port.device)
            elif device2 is None:
                device2 = serial.Serial(port.device)
            
            if device1 is not None and device2 is not None:
                return device1, device2
    return None
