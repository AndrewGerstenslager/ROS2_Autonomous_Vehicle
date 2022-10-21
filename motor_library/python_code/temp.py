import serial

def get_arduino_port() -> serial.Serial:
    '''
        This method gets and returns the arduino serial port.
        We do this by checking all oprn ports and selecting
        the port that contains 'ACM'

        returns serial.Serial object
    '''
    import serial.tools.list_ports

    port_name = ""
    for port in serial.tools.list_ports.comports():
        if 'ACM' in port.device:
            port_name = port.device
        
    if port_name != "":
        print(f"Arduino found on port: \"{port_name}\"")
        return serial.Serial(port_name, baudrate= 9600, timeout= 5)
    else:
        return None

arduino_port = get_arduino_port()
print(arduino_port)