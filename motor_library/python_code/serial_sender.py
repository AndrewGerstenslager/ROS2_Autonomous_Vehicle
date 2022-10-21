from time import sleep
import serial
import time

def get_arduino_port() -> serial.Serial:
    '''
        This method gets and returns the arduino serial port.\n
        We do this by checking all oprn ports and selecting
        the port that contains 'ACM'\n

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

ser = get_arduino_port()

def get_str_from_port(port : serial.Serial = ser) -> str:
    '''Removes last escape characters from string'''
    resp = ser.readline().decode('utf-8')
    return resp[:len(resp) - 2]


def connect():
    s = ""
    while(s != "Ready"):
        #time.sleep(0.25)
        #print("waiting")
        ser.write("Ready".encode('utf-8'))
        s = get_str_from_port()
        if(s == "Ready"):
            print("Arduino is ready")
            return None

def write_read(data : str, port : serial.Serial = ser): 
    port.write(data.encode('utf-8'))
    response = ""
    while(response != ""):
        response = get_str_from_port()
    return response


connect()

print(write_read("YO"))

#response = write_read("t,getp")

#print(f'Response: {response}')
#print("finished")

