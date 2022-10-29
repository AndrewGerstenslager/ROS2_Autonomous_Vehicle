import serial
import serial.tools.list_ports
import time

def get_arduino_port() -> serial.Serial:
    '''
        This method gets and returns the arduino serial port.
        We do this by checking all open ports and selecting
        the port that contains 'ACM'

        returns serial.Serial object
    '''
    port_name = ""
    for port in serial.tools.list_ports.comports():
        print(port.description)
        if 'Arduino' in port.description or 'ACM' in port.description:
            port_name = port.device
    if port_name != "":
        print(f"Arduino found on port: \"{port_name}\"")
        return serial.Serial(port_name, baudrate= 9600, timeout= 1)
    else:
        print(f"No Arduino found")
        return None

def clear_buffer(port : serial.Serial) -> bool:
    resp = "NOT_CLEAR"
    while(resp != ""):
        resp = port.readline().decode('utf-8')
        print(resp)
    return True

def read_port_formatted(port : serial.Serial) -> str:
    '''
        Reads the serial port for its current value\n
        Additionally removes last escape characters from string
    '''
    if port is None: raise Exception("Port undefined")
    
    resp = port.readline().decode('utf-8')
    #clear_buffer(port)
    return resp[:len(resp) - 2]

def wait_for_arduino(port : serial.Serial):
    '''
        Waits for arduino to finish its setup method.\n
        We should have Serial.println("Ready"); at the end of setup()
        in the arduino's code
    '''
    if port is None: raise Exception("Port undefined")
    resp = ""
    while(resp != "Ready"):
        resp = read_port_formatted(port)
        if(resp == "Ready"):
            print("Arduino is ready")
            clear_buffer(port)
            return

def write_read(data : str, port : serial.Serial, timeout : int = 1) -> str: 
    '''
        Writes to arduino and waits for a response.\n
        Returns a string with the appropriate response.\n
        Optional parameter to set the timeout for a response from the Arduino
    '''
    if port is None: raise Exception("Port undefined")
    start_time = time.time()
    port.write(data.encode('utf-8'))
    resp = ""
    while(resp == ""):
        if (time.time() - start_time) > timeout:
            return "REQUEST TIMED OUT"
        resp = read_port_formatted(port)
    return resp

def write_without_response(data : str, port : serial.Serial):
    '''
        Writes a string to the serial port without waiting for response
    '''
    if port is None: raise Exception("Port undefined")
    port.write(data.encode('utf-8'))

#this method will run only if this library is ran in python directly
if __name__=="__main__":
    print("RUNNING SERIAL UTILS")
    port = get_arduino_port()
    wait_for_arduino(port)