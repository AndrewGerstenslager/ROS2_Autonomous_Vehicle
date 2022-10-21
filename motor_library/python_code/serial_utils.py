import serial
import serial.tools.list_ports


def get_arduino_port() -> serial.Serial:
    '''
        This method gets and returns the arduino serial port.
        We do this by checking all open ports and selecting
        the port that contains 'ACM'

        returns serial.Serial object
    '''
    port_name = ""
    for port in serial.tools.list_ports.comports():
        if 'Arduino' in port.description:
            port_name = port.device
    if port_name != "":
        print(f"Arduino found on port: \"{port_name}\"")
        return serial.Serial(port_name, baudrate= 9600, timeout= 5)
    else:
        print(f"No Arduino found")
        return None

def read_port_formatted(port : serial.Serial) -> str:
    '''
        Reads the serial port for its current value\n
        Additionally removes last escape characters from string
    '''
    global default_serial_port
    if port is None: raise Exception("Port undefined")
        
    resp = port.readline().decode('utf-8')
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
        port.write("Ready".encode('utf-8'))
        resp = read_port_formatted(port)
        if(resp == "Ready"):
            print("Arduino is ready")
            return

def write_read(data : str, port : serial.Serial) -> str: 
    '''
        Writes to arduino and waits for a response.\n
        Returns a string with the appropriate response
    '''
    global default_serial_port
    if port is None: raise Exception("Port undefined")
        
    port.write(data.encode('utf-8'))
    response = ""
    while(response != ""):
        response = read_port_formatted(port)
    return response


#this method will run only if this library is ran in python directly
if __name__=="__main__":
    get_arduino_port()