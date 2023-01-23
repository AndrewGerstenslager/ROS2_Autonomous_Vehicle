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
