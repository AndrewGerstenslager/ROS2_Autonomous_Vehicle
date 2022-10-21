from time import sleep
import serial
from serial_utils import    (get_arduino_port, 
                            read_port_formatted, 
                            wait_for_arduino, 
                            write_read)


ser = get_arduino_port()

wait_for_arduino(ser)

print(write_read("YO",ser))

#response = write_read("t,getp")

#print(f'Response: {response}')
#print("finished")

