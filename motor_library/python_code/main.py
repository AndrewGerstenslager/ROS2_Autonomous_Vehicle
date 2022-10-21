from serial_utils import    (get_arduino_port,  
                            wait_for_arduino, 
                            write_read)


ser = get_arduino_port()

wait_for_arduino(ser)


print(write_read("t,s500",ser))

print(write_read("rc",ser))

print(write_read("YO",ser))

