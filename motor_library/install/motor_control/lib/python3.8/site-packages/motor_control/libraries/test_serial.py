from time import sleep
from serial_utils import    (get_arduino_port,  
                            wait_for_arduino, 
                            write_read,
                            write_without_response)


ser = get_arduino_port()
breakpoint()
wait_for_arduino(ser)

write_without_response("t,s100",ser)
sleep(1)

write_without_response("t,s200",ser)   
sleep(1)

write_without_response("t,s300",ser)  
sleep(1)

write_without_response("t,s400",ser)

write_without_response("t,s0",ser)
sleep(1)

write_without_response("YO",ser)

