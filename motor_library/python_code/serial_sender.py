from time import sleep
import serial
import time

ser = serial.Serial('/dev/ttyACM1', baudrate= 9600, timeout= 5)

def connect():
    s = "Not Ready"
    while(s != "Ready"):
        time.sleep(1)
        print("waiting")
        s = ser.readline().decode('utf-8')
        print(s)
    print("Port Available")

def write_read(data : str, port : serial.Serial = ser): 
    port.write(bytes(data, 'utf-8'))
    time.sleep(0.05)
    response = ""
    while(response == ""):
        response = port.readline().decode("utf-8")
        print("waiting for response")
    return response

connect()

response = write_read("t,getp")

print(f'Response: {response}')
print("finished")

