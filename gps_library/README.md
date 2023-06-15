### [go to main page](../README.md)

# __gps_library__

### This library communicates to the Novatel GPS onboard Dokalman via serial communication. This is done by sending one message to log position and another message to log heading. This is done over two separate serial connections because the Novatel connects with three serial connections to a computer via the USB port. We are not sure why it does but we take advantage of it by using separate connections for different messages.

## __Requirements for this Library__

- ros2 - Foxy
- Python
    - pyserial

## __PYTHON:__

#TODO: UPDATE THIS