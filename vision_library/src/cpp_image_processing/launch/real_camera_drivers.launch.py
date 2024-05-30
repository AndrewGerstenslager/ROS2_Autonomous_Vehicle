from launch import LaunchDescription
from launch_ros.actions import Node
import os
import re
import subprocess
import sys

#CAMERA INFO (ID_SERIAL_SHORT)
#ACCESSED THROUGH RUNNING:
#  udevadm info --query=all --name=/dev/video0
#  videoX can be any open video connection in /dev
FRONT_CAM = {"ID_SERIAL_SHORT": "A231111000200156"}
LEFT_CAM = {"ID_SERIAL_SHORT": "A231111000209494"}
RIGHT_CAM = {"ID_SERIAL_SHORT": "A230401000215170"}

def find_cameras():
    """Finds all video device indices in /dev, queries their serial numbers, and identifies the camera."""
    dev_list = os.listdir('/dev')
    video_devices = [dev for dev in dev_list if re.match(r'video\d+', dev)]

    camera_ports = {}
    for video_device in video_devices:
        device_path = os.path.join('/dev', video_device)
        serial_number = get_serial_number(device_path)
        camera_name = identify_camera_by_serial(serial_number)
        if camera_name:
            camera_ports[camera_name] = device_path
    
    return camera_ports

def get_serial_number(device_path):
    """Queries the serial number of a device using udevadm."""
    try:
        output = subprocess.check_output(['udevadm', 'info', '--query=property', '--name=' + device_path])
        for line in output.decode('utf-8').split('\n'):
            if 'ID_SERIAL_SHORT' in line:
                return line.split('=')[1].strip()
    except subprocess.CalledProcessError as e:
        print(f"Error querying device {device_path}: {e}")
    return None

def identify_camera_by_serial(serial_number):
    """Identifies the camera based on its serial number."""
    if serial_number == FRONT_CAM['ID_SERIAL_SHORT']:
        return 'Front Camera'
    elif serial_number == LEFT_CAM['ID_SERIAL_SHORT']:
        return 'Left Camera'
    elif serial_number == RIGHT_CAM['ID_SERIAL_SHORT']:
        return 'Right Camera'
    return None

def generate_launch_description():
    camera_ports = find_cameras()
    required_cameras = ['Front Cam', 'Left Cam', 'Right Cam']
    launch_nodes = []

    for camera in required_cameras:
        if camera in camera_ports:
            node = Node(
                package='cpp_image_processing',
                executable='live_feed',
                name=f'{camera.lower().replace(" ", "_")}_live_feed',
                parameters=[{'device_path': camera_ports[camera]}, {'published_topic': f'/{camera.lower().replace(" ", "_")}/image_raw'}]
            )
            launch_nodes.append(node)
        else:
            print(f"Error: {camera} not found. Ensure all cameras are connected.")
            sys.exit(1)

    return LaunchDescription(launch_nodes)

