from launch import LaunchDescription
from launch_ros.actions import Node
import os
import subprocess
import re

device1 = "A230401000215170" #right camera
device2 = "A231111000200156" #front camera
device3 = "A231111000209494" #left camera

def get_camera_serial(device):
    command = f"sudo v4l2-ctl --device={device} --all"
    output = subprocess.check_output(command.split()).decode('utf-8')
    match = re.search(r'Serial\s+:\s+(\w+)', output)
    return match.group(1) if match else None

def find_device_by_serial(target_serial):
    for i in range(10):  # adjust range as necesysary
        device = f"/dev/video{i}"
        if os.path.exists(device):
            serial = get_camera_serial(device)
            if serial == target_serial:
                return device
    return None

print(find_device_by_serial('A230401000215170'))
print(find_device_by_serial('A231111000200156'))
print(find_device_by_serial('A231111000209494'))

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_image_processing', 
            executable='live_feed', 
            name='right_live_feed',
            parameters=[{'serial_number':find_device_by_serial('A230401000215170')}, {'published_topic':'right_img_raw'}]
        ),
        Node(
            package='cpp_image_processing', 
            executable='live_feed', 
            name='front_live_feed',
            parameters=[{'serial_number':find_device_by_serial('A231111000200156')}, {'published_topic':'front_img_raw'}]
        ),
        Node(
            package='cpp_image_processing', 
            executable='live_feed', 
            name='left_live_feed',
            parameters=[{'serial_number':find_device_by_serial('A231111000209494')}, {'published_topic':'left_img_raw'}]
        ),

        # Node(
        #     package='cpp_image_processing',2
        #     executable='image_threshold',
        #     name='right_line_threshold',
        #     parameters=[{'subscribed_topic': 'right_img_raw'},
        #                 {'published_topic': 'right_img_processed'}]
        # ),
        # Node(
        #     package='cpp_image_processing',
        #     executable='image_threshold',
        #     name='left_line_threshold',
        #     parameters=[{'subscribed_topic': 'left_img_raw'},
        #                 {'published_topic': 'left_img_processed'}]
        # ),
    ])
