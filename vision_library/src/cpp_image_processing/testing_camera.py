import cv2
import subprocess
import os
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
                cap = cv2.VideoCapture(f"/dev/video{i}") 
                return cap
    return None

cap = find_device_by_serial(device3)

if not cap.isOpened():
    print("Error: Could not open video device.")
    exit()


# Set the desired resolution to 1920x1080.
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# Additionally, set the video format to MJPG.
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))

# Capture a frame
ret, frame = cap.read()

if ret:
    # Resize the frame to a smaller size for display
    display_frame = cv2.resize(frame, (960, 540))  # Resize to half of 1920x1080

    # Display the resized image
    cv2.imshow('Resized Frame', display_frame)
    cv2.waitKey(0)  # Wait for a key press to exit
    cv2.destroyAllWindows()
else:
    print("Error: Could not capture a frame.")

# Release the video capture device
cap.release()
