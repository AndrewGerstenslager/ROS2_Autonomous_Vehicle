import cv2
import os
import re

def find_even_cameras():
    """Finds all even-numbered video device indices in /dev."""
    dev_list = os.listdir('/dev')
    video_devices = [dev for dev in dev_list if re.match(r'video\d+', dev)]
    even_video_devices = [int(dev.replace('video', '')) for dev in video_devices if int(dev.replace('video', '')) % 2 == 0]
    return even_video_devices

def open_camera(device_index):
    """Opens a video capture device based on the device index."""
    cap = cv2.VideoCapture(device_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"Cannot open camera at /dev/video{device_index}")
    return cap

def main():
    # Find and open all even-numbered camera devices
    even_cameras = find_even_cameras()
    cameras = [open_camera(i) for i in even_cameras]

    if not cameras:
        print("No even-numbered cameras found.")
        return

    while all(cam.isOpened() for cam in cameras):
        # Read frames from all cameras
        for idx, cam in enumerate(cameras):
            ret, frame = cam.read()
            if not ret:
                print(f"Failed to read frame from camera at /dev/video{even_cameras[idx]}")
                continue
            # Display each frame in a separate window
            #cv2.imshow(f'Camera {even_cameras[idx]}', frame)

        # Break the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release all cameras and destroy all windows
    for cam in cameras:
        cam.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

