import cv2

# Initialize the video capture object. 0 is usually the default camera.
cap = cv2.VideoCapture('/sys/bus/usb/devices/usb1/1-1/port')

# Check if the webcam is opened correctly
if not cap.isOpened():
    raise IOError("Cannot open webcam")

while True:
    ret, frame = cap.read()

    # If frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    # Display the resulting frame
    cv2.imshow('Webcam Feed', frame)

    # Break the loop with the 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the video capture object and close windows
cap.release()
cv2.destroyAllWindows()
