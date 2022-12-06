# __VISION LIBRARY__

## __Requirements for this Library__

- ros2 - Foxy
- Python
    - opencv-python (cv2)
    - matplotlib (using pyplot specifically)
    - jupyter notebook interpreting tools [see example here](https://marketplace.visualstudio.com/items?itemName=ms-toolsai.jupyter)

This library is used to handle all image processing needs of Dokalman.

## __Preprocessing Images__

We have a jupyter notebook designed for saving the frames of a video to a directory [see here](/vision_library/src/dokalman_vision/libraries/save_vid_frames.ipynb)


## __ROS2 Nodes__

-   camera_pub
    - _Description: Takes in images from left and right images and publishes raw images to ROS2_
    - pub: left_img_raw, right_img_raw
- dummy_cam_pub
    - _Description: Substitute for camera_pub node. Takes a directory and publishes images to replicate live camera feed for testing_
    - pub: left_camera_frames
    - pub: right_camera_frames
-   image_processing_cv
    - _Description: Uses opencv to extract lines out of image and publishes processed images to ROS2_
    - sub: left_img_raw, right_img_raw
    - pub: left_img_processed, right_img_processed
-   img_to_pointcloud
    - _Description: converts processed images into pointcloud that represents obstacle map of the lines relative to the center of the robot_
    - sub: left_img_processed, right_img_processed
    - pub: pointcloud



