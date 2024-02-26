from launch import LaunchDescription
from launch_ros.actions import Node
import math

def generate_launch_description():    
    return LaunchDescription([
        Node(package='cpp_image_processing', 
            executable='camera_calibrate', 
            name='camera_calibrate',
            parameters=[{'subscribed_topic':'left_camera/image_raw'}, 
                        {'published_topic':'left_camera/image_cal'},
                        {'local_ipm_file_path':'/calibration_data/test_cal.txt'},
                        {'global_ipm_file_path':'/dokalman/vision_library/src/cpp_image_processing/calibration_data/test_cal.txt'}])
    ])