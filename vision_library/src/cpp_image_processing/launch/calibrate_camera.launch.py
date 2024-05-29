from launch import LaunchDescription
from launch_ros.actions import Node
import math

def generate_launch_description():  
    nodes=[]  
    nodes+=calibrate_camera('left_cam_img_raw','left')
    nodes+=calibrate_camera('right_cam_img_raw','right')
    nodes+=calibrate_camera('front_cam_img_raw','front')
    return LaunchDescription(
        nodes
    )
    
def calibrate_camera(camera,name):
    cal_name=name+'_calibrated'
    glo_ipm='/home/uc_jetson/repo/dokalman/vision_library/src/cpp_image_processing/calibration_data/'+name+'_test_cal.txt'
    return [Node(package='cpp_image_processing', 
            executable='camera_calibrate', 
            name='camera_calibrate',
            parameters=[{'subscribed_topic':camera}, 
                        {'published_topic':cal_name},
                        {'global_ipm_file_path':glo_ipm}])
    ]