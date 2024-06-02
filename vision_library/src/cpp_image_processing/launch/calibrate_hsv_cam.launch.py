from launch import LaunchDescription
from launch_ros.actions import Node
import math

def generate_launch_description():  
    nodes=[]  
    nodes+=calibrate_camera('/front_camera/image_raw','front')
    return LaunchDescription(
        nodes
    )
    
def calibrate_camera(camera,name):
    cal_name=name+'_hsv_calibrated'
    debug_name=cal_name+'_debug'
    #glo_ipm='/home/uc_jetson/repo/dokalman/vision_library/src/cpp_image_processing/calibration_data/'+name+'_test_cal.txt'
    glo_ipm='/dokalman/vision_library/src/cpp_image_processing/calibration_data/'+name+'_test_cal.txt'
    
    return [Node(
            package='cpp_image_processing',
            executable='image_threshold',
            name='image_threshold',
            parameters=[{'subscribed_topic':camera},
                        {'published_topic':cal_name},
                        {'published_debug_topic':debug_name},
                        {'min_area': 20},
                        {'debug_mode': True},
                        {'remove_orange': True}
                        ])
            
    ]