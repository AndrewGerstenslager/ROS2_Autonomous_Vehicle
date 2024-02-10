from launch import LaunchDescription
from launch_ros.actions import Node
import math
def generate_launch_description():    
    nodes=[]
    nodes+=process_image('right_camera/image_raw','right',180.0+70.0,-10.74,-2.1) 
    nodes+=process_image('left_camera/image_raw','left',180.0-70.0,-2.2,-4.4)
    return LaunchDescription(
        nodes
    )
#positive yaw means counter-clockwise
def process_image(camera,name,yaw,trans_x,trans_y):
    skew_topic=name+'_img_skewed'
    threshold_topic=name+'_img_processed'
    pcd_topic=name+'_pcd'
    #Node(
    #    package='cpp_image_processing', 
    #    executable='live_feed', 
    #    name='live_feed',
    #    parameters=[{'device_path':'/dev/video2'}, 
    #                {'published_topic':'img_raw'}]
    #),
    skew=Node(
            package='cpp_image_processing', 
            executable='image_skew', 
            name='image_skew',
            parameters=[{'subscribed_topic':camera}, 
                        {'published_topic':skew_topic},
                        {"ipm_file_path":"/calibration_data/test2.txt"}]
        )
    threshold=Node(
            package='cpp_image_processing',
            executable='image_threshold',
            name='image_threshold',
            parameters=[{'subscribed_topic':skew_topic},
                        {'published_topic':threshold_topic},
                        {"threshold_value":180}] #[0,255] The higher the number the more exact color it has to be
        )
    pcd=Node(
            package='cpp_image_processing',
            executable='image_to_pointcloud',
            parameters=[
                {'scale': 5.5*(1.0 / 255.0)},
                {'yaw' : yaw},
                {'trans_x': trans_x},
                {'trans_y':trans_y},
                {'input_topic': threshold_topic},
                {'output_topic': pcd_topic}
            ]
        )
    return [skew,threshold,pcd]