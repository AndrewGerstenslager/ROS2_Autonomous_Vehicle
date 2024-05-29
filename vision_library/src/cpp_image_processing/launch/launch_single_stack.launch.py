from launch import LaunchDescription
from launch_ros.actions import Node
import math

def generate_launch_description():    
    nodes=[]
    # right and left camera in this case are from looking
    # from the front to the back perspective
    
    # Angle of the camera from the front camera
    # Front cam is at 0 degree, and counter-clockwise is positive
    left_camera_angle=90.0 
    right_camera_angle=-90.0
    
    #process_image(input,name_topic,angle_yaw,x_translation,y_translation)
    nodes+=process_image('right_cam_img_raw','right',180.0+right_camera_angle,0.35,0.0) 
    nodes+=process_image('left_cam_img_raw','left',180.0+left_camera_angle,-0.35,0.0)
    nodes+=process_image('front_cam_img_raw','front',180.0,0.0,0.48)
    
    return LaunchDescription(
        nodes
    )
    
# positive yaw means counter-clockwise
# return a list of 3 nodes (skewed, black&white filtered, and cloudpoints)
# trans_x and trans_y can be calibrated in rviz by 
# manually changing the numbers until it looks right
def process_image(camera,name,yaw,trans_x,trans_y):
    skew_topic=name+'_img_skewed'
    threshold_topic=name+'_img_processed'
    pcd_topic=name+'_pcd'
    ipm_path='/calibration_data/'+name+'_test_cal.txt'
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
                        {"ipm_file_path":ipm_path}]
        )
    threshold=Node(
            package='cpp_image_processing',
            executable='image_threshold',
            name='image_threshold',
            parameters=[{'subscribed_topic':skew_topic},
                        {'published_topic':threshold_topic},
                        {"threshold_value":165}] #[0,255] The higher the number the stricter the white color filter
        )
    pcd=Node(
            package='cpp_image_processing',
            executable='image_to_pointcloud',
            parameters=[
                {'ipm_file_path':ipm_path},
                {'yaw' : yaw},
                {'trans_x': trans_x},
                {'trans_y':trans_y},
                {'input_topic': threshold_topic},
                {'output_topic': pcd_topic}
            ]
        )
    return [skew,threshold,pcd]