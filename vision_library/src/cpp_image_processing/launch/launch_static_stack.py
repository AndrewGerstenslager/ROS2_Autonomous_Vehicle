from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_image_processing', 
            executable='raw_image_static', 
            name='right_image_static',
            parameters=[{'topic_name_pub':'right_img_raw'}]
        ),
        Node(
            package='cpp_image_processing', 
            executable='raw_image_static', 
            name='left_image_static',
            parameters=[{'topic_name_pub':'left_img_raw'}]
        ),

        Node(
            package='cpp_image_processing',
            executable='image_processor',
            name='right_image_processor',
            parameters=[{'topic_name_sub': 'right_img_raw'},
                        {'topic_name_pub': 'right_img_processed'}]
        ),
        Node(
            package='cpp_image_processing',
            executable='image_processor',
            name='left_image_processor',
            parameters=[{'topic_name_sub': 'left_img_raw'},
                        {'topic_name_pub': 'left_img_processed'}]
        ),
    ])