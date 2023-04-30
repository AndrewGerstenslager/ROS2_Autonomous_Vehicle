from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_image_processing', 
            executable='live_feed', 
            name='right_live_feed',
            parameters=[{'serial':'5EBF6F8F'}, {'topic':'right_img_raw'}]
        ),
        Node(
            package='cpp_image_processing', 
            executable='live_feed', 
            name='left_live_feed',
            parameters=[{'serial':'B1CF6F8F'}, {'topic':'left_img_raw'}]
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
