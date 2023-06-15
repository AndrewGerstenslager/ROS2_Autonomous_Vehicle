from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_image_processing', 
            executable='static_feed', 
            name='right_image_static',
            parameters=[{'published_topic':'right_img_raw'}]
        ),
        Node(
            package='cpp_image_processing', 
            executable='static_feed', 
            name='left_image_static',
            parameters=[{'published_topic':'left_img_raw'}]
        ),

        Node(
            package='cpp_image_processing',
            executable='image_threshold',
            name='image_threshold',
            parameters=[{'subscribed_topic': 'right_img_raw'},
                        {'published_topic': 'right_img_processed'}]
        ),
        Node(
            package='cpp_image_processing',
            executable='image_threshold',
            name='image_threshold',
            parameters=[{'subscribed_topic': 'left_img_raw'},
                        {'published_topic': 'left_img_processed'}]
        ),
    ])