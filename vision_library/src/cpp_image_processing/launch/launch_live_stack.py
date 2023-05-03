from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_image_processing', 
            executable='live_feed', 
            name='right_live_feed',
            parameters=[{'serial':'5EBF6F8F'}, {'published_topic':'right_img_raw'}]
        ),
        Node(
            package='cpp_image_processing', 
            executable='live_feed', 
            name='left_live_feed',
            parameters=[{'serial':'B1CF6F8F'}, {'published_topic':'left_img_raw'}]
        ),

        Node(
            package='cpp_image_processing',
            executable='line_threshold',
            name='right_line_threshold',
            parameters=[{'subscribed_topic': 'right_img_raw'},
                        {'published_topic': 'right_img_processed'}]
        ),
        Node(
            package='cpp_image_processing',
            executable='line_threshold',
            name='left_line_threshold',
            parameters=[{'subscribed_topic': 'left_img_raw'},
                        {'published_topic': 'left_img_processed'}]
        ),
    ])
