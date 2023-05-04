from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_image_processing', 
            executable='live_feed', 
            name='live_feed',
            parameters=[{'device_path':'/dev/video2'}, 
                        {'published_topic':'img_raw'}]
        ),

        Node(
            package='cpp_image_processing', 
            executable='image_skew', 
            name='image_skew',
            parameters=[{'subscribed_topic':'img_raw'}, 
                        {'published_topic':'img_skewed'},
                        {"ipm_file_path":"/calibration_data/video2.txt"}]
        ),

        Node(
            package='cpp_image_processing',
            executable='line_threshold',
            name='line_threshold',
            parameters=[{'subscribed_topic': 'img_skewed'},
                        {'published_topic': 'img_processed'},
                        {"threshold_value":150}]
        ),
    ])
