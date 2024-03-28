from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_image_processing', 
            executable='live_feed', 
            name='right_live_feed',
            parameters=[{'serial_number':'A230401000215170'}, {'published_topic':'right_img_raw'}]
        ),
        Node(
            package='cpp_image_processing', 
            executable='live_feed', 
            name='front_live_feed',
            parameters=[{'serial_number':'A231111000200156'}, {'published_topic':'front_img_raw'}]
        ),
        Node(
            package='cpp_image_processing', 
            executable='live_feed', 
            name='left_live_feed',
            parameters=[{'serial_number':'A231111000209494'}, {'published_topic':'left_img_raw'}]
        ),

        # Node(
        #     package='cpp_image_processing',
        #     executable='image_threshold',
        #     name='right_line_threshold',
        #     parameters=[{'subscribed_topic': 'right_img_raw'},
        #                 {'published_topic': 'right_img_processed'}]
        # ),
        # Node(
        #     package='cpp_image_processing',
        #     executable='image_threshold',
        #     name='left_line_threshold',
        #     parameters=[{'subscribed_topic': 'left_img_raw'},
        #                 {'published_topic': 'left_img_processed'}]
        # ),
    ])
