from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_image_processing', 
            executable='live_feed', 
            name='live_feed',
            parameters=[{'device_path':'/dev/video0'}, 
                        {'published_topic':'img_raw_0'}]
        ),
        TimerAction(
            period="5",
            actions=[
            Node(
            package='cpp_image_processing', 
            executable='live_feed', 
            name='live_feed',
            parameters=[{'device_path':'/dev/video2'}, 
                        {'published_topic':'img_raw_1'}]
        )]),
        Node(
            package='cpp_image_processing', 
            executable='live_feed', 
            name='live_feed',
            parameters=[{'device_path':'/dev/video4'}, 
                        {'published_topic':'img_raw_2'}]
        ),
        # Node(
        #     package='cpp_image_processing', 
        #     executable='image_skew', 
        #     name='image_skew',
        #     parameters=[{'subscribed_topic':'img_raw_0'}, 
        #                 {'published_topic':'img_skewed'},
        #                 {"ipm_file_path":"/calibration_data/video2.txt"}]
        # ),
        # Node(
        #     package='cpp_image_processing',
        #     executable='image_threshold',
        #     name='image_threshold',
        #     parameters=[{'subscribed_topic': 'img_skewed'},
        #                 {'published_topic': 'img_processed'},
        #                 {"threshold_value":140}]
        # ),
        # Node(
        #     package='cpp_image_processing',
        #     executable='image_to_pointcloud',
        #     name='image_to_pointcloud',
        #     parameters=[{'subscribed_topic': 'img_processed'},
        #                 {'published_topic': 'my_pointcloud'}]
        # ),
    ])
