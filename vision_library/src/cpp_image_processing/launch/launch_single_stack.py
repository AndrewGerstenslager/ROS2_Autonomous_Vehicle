from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #Node(
        #    package='cpp_image_processing', 
        #    executable='live_feed', 
        #    name='live_feed',
        #    parameters=[{'device_path':'/dev/video2'}, 
        #                {'published_topic':'img_raw'}]
        #),
        Node(
            package='cpp_image_processing', 
            executable='image_skew', 
            name='image_skew',
            parameters=[{'subscribed_topic':'left_camera/image_raw'}, 
                        {'published_topic':'img_skewed'},
                        {"ipm_file_path":"/calibration_data/test2.txt"}]
        ),
        Node(
            package='cpp_image_processing',
            executable='image_threshold',
            name='image_threshold',
            parameters=[{'subscribed_topic': 'img_skewed'},
                        {'published_topic': 'img_processed'},
                        {"threshold_value":180}] #[0,255] The higher the number the more exact color it has to be
        ),

        Node(
            package='cpp_image_processing',
            executable='image_to_pointcloud',
            parameters=[
                {'scale': 1.0 / 255.0},
                {'input_topic': 'img_processed'},
                {'output_topic': 'pcd'}
            ]
        )
    ])
