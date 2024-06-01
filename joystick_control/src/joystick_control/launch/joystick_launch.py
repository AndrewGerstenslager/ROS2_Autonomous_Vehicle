from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'dev': '/dev/input/js0'}]  # Adjust device path if needed
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',  # Use your teleop_node.py if desired
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[
                {'joy_config': 'xbox'},  # Use 'xbox' config for Xbox controller
                {'require_enable_button': True},  # Enable button requirement (optional)
                {'axis_linear.x': 1},  # Assuming left stick X for linear (adjust based on your controller)
                {'axis_linear.y': -1},  # Assuming left stick Y for linear (adjust based on your controller)
                {'scale_linear.x': 1.0},  # Adjust linear velocity scaling
                {'scale_linear.y': 1.0},  # Adjust linear velocity scaling
                {'axis_angular.yaw': 0},  # Assuming right stick X for angular (adjust based on your controller)
                {'scale_angular.yaw': 1.0},  # Adjust angular velocity scaling
            ]
        ),
    ])
