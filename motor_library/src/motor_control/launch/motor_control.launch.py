import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#TODO: Lets automatically grab the serial port of the robot. 

def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the Arduino'
    )

    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.5207',
        description='Distance between wheels in meters'
    )

    wheel_diameter_arg = DeclareLaunchArgument(
        'wheel_diameter',
        default_value='0.318',
        description='Diameter of the wheels in meters'
    )

    # Define nodes
    serial_interface_node = Node(
        package='motor_control',
        executable='serial_interface',
        name='serial_interface_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port')
        }]
    )

    encoder_odom_node = Node(
        package='motor_control',
        executable='encoder_odom',
        name='encoder_odom_node',
        output='screen',
        parameters=[{
            'wheel_base': LaunchConfiguration('wheel_base'),
            'wheel_diameter': LaunchConfiguration('wheel_diameter')
        }]
    )

    # Create and return launch description
    return LaunchDescription([
        serial_port_arg,
        wheel_base_arg,
        wheel_diameter_arg,
        serial_interface_node,
        encoder_odom_node
    ])
