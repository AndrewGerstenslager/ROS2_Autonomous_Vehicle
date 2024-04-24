from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='basic_navigation',
            executable='heading_from_odom_node',
            name='odom_node',
            output='screen'
        ),
        Node(
            package='basic_navigation',
            executable='waypoint_turning_node',
            name='turn_node',
            output='screen'
        ),
        Node(
            package='basic_navigation',
            executable='random_goal_setter',
            name='goal_setter',
            output='screen'
        ),
    ])