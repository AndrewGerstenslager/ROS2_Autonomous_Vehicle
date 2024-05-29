from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

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
            executable='generate_bearing',
            name='generate_bearing',
            output='screen'
        ),
        Node(
            package='basic_navigation',
            executable='visualize_bearing',
            name='visualize_bearing',
            output='screen'
        ),
        # Node(
        #     package='basic_navigation',
        #     #executable='run_neural_net',
        #     executable='waypoint_turning_node',
        #     name='driving_node',
        #     output='screen'
        # ),
        TimerAction(
            period=1.5,
            actions=[
                Node(
                    package='basic_navigation',
                    executable='random_goal_setter',
                    name='random_goal_setter',
                    output='screen'
                ),
            ]
        ),
        TimerAction(
            period=1.5,
            actions=[
                Node(
                    package='basic_navigation',
                    executable='random_goal_setter',
                    name='random_goal_setter',
                    output='screen'
                ),
            ]
        ),
    ])