from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the URDF file
    urdf = os.path.join(get_package_share_directory('dokalman_model_package'), 'urdf', 'dokalman_model.urdf')
    print("\n\n\n\nURDF",os.path.isfile(urdf))
    # Set robot description parameter
    robot_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf).read()}]
    )

    # Launch RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('dokalman_model_package'), 'rviz', 'my_robot.rviz')],
        remappings=[('/tf', '/tf_static')]
    )

    # Compose the launch description
    ld = LaunchDescription()

    # Add the launch actions
    ld.add_action(robot_description)
    ld.add_action(rviz)

    return ld
