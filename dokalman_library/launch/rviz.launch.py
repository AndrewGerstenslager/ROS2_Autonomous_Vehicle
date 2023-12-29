from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    # Define the path to the DokalmanCore.xacro file
    dokalman_library_path = FindPackageShare('dokalman_library')
    dokalman_xacro_file = PathJoinSubstitution([dokalman_library_path, 'urdf', 'DokalmanCore.xacro'])

    # Define the path to the rviz config file
    rviz_config_file = PathJoinSubstitution([dokalman_library_path, 'config', 'view_robot.rviz'])

    # Convert the XACRO file to URDF and load the content into the 'robot_description' parameter
    robot_description_content = Command(['xacro ', dokalman_xacro_file])
    robot_description_param = ParameterValue(robot_description_content, value_type=str)

    # Define the nodes
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_param}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
