import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node

def generate_launch_description():

    package_name='dokalman_library'

    # Define the path to the DokalmanCore.xacro file
    dokalman_library_path = FindPackageShare(package_name)
    dokalman_xacro_file = PathJoinSubstitution([dokalman_library_path, 'urdf', 'DokalmanCore.xacro'])

    # Define the path to the rviz config file
    rviz_config_file = PathJoinSubstitution([dokalman_library_path, 'config', 'view_robot.rviz'])

    # Convert the XACRO file to URDF and load the content into the 'robot_description' parameter
    robot_description_content = Command(['xacro ', dokalman_xacro_file])
    robot_description_param = ParameterValue(robot_description_content, value_type=str)

    # Set GAZEBO_MODEL_PATH environment variable
    models_dir = os.path.join(get_package_share_directory(package_name), 'worlds', 'models')
    models_var = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', models_dir
    )

    # Include launch file to publish joint states of robot
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','robot_state_pub.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                launch_arguments={'world': os.path.join(get_package_share_directory(package_name), 'worlds', 'line_det_course.world')}.items()
             )

    # Run the spawner node from the gazebo_ros package
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot',
                                   '-z', '1.0'],
                        output='screen')

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
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Node to publish the static transform from base_link to laser_frame
    static_tf_pub_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0', '0', '0.35', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        models_var,
        rsp,
        gazebo,
        spawn_entity,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        static_tf_pub_laser
    ])
