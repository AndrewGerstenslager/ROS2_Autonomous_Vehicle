import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    package_name = 'dokalman_library'
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path, 'urdf', 'DokalmanCore.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
        
    # Set GAZEBO_MODEL_PATH environment variable
    models_dir = os.path.join(get_package_share_directory(package_name), 'worlds', 'models')
    models_var = SetEnvironmentVariable('GAZEBO_MODEL_PATH', models_dir)

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': os.path.join(get_package_share_directory(package_name), 'worlds', 'line_det_course.world')}.items()
    )

    # Spawn entity in Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description', '-entity', 'my_bot', '-z', '1.0'],
                        output='screen')

    # Create a caster_joint_publisher node
    caster_joint_publisher = Node(
        package='dokalman_library',
        executable='caster_joint_publisher',
        output='screen',
    )

    # Create a heading_publisher node
    odom_heading_node = Node(
    package='dokalman_library',
    executable='heading_publisher',
    output='screen',
)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true'),
        models_var,
        gazebo,
        spawn_entity,
        robot_state_publisher,
        caster_joint_publisher,
        odom_heading_node,
    ])
