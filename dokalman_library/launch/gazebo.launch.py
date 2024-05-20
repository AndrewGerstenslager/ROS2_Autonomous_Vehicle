# Run this with rviz with the following command:
# ros2 launch dokalman_library gazebo.launch.py with_rviz:=true

# Run this without rviz with either of the following commands:
# ros2 launch dokalman_library gazebo.launch.py
# ros2 launch dokalman_library gazebo.launch.py with_rviz:=false

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import os

def generate_launch_description():
    package_name = 'dokalman_library'
    use_sim_time = LaunchConfiguration('use_sim_time')
    with_rviz = LaunchConfiguration('with_rviz')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path, 'urdf', 'DokalmanCore.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
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
                        arguments=['-topic', 'robot_description', '-entity', 'my_bot', '-z', '0.25'],
                        output='screen')

    # Include the RViz launch file conditionally with a delay
    rviz = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name), 'launch', 'rviz.launch.py')]),
                condition=IfCondition(with_rviz)
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true'),
        DeclareLaunchArgument('with_rviz', default_value='false', description='Launch RViz if true'),
        models_var,
        gazebo,
        spawn_entity,
        node_robot_state_publisher,
        rviz
    ])