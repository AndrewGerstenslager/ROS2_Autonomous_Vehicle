import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node

def generate_launch_description():

    package_name='dokalman_library'
    
    # Find location of the dokalman_library package
    package_path=get_package_share_directory(package_name)

    # Define the path to the rviz config file
    rviz_config_file = PathJoinSubstitution([package_path, 'config', 'view_robot.rviz'])
    
    # Build the full path to your models directory
    models_dir = os.path.join(package_path, 'worlds', 'models')
    print("Models directory: ", models_dir)  # Add this line
    
    # Set GAZEBO_MODEL_PATH environment variable
    models_var = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', models_dir
    )

    # Include launch file to publish joint states of robot
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    package_path,'launch','robot_state_pub.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot',
                                   '-z', '1.0'],
                        output='screen')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        models_var,
        rsp,
        spawn_entity,
        #rviz_node,
    ])
