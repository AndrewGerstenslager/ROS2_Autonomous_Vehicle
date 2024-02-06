import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name='dokalman_library'
    package_path=get_package_share_directory(package_name)

    models_dir = os.path.join(package_path, 'worlds', 'models')
    print("Models directory: ", models_dir)  # For debugging

    models_var = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', models_dir
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    package_path,'launch','robot_state_pub.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot',
                                   '-z', '1.0'],
                        output='screen')

    return LaunchDescription([
        models_var,
        rsp,
        spawn_entity,
    ])
