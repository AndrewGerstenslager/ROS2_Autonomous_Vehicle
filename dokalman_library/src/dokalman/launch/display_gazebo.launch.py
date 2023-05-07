from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
import pathlib
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Get the path to the robot package
    robot_pkg_dir = get_package_share_directory('dokalman')
    urdf_path = os.path.join(robot_pkg_dir, 'urdf', 'dokalman_description.urdf')
   
    # Start Gazebo with an empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            )
        ),
        launch_arguments={'verbose': 'false'}.items(),
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'dokalman', '-file', urdf_path],
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
    ])
