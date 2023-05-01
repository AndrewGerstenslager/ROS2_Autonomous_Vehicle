from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution, Command
from launch.substitutions import LaunchConfiguration
import os
import launch

def generate_launch_description():
    pkg_dir = get_package_share_directory('dokalman_model_package')
    urdf = os.path.join(pkg_dir, 'urdf', 'dokalman_model.urdf')
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        Node(
            package='gazebo_ros', 
            executable='spawn_entity.py', 
            arguments=['-entity', 'dokalman_model', '-file', urdf], 
            output='screen'
        )
    ])
