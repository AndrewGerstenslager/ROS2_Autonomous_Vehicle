from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('dokalman_model_library')
    urdf = os.path.join(pkg_dir, 'urdf', 'dokalman_model.urdf')

    return LaunchDescription([
        Node(package='gazebo_ros', executable='gazebo', arguments=['-s', 'libgazebo_ros_factory.so'], output='screen'),
        Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity', 'dokalman_model', '-file', urdf], output='screen')
    ])
