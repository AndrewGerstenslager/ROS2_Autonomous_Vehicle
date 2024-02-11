from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'dokalman_library'
    # Get the share directory of your package
    share_dir = get_package_share_directory(package_name)

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            parameters=[
                {'use_sim_time': True},
                f'{share_dir}/config/mapper_params_online_async.yaml'
            ],
            output='screen'
        ),
    ])