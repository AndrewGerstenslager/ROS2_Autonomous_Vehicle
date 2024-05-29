from setuptools import setup
from glob import glob

package_name = 'basic_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/waypoints', glob('waypoints/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/tf_models', glob('tf_models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vscode',
    maintainer_email='vscode@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'heading_from_odom_node = basic_navigation.odom:main',
            'waypoint_turning_node = basic_navigation.turn_to_waypoint:main',
            'goal_setter = basic_navigation.waypoint_goal_setter:main',
            'random_goal_setter = basic_navigation.random_waypoint_goal_setter:main',
            'train_neural_net = basic_navigation.train_neural_net:main',
            'run_neural_net = basic_navigation.run_neural_net:main',
            'data_collection = basic_navigation.data_collection:main',
            'generate_bearing = basic_navigation.bearing:main',
            'visualize_bearing = basic_navigation.visualize_bearing:main',
            'publish_initial_pose = basic_navigation.publish_initial_pose:main',
        ],
    },
)
