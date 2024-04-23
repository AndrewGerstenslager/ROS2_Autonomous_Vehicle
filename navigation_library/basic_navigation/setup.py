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
            'waypoint_follower = basic_navigation.basic_waypoint_follower:main',
            'goal_setter = basic_navigation.waypoint_goal_setter:main',
        ],
    },
)
