from setuptools import setup
import os

package_name = 'basic_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['waypoints/waypoints.json']),
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
            'planner_node = basic_navigation.planner_node:main',
            'executor_node = basic_navigation.executor_node:main',
            'heading_from_pose_node = basic_navigation.heading_from_pose_node:main',
        ],
    },
)
