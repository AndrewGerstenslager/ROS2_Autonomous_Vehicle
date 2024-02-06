from setuptools import setup

package_name = 'pointcloud_library'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'point_cloud_publisher = pointcloud_library.point_cloud_publisher:main',
            'image_to_pointcloud = pointcloud_library.image_to_pointcloud_publisher:main',
        ],
    },
)
