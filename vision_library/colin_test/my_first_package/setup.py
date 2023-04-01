from setuptools import setup

package_name = 'my_first_package'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: First test',
    license='TODO: N/A',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_pub_node = my_first_package.image_publisher:main',
            'image_sub_node = my_first_package.image_subscriber:main',
            'image_process_node = my_first_package.image_process_subscriber:main',
        ],
    },
)
