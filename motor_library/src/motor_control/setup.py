from setuptools import setup, find_packages

package_name = 'motor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/motor_control_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrew Gerstenslager',
    maintainer_email='gerstead@mail.uc.edu',
    description='TODO: Package description',
    license='There is no license for this package at this time.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_node = motor_control.motor_subscriber_node:main',
            'serial_interface = motor_control.serial_interface:main',
            'encoder_odom = motor_control.encoder_odom:main',
        ],
    },
)
