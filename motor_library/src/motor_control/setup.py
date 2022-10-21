from setuptools import setup

package_name = 'motor_control'

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
    maintainer='Andrew Gerstenslager',
    maintainer_email='gerstead@mail.uc.edu',
    description='TODO: Package description',
    license='There is no license for this package at this time.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_node = motor_control.motor_subscriber_node:main',
        ],
    },
)
