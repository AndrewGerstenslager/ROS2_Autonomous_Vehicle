from setuptools import setup
import os
import glob

package_name = 'dokalman_model_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='andrewgerstenslager@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
    
)
