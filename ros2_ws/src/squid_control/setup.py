from setuptools import setup
import os
from glob import glob

package_name = 'squid_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Squid Student',
    maintainer_email='student@squid.drone',
    description='ROS 2 implementation of the Squid Drone control stack.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_mixer = squid_control.motor_mixer_node:main',
        ],
    },
)
