from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'vision_system'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'vision_system_node = vision_system.vision_controller_node:main',
            'collector_tracker_node = vision_system.collector_tracker_node:main',
        ],
    },
)
