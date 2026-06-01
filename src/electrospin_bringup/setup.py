from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'electrospin_bringup'

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
            'system_monitor = electrospin_bringup.system_monitor_node:main',
            'command_bridge = electrospin_bringup.command_bridge_node:main',
            'passive_collector = electrospin_bringup.passive_collector_node:main',
            'presentation_game = electrospin_bringup.presentation_game_node:main',
        ],
    },
)
