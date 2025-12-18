from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'agv_auto_explore'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGV Developer',
    maintainer_email='developer@agv.local',
    description='Autonomous exploration package for AGV warehouse system using frontier-based exploration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'exploration_node = agv_auto_explore.exploration_node:main',
        ],
    },
)
