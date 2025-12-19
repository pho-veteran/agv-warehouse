from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'agv_transport'

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
    description='AGV Transport Task Manager - FSM-based transport task management for warehouse AGV',
    license='Apache-2.0',
    tests_require=['pytest', 'hypothesis'],
    entry_points={
        'console_scripts': [
            'transport_task_manager_node = agv_transport.transport_task_manager_node:main',
            'send_transport_order = agv_transport.scripts.send_transport_order:main',
        ],
    },
)
