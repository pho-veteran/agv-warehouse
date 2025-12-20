from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'agv_transport_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/templates', glob('templates/*.html')),
        ('share/' + package_name + '/static/css', glob('static/css/*.css')),
        ('share/' + package_name + '/static/js', glob('static/js/*.js')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGV Developer',
    maintainer_email='developer@example.com',
    description='Web dashboard for AGV transport system monitoring and control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_dashboard = agv_transport_web.web_dashboard:main',
        ],
    },
)