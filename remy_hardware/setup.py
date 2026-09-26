from setuptools import setup
import os
from glob import glob

package_name = 'remy_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sarah',
    maintainer_email='diassarah238@gmail.com',
    description='Pacote de navegação com Nav2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cmd_vel_to_serial = remy_hardware.cmd_vel_to_serial:main',
            'cmd_vel = remy_hardware.cmd_vel:main',
        ],
    },
)

