from setuptools import setup
import os
from glob import glob

package_name = 'navegation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*.yaml') + glob('maps/*.pgm') + glob('maps/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sarah',
    maintainer_email='diassarah238@gmail.com',
    description='Pacote de navegação com Nav2',
    license='MIT',
    entry_points={
        'console_scripts': [
                        'laser_to_pointcloud = navegation.laser_to_pointcloud:main',
                        'ps4_teleop_node = navegation.ps4_teleop_node:main',
                        'joystick_reader = navegation.joystick_reader:main',
                        'serial_controller = navegation.serial_controller:main',
                        'odom_pub = navegation.odom_pub:main',
                        'pid_converter = navegation.pid_converter:main',
            ],
    },
)
