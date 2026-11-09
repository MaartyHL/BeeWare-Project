import os
from glob import glob
from setuptools import setup

package_name = 'beeware_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Henry-Léo Martin',
    description='Contrôle de vol autonome BeeWare : hover PID + suivi ArUco vers PX4 Offboard.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'auto_hover_node = beeware_control.auto_hover_node:main',
            'aruco_follower = beeware_control.aruco_follower:main',
        ],
    },
)
