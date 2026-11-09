from setuptools import setup

package_name = 'beeware_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Henry-Léo Martin',
    description='Vision BeeWare : détection ArUco + stream FPV + UDP vers le contrôle.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'aruco_live_stream = beeware_vision.aruco_live_stream:main',
            'generate_aruco = beeware_vision.generate_aruco:main',
        ],
    },
)
