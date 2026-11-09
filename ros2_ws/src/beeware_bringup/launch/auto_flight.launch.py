"""Launch du vol automatique BeeWare.

Usage (sur la Pi, dans l'environnement ROS 2 Humble, après colcon build + source) :
  ros2 launch beeware_bringup auto_flight.launch.py
  ros2 launch beeware_bringup auto_flight.launch.py bench_mode:=false mode:=track

La vision (aruco_live_stream.py) tourne avec le Python système à cause de picamera2 :
  ros2 launch beeware_bringup auto_flight.launch.py \
      start_vision:=true vision_script:=/home/martin/BeeWare-Project/ros2_ws/src/beeware_vision/beeware_vision/aruco_live_stream.py
ou dans un terminal séparé :
  python3 ~/BeeWare-Project/ros2_ws/src/beeware_vision/beeware_vision/aruco_live_stream.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bench_mode = LaunchConfiguration('bench_mode')
    mode = LaunchConfiguration('mode')
    start_vision = LaunchConfiguration('start_vision')
    vision_script = LaunchConfiguration('vision_script')

    control_params = PathJoinSubstitution([
        FindPackageShare('beeware_control'), 'config', 'auto_hover.yaml'
    ])

    control_node = Node(
        package='beeware_control',
        executable='auto_hover_node',
        name='auto_hover_node',
        output='screen',
        parameters=[control_params, {'bench_mode': bench_mode, 'mode': mode}],
    )

    # Vision avec le Python système (/usr/bin/python3) si un chemin est fourni.
    # env -u PYTHONPATH/LD_LIBRARY_PATH : indispensable, sinon le Python système
    # hérite des site-packages de l'env ROS (python 3.14) et numpy/cv2 plantent.
    vision_proc = ExecuteProcess(
        cmd=['env', '-u', 'PYTHONPATH', '-u', 'LD_LIBRARY_PATH',
             '/usr/bin/python3', vision_script],
        output='screen',
        condition=IfCondition(start_vision),
    )

    return LaunchDescription([
        DeclareLaunchArgument('bench_mode', default_value='true',
                              description='true = banc sans hélices, false = vol réel'),
        DeclareLaunchArgument('mode', default_value='auto',
                              description='auto (suit l ArUco si visible, sinon hover) | hover | off'),
        DeclareLaunchArgument('start_vision', default_value='false',
                              description='lance aussi aruco_live_stream.py (Python système)'),
        DeclareLaunchArgument('vision_script', default_value='',
                              description='chemin absolu de aruco_live_stream.py'),
        control_node,
        vision_proc,
    ])
