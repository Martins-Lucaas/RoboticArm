import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = Path(get_package_share_directory('covvi'))
    urdf_path = pkg_share / 'urdf' / 'cr10_robot.urdf'   # << aqui!

    if not urdf_path.exists():
        raise FileNotFoundError(f'URDF não encontrado: {urdf_path}')

    robot_description = urdf_path.read_text()

    return LaunchDescription([
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             name='joint_state_publisher_gui', output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='world_to_base_link',
             arguments=['0','0','0','0','0','0','1','world','base_link'], output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen'),
    ])
