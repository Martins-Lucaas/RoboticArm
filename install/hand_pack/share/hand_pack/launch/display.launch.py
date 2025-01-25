import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    hand_pack_path = get_package_share_directory('hand_pack')
    urdf_file = os.path.join(hand_pack_path, 'urdf', 'hand.xacro')

    # Converte Xacro para URDF
    robot_description = Command(['xacro ', urdf_file])

    # Define parâmetros
    params = {
        'robot_description': robot_description
    }

    return LaunchDescription([
        # Publicação do joint_state_publisher_gui
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Publicação do robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[params]
        ),

        # Publicação do TF base
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
            output='screen'
        ),

        # Lançamento do RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(hand_pack_path, 'rviz', 'display_config.rviz')],
            output='screen'
        )
    ])
