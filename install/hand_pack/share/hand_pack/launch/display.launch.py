import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    hand_pack_path = get_package_share_directory('hand_pack')
    urdf_file = os.path.join(hand_pack_path, 'urdf', 'ar10.urdf')

    # Read the URDF file directly
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Define parameters
    params = {'robot_description': robot_description}

    return LaunchDescription([
        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[params]
        ),

        # Static TF Publisher (World to Base Link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'wrist_plate_to_circuit_support'],
            output='screen'
        ),

        # RViz2 Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(hand_pack_path, 'rviz', 'display_config.rviz')],
            output='screen'
        )
    ])