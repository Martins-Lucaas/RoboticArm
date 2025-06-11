import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    hand_pack_path = get_package_share_directory('hand_pack')
    urdf_file = os.path.join(hand_pack_path, 'urdf', 'linear_covvi_hand_right_scaled.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

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

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0', '0', '-1',    # x y z
                '0', '0', '0',    # roll pitch yaw
                'world',          # frame pai
                '__base_link'     
            ],
            output='screen'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(hand_pack_path, 'rviz', 'display_config.rviz')],
            output='screen'
        )
    ])
