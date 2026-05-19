"""Launch standalone da mão COVVI no Gazebo.

Lê o URDF cru do Onshape, aplica os patches de:
  - limites factíveis das juntas (manual COVVI: 81° flexão);
  - dinâmica para grasp por contato (effort, damping);
  - "pele" macia em falanges e palma;
  - supressão de auto-colisão intra-dedo.

E publica o `robot_description` resultante para o `robot_state_publisher`
+ Gazebo via `spawn_entity`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from hand_pack.urdf_helpers import apply_all


def _load_patched_urdf(urdf_path: str) -> str:
    with open(urdf_path, 'r') as f:
        raw = f.read()
    return apply_all(raw, skin_inflate_m=0.002)


def generate_launch_description():
    package_name = 'hand_pack'
    pkg_share = get_package_share_directory(package_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'linear_covvi_hand_gazebo.urdf')

    robot_desc = _load_patched_urdf(urdf_file)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'covvi_hand', '-z', '0.1'],
        output='screen'
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output='screen'
    )

    load_hand_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_position_controller", "--controller-manager", "/controller_manager"],
        output='screen'
    )

    delay_broadcaster_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    delay_controller_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_hand_position_controller],
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_entity,
        delay_broadcaster_after_spawn,
        delay_controller_after_broadcaster
    ])
