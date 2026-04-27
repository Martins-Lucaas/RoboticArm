import os
import re
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _build_gazebo_urdf():
    hand_pack_share = get_package_share_directory('hand_pack')
    combined_yaml = os.path.join(
        hand_pack_share, 'config', 'cr10_covvi_controllers.yaml')

    # --- CR10 arm (process xacro) ---
    cr10_xacro_path = os.path.join(
        get_package_share_directory('cra_description'), 'urdf', 'cr10_robot.xacro')
    doc = xacro.parse(open(cr10_xacro_path))
    xacro.process_doc(doc)
    cr10_urdf = doc.toxml()

    # Replace CR10 controller yaml path with combined yaml
    cr10_urdf = re.sub(
        r'<parameters>[^<]*/ros2_controllers\.yaml</parameters>',
        f'<parameters>{combined_yaml}</parameters>',
        cr10_urdf)

    # --- COVVI hand (raw URDF) ---
    hand_urdf_path = os.path.join(
        hand_pack_share, 'urdf', 'linear_covvi_hand_gazebo.urdf')
    with open(hand_urdf_path) as f:
        hand_urdf = f.read()

    hand_body = re.search(
        r'<robot[^>]*>(.*)</robot>', hand_urdf, re.DOTALL).group(1)

    # Remove standalone world / base_footprint links
    hand_body = re.sub(r'<link\s+name="world"\s*/>\s*', '', hand_body)
    hand_body = re.sub(r'<link\s+name="base_footprint"\s*/>\s*', '', hand_body)

    # Remove world_fixed and base_joint joints
    hand_body = re.sub(
        r'<joint\s+name="world_fixed"[^>]*>.*?</joint>', '', hand_body, flags=re.DOTALL)
    hand_body = re.sub(
        r'<joint\s+name="base_joint"[^>]*>.*?</joint>', '', hand_body, flags=re.DOTALL)

    # Rename base_link → hand_base_link
    hand_body = hand_body.replace('"base_link"', '"hand_base_link"')

    # Remove hand's gazebo_ros2_control plugin (CR10's plugin handles both)
    hand_body = re.sub(
        r'<gazebo>\s*<plugin[^>]*gazebo_ros2_control[^>]*>.*?</plugin>\s*</gazebo>',
        '', hand_body, flags=re.DOTALL)

    # Rename hand's ros2_control block to avoid name collision with CR10's
    hand_body = hand_body.replace(
        '<ros2_control name="GazeboSystem"',
        '<ros2_control name="HandGazeboSystem"')

    coupling_joint = """
  <joint name="hand_coupling" type="fixed">
    <parent link="Link6"/>
    <child link="hand_base_link"/>
    <origin xyz="0 0 0" rpy="1.5708 0 0"/>
  </joint>
"""
    combined = cr10_urdf.replace(
        '</robot>', hand_body + coupling_joint + '</robot>')
    # RCL's YAML parser rejects multi-line plain scalars, so the combined URDF
    # must be a single line when gazebo_ros2_control passes it via --param.
    combined = combined.replace('\n', ' ').replace('\r', ' ')
    return combined


def generate_launch_description():
    hand_pack_share = get_package_share_directory('hand_pack')
    combined_urdf = _build_gazebo_urdf()
    world_file = os.path.join(hand_pack_share, 'worlds', 'factory.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_file}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': combined_urdf, 'use_sim_time': True}],
    )

    # Spawn on top of the robot table (z = 0.85 m)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'cr10_covvi',
                   '-z', '0.85'],
        output='screen',
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    load_cr10_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['cr10_group_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    load_hand_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hand_position_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    combined_gui = Node(
        package='hand_pack',
        executable='combined_gui',
        output='screen',
    )

    after_spawn_load_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    after_broadcaster_load_cr10 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_cr10_controller],
        )
    )

    after_cr10_load_hand = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_cr10_controller,
            on_exit=[load_hand_controller],
        )
    )

    after_hand_launch_gui = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_hand_controller,
            on_exit=[combined_gui],
        )
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        after_spawn_load_broadcaster,
        after_broadcaster_load_cr10,
        after_cr10_load_hand,
        after_hand_launch_gui,
    ])
