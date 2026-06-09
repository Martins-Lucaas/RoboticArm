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
    touch_pack_share = get_package_share_directory('touch_pack')
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

    # Gazebo Classic does not resolve package:// URIs when spawning from file.
    # Convert all hand mesh paths to absolute file:// URIs so meshes are visible.
    hand_urdf = hand_urdf.replace(
        'package://hand_pack', f'file://{hand_pack_share}')

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

    # Disable gravity and self-collision for every hand link so the physics
    # solver never accumulates forces between control cycles.
    hand_link_names = re.findall(r'<link\s+name="([^"]+)"', hand_body)
    for lname in hand_link_names:
        hand_body += (
            f'\n  <gazebo reference="{lname}">'
            f'<gravity>false</gravity>'
            f'<self_collide>false</self_collide>'
            f'</gazebo>'
        )

    # Acoplador da prótese (PecasProtese.stl) entre Link6 e a mão.
    # Disco ⌀75×55.46 mm: fundo do mesh assenta no flange (Link6) e a mão
    # COVVI monta no topo (+0.05546 m ao longo de +Link6_z). Rx(+90°) mantém
    # a mão estendendo-se axialmente ao pulso, agora deslocada pela altura
    # do acoplador. Gazebo Classic não resolve package:// → usa file://.
    coupler_mesh = os.path.join(
        touch_pack_share, 'meshes', 'PecasProtese.stl')
    coupling_joint = f"""
  <link name="hand_coupler_link">
    <inertial>
      <origin xyz="0 0 0.02773" rpy="0 0 0"/>
      <mass value="0.150"/>
      <inertia ixx="9.12e-5" ixy="0.0" ixz="0.0"
               iyy="9.12e-5" iyz="0.0" izz="1.055e-4"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="file://{coupler_mesh}" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="coupler_black">
        <color rgba="0.03 0.03 0.03 1.0"/>
      </material>
    </visual>
    <collision name="col_hand_coupler">
      <origin xyz="0 0 0.02773" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.0375" length="0.05546"/>
      </geometry>
    </collision>
  </link>

  <joint name="coupler_attach" type="fixed">
    <parent link="Link6"/>
    <child link="hand_coupler_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <joint name="hand_coupling" type="fixed">
    <parent link="hand_coupler_link"/>
    <child link="hand_base_link"/>
    <origin xyz="0 0 0.05546" rpy="1.5708 0 0"/>
  </joint>

  <gazebo reference="hand_coupler_link">
    <gravity>false</gravity>
    <self_collide>false</self_collide>
    <visual>
      <material>
        <ambient>0.02 0.02 0.02 1</ambient>
        <diffuse>0.03 0.03 0.03 1</diffuse>
        <specular>0.10 0.10 0.10 1</specular>
      </material>
    </visual>
  </gazebo>
"""
    combined = cr10_urdf.replace(
        '</robot>', hand_body + coupling_joint + '</robot>')

    # === Build minimal URDF for robot_state_publisher ===
    # gazebo_ros2_control fetches robot_description from the RSP parameter service.
    # RSP only declares robot_description explicitly, so any other parameter name
    # fails the GET_PARAMETERS service call and crashes the plugin (SIGSEGV).
    # Fix: RSP publishes the minimal URDF as robot_description (kinematic structure
    # is identical — only meshes/inertials are stripped); the full URDF is written
    # to a file so Gazebo spawn still loads the correct visuals and collisions.
    minimal = combined
    minimal = re.sub(r'<visual\b[^>]*>.*?</visual>', '', minimal, flags=re.DOTALL)
    minimal = re.sub(r'<collision\b[^>]*>.*?</collision>', '', minimal, flags=re.DOTALL)
    minimal = re.sub(r'<inertial\b[^>]*>.*?</inertial>', '', minimal, flags=re.DOTALL)
    minimal = re.sub(
        r'<gazebo\s+reference\s*=\s*"[^"]*"\s*>.*?</gazebo>', '', minimal, flags=re.DOTALL)
    minimal = re.sub(r'<!--.*?-->', '', minimal, flags=re.DOTALL)
    minimal = re.sub(r'<\?xml[^?]*\?>', '', minimal)
    minimal = ' '.join(minimal.split())

    return combined, minimal


def generate_launch_description():
    hand_pack_share = get_package_share_directory('hand_pack')
    combined_urdf, minimal_urdf = _build_gazebo_urdf()
    world_file = os.path.join(hand_pack_share, 'worlds', 'factory.world')

    urdf_spawn_path = '/tmp/cr10_covvi_spawn.urdf'
    with open(urdf_spawn_path, 'w') as _f:
        _f.write(combined_urdf)

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
        parameters=[{
            'robot_description': minimal_urdf,
            'use_sim_time': True,
        }],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', urdf_spawn_path,
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
