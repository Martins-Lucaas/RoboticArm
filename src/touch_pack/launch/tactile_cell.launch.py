"""
tactile_cell.launch.py — Launcher principal do touch_pack.

Inicia, em ordem, AUTOMATICAMENTE:

    1. Gazebo Classic com `research_lab.world`
    2. robot_state_publisher com URDF combinado (CR10 + COVVI Hand + alias
       `tcp_link` para o admittance / IK do explorer)
    3. spawn_entity do robô em z = 0.75 (topo do tampo da mesa)
    4. Controllers (joint_state_broadcaster → cr10_group → hand_position)
    5. `tactile_explorer` (backend FSM 4 fases)
    6. `palpation_gui` (Tkinter — janela abre sozinha)

Argumentos (todos opcionais):
    control_mode     sim_only | mirror | real_from_sim (default sim_only)
    robot_ip         IP do controlador CR10 real (default 192.168.5.1)
    robot_dry_run    true = sockets não abertos (default true)
    no_gui           true = não abrir palpation_gui (default false)

Exemplo:
    ros2 launch touch_pack tactile_cell.launch.py
    ros2 launch touch_pack tactile_cell.launch.py no_gui:=true
"""
import os
import re
import xacro

from ament_index_python.packages import get_package_share_directory
from hand_pack.urdf_helpers import (
    clamp_hand_joint_limits,
    inject_visual_skin_layer,
    INTER_FINGER_COLLISION_LINKS,
)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, RegisterEventHandler,
                             IncludeLaunchDescription)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ──────────────────────────────────────────────────────────────────────
# Helpers de saneamento do URDF combinado (mesma lógica do launch antigo,
# mantida para preservar atrito/skin/inertia tunados para o COVVI no
# Gazebo).
# ──────────────────────────────────────────────────────────────────────
def _fix_virtual_link_inertia(urdf_body: str) -> str:
    phantom = (
        r'<inertial>\s*<mass value="1"\s*/>\s*'
        r'<inertia ixx="1\.0" ixy="0\.0" ixz="0\.0" iyy="1\.0" iyz="0\.0" izz="1\.0"\s*/>'
        r'\s*</inertial>'
    )
    minimal = (
        '<inertial><mass value="0.001"/>'
        '<inertia ixx="1e-9" ixy="0.0" ixz="0.0" iyy="1e-9" iyz="0.0" izz="1e-9"/>'
        '</inertial>'
    )
    return re.sub(phantom, minimal, urdf_body, flags=re.DOTALL)


def _stabilize_hand_joints(urdf_body: str) -> str:
    def _patch(m: re.Match) -> str:
        jxml = m.group(0)
        if 'type="revolute"' not in jxml:
            return jxml
        is_mimic = '<mimic' in jxml
        damp, fric = (30.0, 10.0) if is_mimic else (5.0, 1.0)
        dyn = f'<dynamics damping="{damp}" friction="{fric}"/>'
        if '<dynamics' in jxml:
            jxml = re.sub(r'<dynamics[^/]*/>', dyn, jxml)
        else:
            jxml = jxml.replace('</joint>', f'      {dyn}\n    </joint>')
        if not is_mimic:
            jxml = re.sub(r'effort="[\d.]+"', 'effort="8.0"', jxml)
        return jxml
    return re.sub(r'<joint\b[^>]*>.*?</joint>', _patch,
                   urdf_body, flags=re.DOTALL)


def _build_robot_urdf():
    """Compõe o URDF CR10 + mão COVVI + alias `tcp_link` para o explorer.

    `tcp_link` é uma junta fixa ancorada na ponta do dedo indicador
    (index_distal). É o frame onde o IK do tactile_explorer mira durante
    CONTACT/SLIDING/RETRACT (e onde o admittance_controller, quando
    ativado, calcula a admitância).
    """
    hand_pack_share = get_package_share_directory('hand_pack')
    cra_share = get_package_share_directory('cra_description')

    # YAML dos controladores (mesmo da hand_pack — já contém os 31 joints
    # da mão + as 6 do CR10). O tactile_controllers.yaml fica como
    # referência para futura ativação do admittance_controller.
    controllers_yaml = os.path.join(
        hand_pack_share, 'config', 'cr10_covvi_controllers.yaml')

    # CR10 (xacro) ────────────────────────────────────────────────────
    cr10_xacro_path = os.path.join(cra_share, 'urdf', 'cr10_robot.xacro')
    doc = xacro.parse(open(cr10_xacro_path))
    xacro.process_doc(doc)
    cr10_urdf = doc.toxml()
    cr10_urdf = re.sub(
        r'<parameters>[^<]*/ros2_controllers\.yaml</parameters>',
        f'<parameters>{controllers_yaml}</parameters>',
        cr10_urdf)

    # Mão COVVI (URDF estático) ───────────────────────────────────────
    hand_urdf_path = os.path.join(
        hand_pack_share, 'urdf', 'linear_covvi_hand_gazebo.urdf')
    with open(hand_urdf_path) as f:
        hand_urdf = f.read()
    hand_urdf = hand_urdf.replace(
        'package://hand_pack', f'file://{hand_pack_share}')
    hand_body = re.search(
        r'<robot[^>]*>(.*)</robot>', hand_urdf, re.DOTALL).group(1)
    hand_body = re.sub(r'<link\s+name="world"\s*/>\s*', '', hand_body)
    hand_body = re.sub(r'<link\s+name="base_footprint"\s*/>\s*', '', hand_body)
    hand_body = re.sub(
        r'<joint\s+name="world_fixed"[^>]*>.*?</joint>', '',
        hand_body, flags=re.DOTALL)
    hand_body = re.sub(
        r'<joint\s+name="base_joint"[^>]*>.*?</joint>', '',
        hand_body, flags=re.DOTALL)
    hand_body = hand_body.replace('"base_link"', '"hand_base_link"')
    hand_body = re.sub(
        r'<gazebo>\s*<plugin[^>]*gazebo_ros2_control[^>]*>.*?</plugin>\s*</gazebo>',
        '', hand_body, flags=re.DOTALL)
    hand_body = hand_body.replace(
        '<ros2_control name="GazeboSystem"',
        '<ros2_control name="HandGazeboSystem"')

    hand_body = _fix_virtual_link_inertia(hand_body)
    hand_body = clamp_hand_joint_limits(hand_body)
    hand_body = _stabilize_hand_joints(hand_body)
    hand_body = inject_visual_skin_layer(hand_body)

    # gazebo tags por link da mão (contato suave + atrito alto na pele).
    hand_link_names = re.findall(r'<link\s+name="([^"]+)"', hand_body)
    fc = set(INTER_FINGER_COLLISION_LINKS)
    for lname in hand_link_names:
        is_grip = lname in fc
        sc = 'true' if is_grip else 'false'
        mu = '2.5' if is_grip else '0.8'
        hand_body += (
            f'\n  <gazebo reference="{lname}">'
            f'<gravity>false</gravity>'
            f'<self_collide>{sc}</self_collide>'
            f'<mu1>{mu}</mu1><mu2>{mu}</mu2>'
            f'<kp>5e4</kp><kd>50.0</kd>'
            f'<maxContacts>8</maxContacts>'
            f'<minDepth>0.0005</minDepth>'
            f'<maxVel>0.01</maxVel>'
            f'</gazebo>'
        )

    # Acoplamento da mão no flange Link6.
    attach_joint = '''
    <joint name="hand_attach_joint" type="fixed">
      <parent link="Link6"/>
      <child link="hand_base_link"/>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    </joint>'''

    # Alias tcp_link colado na ponta do dedo Index — o tactile_explorer
    # mira a IK aqui. Posição: 22 mm acima do index_distal (ponta da
    # falange distal do Index segundo o mesh right_index_distal.STL).
    tcp_alias = '''
    <link name="tcp_link"/>
    <joint name="tcp_alias_joint" type="fixed">
      <parent link="index_distal"/>
      <child link="tcp_link"/>
      <origin xyz="0 0 0.022" rpy="0 0 0"/>
    </joint>'''

    full_urdf = cr10_urdf.replace(
        '</robot>', hand_body + attach_joint + tcp_alias + '</robot>')

    # Gazebo refs para cada link do CR10 (sem auto-colisão).
    arm_links = re.findall(r'<link\s+name="([^"]+)"', cr10_urdf)
    arm_gz = ''.join(
        f'\n  <gazebo reference="{n}"><self_collide>false</self_collide></gazebo>'
        for n in arm_links)
    full_urdf = full_urdf.replace('</robot>', arm_gz + '\n</robot>')

    # URDF mínimo para o robot_state_publisher (sem visual/collision/
    # inertial → reduz overhead de tópico).
    minimal = full_urdf
    minimal = re.sub(r'<visual\b[^>]*>.*?</visual>', '',
                      minimal, flags=re.DOTALL)
    minimal = re.sub(r'<collision\b[^>]*>.*?</collision>', '',
                      minimal, flags=re.DOTALL)
    minimal = re.sub(r'<inertial\b[^>]*>.*?</inertial>', '',
                      minimal, flags=re.DOTALL)
    minimal = re.sub(
        r'<gazebo\s+reference\s*=\s*"[^"]*"\s*>.*?</gazebo>', '',
        minimal, flags=re.DOTALL)
    minimal = re.sub(r'<!--.*?-->', '', minimal, flags=re.DOTALL)
    minimal = re.sub(r'<\?xml[^?]*\?>', '', minimal)
    minimal = ' '.join(minimal.split())

    return full_urdf, minimal


def generate_launch_description():
    pkg_touch = get_package_share_directory('touch_pack')
    pkg_gazebo = get_package_share_directory('gazebo_ros')

    # ── Argumentos do launch ────────────────────────────────────────
    # Mesa única (research_lab.world): tampo em z=0.75 m. O robô apoia
    # nesse tampo (robot_spawn_z = 0.75 → base_link do CR10 em z=0.78).
    mode_arg      = DeclareLaunchArgument(
        'control_mode', default_value='sim_only',
        description='sim_only | mirror | real_from_sim')
    robot_ip_arg  = DeclareLaunchArgument(
        'robot_ip', default_value='192.168.5.1')
    robot_dry_arg = DeclareLaunchArgument(
        'robot_dry_run', default_value='true')
    no_gui_arg    = DeclareLaunchArgument('no_gui', default_value='false')

    control_mode = LaunchConfiguration('control_mode')
    robot_ip     = LaunchConfiguration('robot_ip')
    robot_dry    = LaunchConfiguration('robot_dry_run')
    no_gui       = LaunchConfiguration('no_gui')

    # ── URDFs ─────────────────────────────────────────────────────────
    full_urdf, minimal_urdf = _build_robot_urdf()
    urdf_spawn_path = '/tmp/tactile_cell_robot.urdf'
    with open(urdf_spawn_path, 'w') as f:
        f.write(full_urdf)

    world_file = os.path.join(pkg_touch, 'worlds', 'research_lab.world')

    # ── Gazebo ────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items())

    # ── Robot State Publisher ─────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': minimal_urdf,
                     'use_sim_time': True}])

    # ── Spawn do robô CR10+COVVI ──────────────────────────────────────
    # Apoiado no tampo da mesa (z=0.75). base_link cai em z=0.78.
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', urdf_spawn_path, '-entity', 'cr10_covvi',
                   '-x', '0', '-y', '0', '-z', '0.75'],
        parameters=[{'use_sim_time': True}])

    # ── Controladores ros2_control (cadeia de dependência) ───────────
    load_jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'])
    load_arm = Node(
        package='controller_manager', executable='spawner',
        arguments=['cr10_group_controller',
                   '--controller-manager', '/controller_manager'])
    load_hand = Node(
        package='controller_manager', executable='spawner',
        arguments=['hand_position_controller',
                   '--controller-manager', '/controller_manager'])

    after_spawn_load_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_robot,
                                     on_exit=[load_jsb]))
    after_jsb_load_arm = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=load_jsb,
                                     on_exit=[load_arm]))
    after_arm_load_hand = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=load_arm,
                                     on_exit=[load_hand]))

    # ── Backend + GUI da palpação tátil ──────────────────────────────
    # tactile_explorer aceita o param `controller_action` para apontar
    # ao admittance_controller quando este estiver ativo; por default
    # usa o `cr10_group_controller` (joint_trajectory).
    explorer_node = Node(
        package='touch_pack', executable='tactile_explorer',
        parameters=[{
            # Altura da base_link do CR10 em coordenadas do mundo
            # (spawn_z=0.75 + dummy_joint_z=0.03 = 0.78 m).
            'arm_base_z':         0.78,
            'controller_action':  '/cr10_group_controller/follow_joint_trajectory',
            'use_sim_time':       True,
        }])

    gui_node = Node(
        package='touch_pack', executable='palpation_gui',
        condition=UnlessCondition(no_gui))

    # explorer + GUI sobem quando o último controller (hand) terminar
    # de carregar, garantindo que /joint_states e a action do braço já
    # estejam disponíveis.
    after_hand_start = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_hand,
            on_exit=[explorer_node, gui_node]))

    return LaunchDescription([
        mode_arg, robot_ip_arg, robot_dry_arg,
        no_gui_arg,

        gazebo,
        rsp,
        spawn_robot,
        after_spawn_load_jsb,
        after_jsb_load_arm,
        after_arm_load_hand,
        after_hand_start,
    ])
