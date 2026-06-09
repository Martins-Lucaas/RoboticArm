"""
tactile_cell.launch.py — Launcher principal do touch_pack.

Argumentos (todos opcionais):
    end_effector     hand | touch_tool  (default: hand)
                       hand       → CR10 + mão COVVI; tcp_link = ponta do Index
                       touch_tool → CR10 + TouchTool Square 20×20 mm; tcp_link = ponta do probe
    control_mode     sim_only | mirror | real_from_sim (default sim_only)
    robot_ip         IP do controlador CR10 real (default 192.168.5.2)
    robot_dry_run    true = sockets não abertos (default true)
    no_gui           true = não abrir palpation_gui (default false)

Exemplos:
    ros2 launch touch_pack tactile_cell.launch.py
    ros2 launch touch_pack tactile_cell.launch.py end_effector:=touch_tool
    ros2 launch touch_pack tactile_cell.launch.py end_effector:=touch_tool no_gui:=true
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
from launch.actions import (DeclareLaunchArgument, OpaqueFunction,
                             RegisterEventHandler, IncludeLaunchDescription)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ──────────────────────────────────────────────────────────────────────
# Materiais Gazebo (override de cor por link) — aplicados na conversão
# URDF→SDF do spawn_entity. Em Gazebo Classic a cor do <material><color>
# do URDF nem sempre renderiza; o override <gazebo reference><visual>
# <material><ambient/diffuse/specular> é o que efetivamente colore o link.
#   touch_tool   → branco
#   acopladores  → cinza
#   célula carga → prateado (specular alto = brilho metálico)
# ──────────────────────────────────────────────────────────────────────
_GZ_MAT_WHITE = (
    '<visual><material>'
    '<ambient>0.85 0.85 0.85 1</ambient>'
    '<diffuse>0.95 0.95 0.95 1</diffuse>'
    '<specular>0.30 0.30 0.30 1</specular>'
    '</material></visual>'
)
_GZ_MAT_GRAY = (
    '<visual><material>'
    '<ambient>0.25 0.25 0.25 1</ambient>'
    '<diffuse>0.45 0.45 0.45 1</diffuse>'
    '<specular>0.20 0.20 0.20 1</specular>'
    '</material></visual>'
)
_GZ_MAT_SILVER = (
    '<visual><material>'
    '<ambient>0.55 0.55 0.58 1</ambient>'
    '<diffuse>0.82 0.82 0.88 1</diffuse>'
    '<specular>0.95 0.95 1.00 1</specular>'
    '</material></visual>'
)


# ──────────────────────────────────────────────────────────────────────
# Helpers de saneamento do URDF combinado
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


# ──────────────────────────────────────────────────────────────────────
# Construção do URDF combinado (roteado por end_effector)
# ──────────────────────────────────────────────────────────────────────
def _build_robot_urdf(end_effector: str):
    hand_pack_share  = get_package_share_directory('hand_pack')
    cra_share        = get_package_share_directory('cra_description')
    touch_pack_share = get_package_share_directory('touch_pack')

    # ── YAML de controllers ───────────────────────────────────────────
    if end_effector == 'hand':
        controllers_yaml = os.path.join(
            hand_pack_share, 'config', 'cr10_covvi_controllers.yaml')
    else:  # touch_tool
        controllers_yaml = os.path.join(
            touch_pack_share, 'config', 'tactile_controllers.yaml')

    # ── CR10 (xacro) ─────────────────────────────────────────────────
    cr10_xacro_path = os.path.join(cra_share, 'urdf', 'cr10_robot.xacro')
    doc = xacro.parse(open(cr10_xacro_path))
    xacro.process_doc(doc)
    cr10_urdf = doc.toxml()
    cr10_urdf = re.sub(
        r'<parameters>[^<]*/ros2_controllers\.yaml</parameters>',
        f'<parameters>{controllers_yaml}</parameters>',
        cr10_urdf)

    # ── Fim do URDF: links/juntas do efector + Gazebo refs ────────────
    # Unifica selfCollide no xacro: substitui true→false antes de adicionar
    # arm_gz, evitando "multiple inconsistent <self_collide>" do parser_urdf.
    cr10_urdf = cr10_urdf.replace('<selfCollide>true</selfCollide>',
                                   '<selfCollide>false</selfCollide>')
    arm_links = re.findall(r'<link\s+name="([^"]+)"', cr10_urdf)
    # Só adiciona self_collide para links sem <gazebo reference="..."> no xacro.
    existing_arm_gz = set(re.findall(r'<gazebo\s+reference="([^"]+)"', cr10_urdf))
    arm_gz = ''.join(
        f'\n  <gazebo reference="{n}"><self_collide>false</self_collide></gazebo>'
        for n in arm_links if n not in existing_arm_gz)

    if end_effector == 'hand':
        full_urdf = _build_hand_suffix(
            cr10_urdf, hand_pack_share, arm_gz, touch_pack_share)
    else:
        full_urdf = _build_touch_tool_suffix(cr10_urdf, touch_pack_share, arm_gz)

    # ── URDF mínimo para o robot_state_publisher ──────────────────────
    minimal = full_urdf
    minimal = re.sub(r'<visual\b[^>]*>.*?</visual>', '', minimal, flags=re.DOTALL)
    minimal = re.sub(r'<collision\b[^>]*>.*?</collision>', '', minimal, flags=re.DOTALL)
    minimal = re.sub(r'<inertial\b[^>]*>.*?</inertial>', '', minimal, flags=re.DOTALL)
    minimal = re.sub(
        r'<gazebo\s+reference\s*=\s*"[^"]*"\s*>.*?</gazebo>', '',
        minimal, flags=re.DOTALL)
    minimal = re.sub(r'<!--.*?-->', '', minimal, flags=re.DOTALL)
    minimal = re.sub(r'<\?xml[^?]*\?>', '', minimal)
    minimal = ' '.join(minimal.split())

    return full_urdf, minimal


def _build_hand_suffix(cr10_urdf: str, hand_pack_share: str, arm_gz: str,
                       touch_pack_share: str) -> str:
    """Injeta a mão COVVI + tcp_link (Index distal) no CR10."""
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
    # Remove <gazebo reference="..."> estáticos com propriedades de física (mu1,
    # kd, etc.) para evitar "multiple inconsistent" do parser_urdf ao reduzir
    # fixed joints: o loop abaixo adiciona valores canônicos para todos os links.
    hand_body = re.sub(
        r'<gazebo\s+reference="[^"]+">(?:(?!</gazebo>).)*?<mu1>(?:(?!</gazebo>).)*?</gazebo>\s*',
        '', hand_body, flags=re.DOTALL)
    hand_body = inject_visual_skin_layer(hand_body)

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

    # Acoplador da prótese (PecasProtese.stl) entre Link6 e a mão.
    # Disco ⌀75×55.46 mm: fundo do mesh assenta no flange (Link6) e a mão
    # COVVI monta no topo (+0.05546 m ao longo de +Link6_z). Rx(+90°) mantém
    # a mão estendendo-se axialmente ao pulso, agora deslocada pela altura
    # do acoplador. Gazebo Classic não resolve package:// → usa file://.
    coupler_mesh = os.path.join(
        touch_pack_share, 'meshes', 'PecasProtese.stl')
    attach_joint = f'''
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

    <joint name="hand_attach_joint" type="fixed">
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
    </gazebo>'''

    tcp_alias = '''
    <link name="tcp_link"/>
    <joint name="tcp_alias_joint" type="fixed">
      <parent link="index_distal"/>
      <child link="tcp_link"/>
      <origin xyz="0 0 0.022" rpy="0 0 0"/>
    </joint>'''

    full_urdf = cr10_urdf.replace(
        '</robot>', hand_body + attach_joint + tcp_alias + '</robot>')
    full_urdf = full_urdf.replace('</robot>', arm_gz + '\n</robot>')
    return full_urdf


def _build_touch_tool_suffix(cr10_urdf: str, touch_pack_share: str,
                              arm_gz: str) -> str:
    """Injeta acopladores + Célula de Carga + TouchTool Square 20×20 mm + tcp_link no CR10.

    Cadeia (offsets desde Link6):
      lower_coupling (+0mm) → force_sensor (+7mm, entra 8mm no acoplador inferior)
      → upper_coupling (+59mm, entra 8mm no topo da célula)
      → touch_tool (+74mm) → tcp_link (+188.5mm)
    """
    sensor_mesh   = os.path.join(touch_pack_share, 'meshes', 'Celula De Carga.stl')
    tool_mesh     = os.path.join(touch_pack_share, 'meshes', 'touch_tool_square_20x20.stl')
    coupling_mesh = os.path.join(touch_pack_share, 'meshes', 'Acoplador_CelulaDeCarga_Uniaxial.stl')

    tool_snippet = f'''
    <!-- ── Acoplador inferior (Link6 → base da célula) ───────────── -->
    <link name="lower_coupling_link">
      <inertial>
        <origin xyz="0 0 0.0075" rpy="0 0 0"/>
        <mass value="0.200"/>
        <inertia ixx="8.4e-5" ixy="0.0" ixz="0.0"
                 iyy="8.4e-5" iyz="0.0" izz="1.6e-4"/>
      </inertial>
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <mesh filename="file://{coupling_mesh}" scale="0.001 0.001 0.001"/>
        </geometry>
        <material name="coupling_gray">
          <color rgba="0.45 0.45 0.45 1.0"/>
        </material>
      </visual>
      <collision name="col_lower_coupling">
        <origin xyz="0 0 0.0075" rpy="0 0 0"/>
        <geometry><cylinder radius="0.040" length="0.015"/></geometry>
      </collision>
    </link>
    <!-- ── Célula de Carga (em pé: Rx+90°, mesh-Y→+Z robô) ──────── -->
    <link name="force_sensor_link">
      <inertial>
        <origin xyz="0 0 0.030" rpy="0 0 0"/>
        <mass value="0.150"/>
        <inertia ixx="4.70e-5" ixy="0.0" ixz="0.0"
                 iyy="7.73e-5" iyz="0.0" izz="3.43e-5"/>
      </inertial>
      <visual>
        <origin xyz="-0.0254 0.00635 0" rpy="1.5708 0 0"/>
        <geometry>
          <mesh filename="file://{sensor_mesh}" scale="0.001 0.001 0.001"/>
        </geometry>
        <material name="silver">
          <color rgba="0.82 0.82 0.88 1.0"/>
        </material>
      </visual>
      <collision name="col_sensor">
        <origin xyz="0 0 0.030" rpy="0 0 0"/>
        <geometry><box size="0.0508 0.0127 0.060"/></geometry>
      </collision>
    </link>
    <!-- ── Acoplador superior (topo da célula → touch_tool) ─────── -->
    <link name="upper_coupling_link">
      <inertial>
        <origin xyz="0 0 0.0075" rpy="0 0 0"/>
        <mass value="0.200"/>
        <inertia ixx="8.4e-5" ixy="0.0" ixz="0.0"
                 iyy="8.4e-5" iyz="0.0" izz="1.6e-4"/>
      </inertial>
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <mesh filename="file://{coupling_mesh}" scale="0.001 0.001 0.001"/>
        </geometry>
        <material name="coupling_gray">
          <color rgba="0.45 0.45 0.45 1.0"/>
        </material>
      </visual>
      <collision name="col_upper_coupling">
        <origin xyz="0 0 0.0075" rpy="0 0 0"/>
        <geometry><cylinder radius="0.040" length="0.015"/></geometry>
      </collision>
    </link>
    <!-- ── Touch Tool ─────────────────────────────────────────────── -->
    <link name="touch_tool_link">
      <inertial>
        <origin xyz="0 0 0.064" rpy="0 0 0"/>
        <mass value="0.150"/>
        <inertia ixx="2.65e-4" ixy="0.0" ixz="0.0"
                 iyy="2.65e-4" iyz="0.0" izz="2.03e-4"/>
      </inertial>
      <visual>
        <origin xyz="0 0 0.0065" rpy="0 0 0"/>
        <geometry>
          <mesh filename="file://{tool_mesh}" scale="0.001 0.001 0.001"/>
        </geometry>
        <material name="tool_white">
          <color rgba="0.95 0.95 0.95 1.0"/>
        </material>
      </visual>
      <collision name="col_body">
        <origin xyz="0 0 0.047" rpy="0 0 0"/>
        <geometry><box size="0.090 0.090 0.094"/></geometry>
      </collision>
      <collision name="col_tip">
        <origin xyz="0 0 0.106" rpy="0 0 0"/>
        <geometry><box size="0.025 0.025 0.038"/></geometry>
      </collision>
    </link>
    <link name="tcp_link"/>
    <!-- acoplador inferior no flange -->
    <joint name="lower_coupling_attach" type="fixed">
      <parent link="Link6"/>
      <child link="lower_coupling_link"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </joint>
    <!-- célula inicia +7 mm: entra 8 mm dentro do acoplador inferior -->
    <joint name="force_sensor_attach" type="fixed">
      <parent link="lower_coupling_link"/>
      <child link="force_sensor_link"/>
      <origin xyz="0 0 0.007" rpy="0 0 0"/>
    </joint>
    <!-- acoplador superior começa 8 mm antes do topo da célula (+52 mm) -->
    <joint name="upper_coupling_attach" type="fixed">
      <parent link="force_sensor_link"/>
      <child link="upper_coupling_link"/>
      <origin xyz="0 0 0.052" rpy="0 0 0"/>
    </joint>
    <!-- touch tool sobre o acoplador superior (+15 mm) -->
    <joint name="touch_tool_attach" type="fixed">
      <parent link="upper_coupling_link"/>
      <child link="touch_tool_link"/>
      <origin xyz="0 0 0.015" rpy="0 0 0"/>
    </joint>
    <joint name="tcp_alias_joint" type="fixed">
      <parent link="touch_tool_link"/>
      <child link="tcp_link"/>
      <origin xyz="0 0 0.1145" rpy="0 0 0"/>
    </joint>'''

    tool_gz = (
        '\n  <gazebo reference="lower_coupling_link">'
        '<self_collide>false</self_collide>'
        '<gravity>true</gravity>'
        '<mu1>0.40</mu1><mu2>0.40</mu2>'
        '<kp>1.0e5</kp><kd>100.0</kd>'
        '<maxContacts>4</maxContacts>'
        '<minDepth>0.0002</minDepth>'
        '<maxVel>0.05</maxVel>'
        + _GZ_MAT_GRAY +
        '</gazebo>'
        '\n  <gazebo reference="force_sensor_link">'
        '<self_collide>false</self_collide>'
        '<gravity>true</gravity>'
        '<mu1>0.40</mu1><mu2>0.40</mu2>'
        '<kp>1.0e5</kp><kd>100.0</kd>'
        '<maxContacts>4</maxContacts>'
        '<minDepth>0.0002</minDepth>'
        '<maxVel>0.05</maxVel>'
        + _GZ_MAT_SILVER +
        '</gazebo>'
        '\n  <gazebo reference="upper_coupling_link">'
        '<self_collide>false</self_collide>'
        '<gravity>true</gravity>'
        '<mu1>0.40</mu1><mu2>0.40</mu2>'
        '<kp>1.0e5</kp><kd>100.0</kd>'
        '<maxContacts>4</maxContacts>'
        '<minDepth>0.0002</minDepth>'
        '<maxVel>0.05</maxVel>'
        + _GZ_MAT_GRAY +
        '</gazebo>'
        '\n  <gazebo reference="touch_tool_link">'
        '<self_collide>false</self_collide>'
        '<gravity>true</gravity>'
        '<mu1>0.60</mu1><mu2>0.60</mu2>'
        '<kp>1.0e5</kp><kd>100.0</kd>'
        '<maxContacts>4</maxContacts>'
        '<minDepth>0.0002</minDepth>'
        '<maxVel>0.05</maxVel>'
        + _GZ_MAT_WHITE +
        '</gazebo>'
    )

    full_urdf = cr10_urdf.replace('</robot>', tool_snippet + '</robot>')
    full_urdf = full_urdf.replace('</robot>', arm_gz + tool_gz + '\n</robot>')
    return full_urdf


# ──────────────────────────────────────────────────────────────────────
# OpaqueFunction: monta nodes/handlers após resolver os argumentos
# ──────────────────────────────────────────────────────────────────────
_CONTROL_MODE_MAP = {
    'sim_only':      'SIM_ONLY',
    'mirror':        'MIRROR',
    'real_from_sim': 'REAL_FROM_SIM',
}


def launch_setup(context, *args, **kwargs):
    end_effector = LaunchConfiguration('end_effector').perform(context)
    control_mode = LaunchConfiguration('control_mode').perform(context)
    robot_ip     = LaunchConfiguration('robot_ip').perform(context)
    robot_dry    = LaunchConfiguration('robot_dry_run').perform(context)
    no_gui_val   = LaunchConfiguration('no_gui').perform(context)

    robot_mode = _CONTROL_MODE_MAP.get(control_mode, 'SIM_ONLY')

    pkg_touch  = get_package_share_directory('touch_pack')
    pkg_gazebo = get_package_share_directory('gazebo_ros')

    # ── URDFs ─────────────────────────────────────────────────────────
    full_urdf, minimal_urdf = _build_robot_urdf(end_effector)
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

    # ── Spawn do robô ─────────────────────────────────────────────────
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', urdf_spawn_path, '-entity', 'cr10_tcp',
                   '-x', '0.30', '-y', '0', '-z', '0.75'],
        parameters=[{'use_sim_time': True}])

    # ── Home setter ───────────────────────────────────────────────────
    home_setter = Node(
        package='touch_pack', executable='gazebo_home_setter',
        parameters=[{'use_sim_time': True, 'robot_ip': robot_ip,
                     'end_effector': end_effector}])

    # ── Controllers ───────────────────────────────────────────────────
    load_jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'])
    load_arm = Node(
        package='controller_manager', executable='spawner',
        arguments=['cr10_group_controller',
                   '--controller-manager', '/controller_manager'])

    # ── Aplicação (explorer + GUI + logger + force_rx) ────────────────
    explorer_node = Node(
        package='touch_pack', executable='tactile_explorer',
        parameters=[{
            'arm_base_z':        0.78,
            'controller_action': '/cr10_group_controller/follow_joint_trajectory',
            'use_sim_time':      True,
        }])

    gui_node = Node(
        package='touch_pack', executable='palpation_gui',
        parameters=[{'use_sim_time': True,
                     'robot_ip':     robot_ip,
                     'robot_mode':   robot_mode,
                     # Gate do modo Palpação: só liberado com end_effector=touch_tool.
                     'end_effector': end_effector}],
        condition=UnlessCondition(LaunchConfiguration('no_gui')))

    logger_node = Node(
        package='touch_pack', executable='palpation_logger')

    force_rx_node = Node(
        package='touch_pack', executable='force_receiver')

    # Nós que não dependem de controllers — sobem logo após home_setter.
    early_nodes = [gui_node, logger_node, force_rx_node]
    # Explorer precisa da action do cr10_group_controller — sobe por último.
    late_nodes  = [explorer_node]

    # ── Cadeia de dependências: varia com end_effector ────────────────
    after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[home_setter]))
    # GUI, logger e force_rx sobem em paralelo com load_jsb — sem esperar controllers.
    after_home = RegisterEventHandler(
        OnProcessExit(target_action=home_setter, on_exit=[load_jsb] + early_nodes))
    after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=load_jsb, on_exit=[load_arm]))

    if end_effector == 'hand':
        load_hand = Node(
            package='controller_manager', executable='spawner',
            arguments=['hand_position_controller',
                       '--controller-manager', '/controller_manager'])
        after_arm = RegisterEventHandler(
            OnProcessExit(target_action=load_arm, on_exit=[load_hand]))
        after_last = RegisterEventHandler(
            OnProcessExit(target_action=load_hand, on_exit=late_nodes))
        chain = [after_spawn, after_home, after_jsb, after_arm, after_last]
    else:  # touch_tool — sem hand controller
        after_arm = RegisterEventHandler(
            OnProcessExit(target_action=load_arm, on_exit=late_nodes))
        chain = [after_spawn, after_home, after_jsb, after_arm]

    return [gazebo, rsp, spawn_robot] + chain


# ──────────────────────────────────────────────────────────────────────
# generate_launch_description
# ──────────────────────────────────────────────────────────────────────
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'end_effector', default_value='hand',
            description='Efector final: hand (COVVI) | touch_tool (Square 20×20 mm)'),
        DeclareLaunchArgument(
            'control_mode', default_value='sim_only',
            description='sim_only | mirror | real_from_sim'),
        DeclareLaunchArgument(
            'robot_ip', default_value='192.168.5.2'),
        DeclareLaunchArgument(
            'robot_dry_run', default_value='true'),
        DeclareLaunchArgument(
            'no_gui', default_value='false'),

        OpaqueFunction(function=launch_setup),
    ])
