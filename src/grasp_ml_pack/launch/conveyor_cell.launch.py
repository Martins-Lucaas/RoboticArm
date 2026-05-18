"""
Launch file da célula de manufatura com esteira — CR10 + COVVI Hand.

Inicia:
  1. Gazebo com conveyor_cell.world
  2. robot_state_publisher (URDF combinado CR10+COVVI)
  3. spawn_entity
  4. Controllers (joint_state_broadcaster → cr10_group → hand_position)
  5. Nós da célula:
       object_detector   — visão (HSV ou YOLOv8)
       grasp_executor    — pick-and-place determinístico
       conveyor_controller — controle da esteira via Gazebo spawn/delete
       gui_control_node  — painel de operação Tkinter
       conveyor_pipeline — orquestrador / status aggregator

Uso:
  ros2 launch grasp_ml_pack conveyor_cell.launch.py
  ros2 launch grasp_ml_pack conveyor_cell.launch.py use_yolo:=true
  ros2 launch grasp_ml_pack conveyor_cell.launch.py sim_only:=false
  ros2 launch grasp_ml_pack conveyor_cell.launch.py no_gui:=true autonomous:=true
"""

import os
import re
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, RegisterEventHandler,
                             TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription


def _fix_virtual_link_inertia(urdf_body: str) -> str:
    phantom_inertial = (
        r'<inertial>\s*'
        r'<mass value="1"\s*/>\s*'
        r'<inertia ixx="1\.0" ixy="0\.0" ixz="0\.0" iyy="1\.0" iyz="0\.0" izz="1\.0"\s*/>\s*'
        r'</inertial>'
    )
    minimal_inertial = (
        '<inertial>'
        '<mass value="0.001"/>'
        '<inertia ixx="1e-9" ixy="0.0" ixz="0.0" iyy="1e-9" iyz="0.0" izz="1e-9"/>'
        '</inertial>'
    )
    return re.sub(phantom_inertial, minimal_inertial, urdf_body, flags=re.DOTALL)


def _stabilize_hand_joints(urdf_body: str) -> str:
    """Patcha dinâmica + limites das juntas da mão COVVI para permitir
    grasp por contato físico real (sem kinematic attach).

    Mudanças vs. URDF original:
      • effort: 1.0 → 8.0 N·m (juntas primárias) — força de fechamento
        suficiente para o atrito sustentar 180g (frasco) sem deslizar.
        Industry baseline: Robotiq adaptive ~10 N·m, Schunk SDH ~5 N·m.
      • damping: mimic 120 → 30, primária 10 → 5 — dedos transmitem
        força mais rápido para o objeto antes do controlador saturar.
      • friction (junta): mantida (estabilidade do solver ODE).
    """
    def _patch(m: re.Match) -> str:
        jxml = m.group(0)
        if 'type="revolute"' not in jxml:
            return jxml
        is_mimic = '<mimic' in jxml
        damp, fric = (30.0, 10.0) if is_mimic else (5.0, 1.0)
        dyn_tag = f'<dynamics damping="{damp}" friction="{fric}"/>'
        if '<dynamics' in jxml:
            jxml = re.sub(r'<dynamics[^/]*/>', dyn_tag, jxml)
        else:
            jxml = jxml.replace('</joint>',
                                f'      {dyn_tag}\n    </joint>')
        # Boost no effort_limit (apenas para juntas PRIMÁRIAS, não mimic)
        # para o PID desenvolver força de fechamento real.
        if not is_mimic:
            jxml = re.sub(r'effort="[\d.]+"', 'effort="8.0"', jxml)
        return jxml
    return re.sub(r'<joint\b[^>]*>.*?</joint>', _patch, urdf_body, flags=re.DOTALL)


def _inject_skin_layer(urdf_body: str) -> str:
    """Injeta uma camada de "pele" macia ao redor de cada falange da mão.

    Motivação
    ─────────
    O URDF original modela cada falange como uma AABB rígida; entre
    falanges adjacentes existem cantos vivos e descontinuidades onde o
    objeto pode encaixar e ser ejetado lateralmente pelo contato. Em
    mãos reais (incluindo a COVVI física) há uma luva de silicone
    coobrindo todos os dedos — esse "skin" cria uma superfície contínua,
    macia e de alto atrito ao redor do esqueleto rígido.

    Implementação
    ─────────────
    Para cada link de falange (`*_proximal` e `*_distal` dos 5 dedos),
    duplica a `<collision>` original adicionando uma SEGUNDA caixa com:
      • dimensões + `_SKIN_INFLATE_M` em CADA eixo (envelope ~3mm além
        do esqueleto);
      • mesmo `origin xyz/rpy` da original (permanece colada na falange);
    URDF + Gazebo aceitam múltiplos `<collision>` por link e somam-nos
    no shape do ODE. A pele é regida pelas mesmas tags `<gazebo
    reference>` do link (kp=5e4 + kd=50 + mu=1.5 + maxVel=0.01), então
    automaticamente herda contato macio e atrito alto.

    Para a `lisa` (palma) que usa colisão de malha, é adicionado um box
    inflado aproximado para suavizar a interface palma-objeto.
    """
    _SKIN_INFLATE_M = 0.003   # 3mm de pele por face (6mm de aumento total/eixo)

    # 1) Box-collisions de falanges — inflar e duplicar
    finger_link_pat = re.compile(
        r'(<link\s+name="(?:thumb|index|middle|ring|little)_(?:proximal|distal)"[^>]*>'
        r'.*?</link>)',
        re.DOTALL)

    box_coll_pat = re.compile(
        r'(<collision>\s*<geometry>\s*<box\s+size="([^"]+)"\s*/>\s*</geometry>'
        r'\s*<origin\s+xyz="([^"]+)"\s+rpy="([^"]+)"\s*/>\s*</collision>)',
        re.DOTALL)

    def _patch_finger_link(m: re.Match) -> str:
        link_xml = m.group(0)

        def _patch_collision(cm: re.Match) -> str:
            original = cm.group(1)
            sizes = [float(s) for s in cm.group(2).split()]
            xyz   = cm.group(3)
            rpy   = cm.group(4)
            inflated = ' '.join(
                f'{s + 2 * _SKIN_INFLATE_M:.5f}' for s in sizes)
            skin = (
                f'\n        <collision name="skin">'
                f'<geometry><box size="{inflated}"/></geometry>'
                f'<origin xyz="{xyz}" rpy="{rpy}"/>'
                f'</collision>'
            )
            return original + skin

        return box_coll_pat.sub(_patch_collision, link_xml, count=1)

    urdf_body = finger_link_pat.sub(_patch_finger_link, urdf_body)

    # 2) Palma (lisa) — usa colisão de malha; adiciona box-skin
    # aproximado ao redor da palma. Geometria do right_lisa.STL:
    # palm ocupa aprox. x ∈ [-0.04, +0.04], y ∈ [0, +0.092], z ∈
    # [-0.02, +0.02] no frame de hand_base. Box centrado em
    # (0, 0.046, 0) com tamanho (0.085, 0.092, 0.045) — 3mm de pele
    # em cada face suaviza o contato sem mudar o footprint visível.
    palm_skin = (
        '\n        <collision name="palm_skin">'
        '<geometry><box size="0.085 0.092 0.045"/></geometry>'
        '<origin xyz="0 0.046 0" rpy="0 0 0"/>'
        '</collision>'
    )
    urdf_body = re.sub(
        r'(<link\s+name="lisa">.*?<collision>\s*<geometry>\s*<mesh\b[^>]*?/>'
        r'\s*</geometry>\s*<origin\b[^>]*?/>\s*</collision>)',
        lambda m: m.group(1) + palm_skin,
        urdf_body, count=1, flags=re.DOTALL)

    return urdf_body


def _build_robot_urdf():
    hand_pack_share = get_package_share_directory('hand_pack')
    combined_yaml = os.path.join(
        hand_pack_share, 'config', 'cr10_covvi_controllers.yaml')

    cr10_xacro_path = os.path.join(
        get_package_share_directory('cra_description'),
        'urdf', 'cr10_robot.xacro')
    doc = xacro.parse(open(cr10_xacro_path))
    xacro.process_doc(doc)
    cr10_urdf = doc.toxml()
    cr10_urdf = re.sub(
        r'<parameters>[^<]*/ros2_controllers\.yaml</parameters>',
        f'<parameters>{combined_yaml}</parameters>',
        cr10_urdf)

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
        r'<joint\s+name="world_fixed"[^>]*>.*?</joint>', '', hand_body, flags=re.DOTALL)
    hand_body = re.sub(
        r'<joint\s+name="base_joint"[^>]*>.*?</joint>', '', hand_body, flags=re.DOTALL)
    hand_body = hand_body.replace('"base_link"', '"hand_base_link"')
    hand_body = re.sub(
        r'<gazebo>\s*<plugin[^>]*gazebo_ros2_control[^>]*>.*?</plugin>\s*</gazebo>',
        '', hand_body, flags=re.DOTALL)
    hand_body = hand_body.replace(
        '<ros2_control name="GazeboSystem"',
        '<ros2_control name="HandGazeboSystem"')

    hand_body = _fix_virtual_link_inertia(hand_body)
    hand_body = _stabilize_hand_joints(hand_body)
    hand_body = _inject_skin_layer(hand_body)

    # Tag <gazebo reference> por link da mão.
    #
    # CONTATO SUAVE para grasp sem ejeção:
    #   kp=5e4 (era 1e6) — rigidez Hertziana 20× menor: a "mola" do
    #     contato armazena MUITO menos energia ao penetrar, então o
    #     impulso de separação é proporcionalmente menor → o objeto
    #     não sai disparado quando o dedo toca.
    #   kd=50 (era 1.0) — amortecimento alto na junção de contato
    #     dissipa a velocidade relativa rapidamente; sem isso o objeto
    #     oscila/quica antes de estabilizar.
    #   maxContacts=8 (era default) — mais pontos de contato suaviza
    #     a distribuição da força ao redor da geometria cilíndrica.
    #   minDepth=0.0005 — pequena interpenetração permitida absorve o
    #     impacto inicial sem disparar correção brusca do solver.
    #
    # Industry baseline: ROS-Industrial gripper sims usam kp ≈ 1e4–1e5
    # com kd ~ 1–10% de kp para grasping estável.
    hand_link_names = re.findall(r'<link\s+name="([^"]+)"', hand_body)
    for lname in hand_link_names:
        hand_body += (
            f'\n  <gazebo reference="{lname}">'
            f'<gravity>false</gravity>'
            f'<self_collide>false</self_collide>'
            f'<kp>5e4</kp><kd>50.0</kd>'
            f'<maxContacts>8</maxContacts>'
            f'<minDepth>0.0005</minDepth>'
            f'<maxVel>0.01</maxVel>'
            f'</gazebo>'
        )

    # Acoplamento flush: face de montagem da mão = origem do Link6
    # (centro do flange). Rx(+90°) alinha o eixo Y da mão com o eixo Z
    # do flange para a palma estender axialmente ao pulso.
    attach_joint = """
    <joint name="hand_attach_joint" type="fixed">
      <parent link="Link6"/>
      <child link="hand_base_link"/>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    </joint>"""

    full_urdf = cr10_urdf.replace('</robot>', hand_body + attach_joint + '</robot>')

    arm_link_names = re.findall(r'<link\s+name="([^"]+)"', cr10_urdf)
    arm_gazebo_tags = ''
    for lname in arm_link_names:
        arm_gazebo_tags += (
            f'\n  <gazebo reference="{lname}">'
            f'<self_collide>false</self_collide>'
            f'</gazebo>'
        )
    full_urdf = full_urdf.replace('</robot>', arm_gazebo_tags + '\n</robot>')

    minimal = full_urdf
    minimal = re.sub(r'<visual\b[^>]*>.*?</visual>', '', minimal, flags=re.DOTALL)
    minimal = re.sub(r'<collision\b[^>]*>.*?</collision>', '', minimal, flags=re.DOTALL)
    minimal = re.sub(r'<inertial\b[^>]*>.*?</inertial>', '', minimal, flags=re.DOTALL)
    minimal = re.sub(
        r'<gazebo\s+reference\s*=\s*"[^"]*"\s*>.*?</gazebo>', '', minimal, flags=re.DOTALL)
    minimal = re.sub(r'<!--.*?-->', '', minimal, flags=re.DOTALL)
    minimal = re.sub(r'<\?xml[^?]*\?>', '', minimal)
    minimal = ' '.join(minimal.split())

    return full_urdf, minimal


def generate_launch_description():
    pkg_grasp  = get_package_share_directory('grasp_ml_pack')
    pkg_gazebo = get_package_share_directory('gazebo_ros')

    # Argumentos de launch
    use_yolo_arg  = DeclareLaunchArgument('use_yolo',    default_value='false')
    sim_only_arg  = DeclareLaunchArgument('sim_only',    default_value='true')
    no_gui_arg    = DeclareLaunchArgument('no_gui',      default_value='false')
    auto_arg      = DeclareLaunchArgument('autonomous',  default_value='false')

    use_yolo   = LaunchConfiguration('use_yolo')
    sim_only   = LaunchConfiguration('sim_only')
    no_gui     = LaunchConfiguration('no_gui')
    autonomous = LaunchConfiguration('autonomous')

    full_urdf, minimal_urdf = _build_robot_urdf()
    urdf_spawn_path = '/tmp/cr10_covvi_cell.urdf'
    with open(urdf_spawn_path, 'w') as f:
        f.write(full_urdf)

    params_file = os.path.join(pkg_grasp, 'config', 'pipeline_params.yaml')
    world_file  = os.path.join(pkg_grasp, 'worlds', 'conveyor_cell.world')

    # ── Gazebo ─────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items())

    # ── Robot State Publisher ───────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': minimal_urdf, 'use_sim_time': True}])

    # ── Spawn ───────────────────────────────────────────────────────────
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', urdf_spawn_path, '-entity', 'cr10_covvi',
                   '-x', '0', '-y', '0', '-z', '0.375'],
        parameters=[{'use_sim_time': True}])

    # ── Controllers (cadeia de dependência) ────────────────────────────
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
        event_handler=OnProcessExit(target_action=spawn_robot, on_exit=[load_jsb]))
    after_jsb_load_arm = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=load_jsb, on_exit=[load_arm]))
    after_arm_load_hand = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=load_arm, on_exit=[load_hand]))

    # ── Nós da célula (aguardam hand controller) ───────────────────────
    detector = Node(
        package='grasp_ml_pack',
        executable='object_detector',
        parameters=[params_file, {'use_yolo': use_yolo}])

    executor_node = Node(
        package='grasp_ml_pack',
        executable='grasp_executor',
        parameters=[params_file])

    # NOTA: o grasp_attacher_node (kinematic teleport) foi removido —
    # a captura agora é feita por contato físico real (atrito ODE +
    # esforço dos dedos COVVI), como em manipuladores industriais.

    conveyor = Node(
        package='grasp_ml_pack',
        executable='conveyor_controller',
        parameters=[params_file, {'sim_only': sim_only}])

    # GUI = manual_control (substitui o gui_control antigo). Expõe o
    # ciclo completo de pick em 6 fases (F1..F6) com gating por
    # /joint_states e attach kinemático imediato em F5 — fluxo testado
    # para frasco/ampola/tubo. Para desabilitar a GUI: no_gui:=true.
    gui = Node(
        package='grasp_ml_pack',
        executable='manual_control',
        condition=UnlessCondition(no_gui))

    pipeline = Node(
        package='grasp_ml_pack',
        executable='pipeline',
        parameters=[params_file, {'autonomous': autonomous}])

    after_hand_start_cell = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_hand,
            on_exit=[detector, executor_node,
                     conveyor, gui, pipeline]))

    return LaunchDescription([
        use_yolo_arg,
        sim_only_arg,
        no_gui_arg,
        auto_arg,
        gazebo,
        rsp,
        spawn_robot,
        after_spawn_load_jsb,
        after_jsb_load_arm,
        after_arm_load_hand,
        after_hand_start_cell,
    ])
