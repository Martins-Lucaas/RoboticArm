"""
Launch file do sistema autônomo de grasp ML/CV.

Inicia:
  1. Gazebo com o mundo grasp_experiment.world (+ robô CR10+COVVI)
  2. robot_state_publisher  (URDF mínimo — sem meshes/inertials)
  3. spawn_entity           (URDF completo gravado em /tmp)
  4. Controllers carregados em cadeia via OnProcessExit
  5. Nós do pipeline ML: detector → estimador → planejador → executor → orquestrador

Uso:
    ros2 launch grasp_ml_pack grasp_pipeline.launch.py
    ros2 launch grasp_ml_pack grasp_pipeline.launch.py use_yolo:=true
    ros2 launch grasp_ml_pack grasp_pipeline.launch.py sim_only:=true
"""

import os
import re
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                             RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _fix_virtual_link_inertia(urdf_body: str) -> str:
    """
    Substitui inércia irreal nos links virtuais da mão (driver_link e _l1).

    O URDF exportado pelo SolidWorks atribui mass=1 kg e ixx=iyy=izz=1 kg·m² a 32
    links sem geometria (links de constraint da cadeia de 4 barras que simula o
    mecanismo mimic). Com esses valores:
      - 32 kg de massa invisível recebe forças de gravidade / aceleração
      - O ODE precisa resolver constraints entre inércias de 1 kg·m² e links reais
        de 0.01 kg → razão de inércia 100× → instabilidade numérica garantida
    Solução: reduzir para mass=0.001 kg e ixx=iyy=izz=1e-9 (praticamente massless).
    """
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
    """
    Adiciona amortecimento a todos os joints revolute da mão com valores distintos:
      - Joints mimic: damping alto (120) — Gazebo ignora <mimic>, precisam resistir
        às forças de constraint para permanecerem na posição correta.
      - Joints primários (Thumb, Index...): damping baixo (10) — são controlados
        via SetPosition() a cada step de física (1 kHz), mas sem nenhum damping
        o ODE não consegue resolver bem as forças de constraint transmitidas pelo
        hand_attach_joint quando o braço se move, causando instabilidade numérica.
    """
    def _patch(m: re.Match) -> str:
        jxml = m.group(0)
        if 'type="revolute"' not in jxml:
            return jxml
        if '<mimic' in jxml:
            damp, fric = 120.0, 20.0
        else:
            damp, fric = 10.0, 2.0
        dyn_tag = f'<dynamics damping="{damp}" friction="{fric}"/>'
        if '<dynamics' in jxml:
            jxml = re.sub(r'<dynamics[^/]*/>', dyn_tag, jxml)
        else:
            jxml = jxml.replace(
                '</joint>',
                f'      {dyn_tag}\n    </joint>')
        return jxml

    return re.sub(r'<joint\b[^>]*>.*?</joint>', _patch, urdf_body, flags=re.DOTALL)


def _build_robot_urdf():
    """
    Monta o URDF combinado CR10 + COVVI.

    Retorna (full_urdf, minimal_urdf):
      - full_urdf    → gravado em /tmp e usado pelo spawn_entity (tem meshes)
      - minimal_urdf → passado ao robot_state_publisher como robot_description
                       (sem meshes/inertials/collisions, string pequena o suficiente
                       para o gazebo_ros2_control plugin não quebrar no parser de args)
    """
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

    # Corrige inércia dos 32 links virtuais (mass=1→0.001, ixx=1→1e-9).
    # Sem isso, 32 kg de massa invisível recebe forças do ODE e destrói a simulação.
    hand_body = _fix_virtual_link_inertia(hand_body)

    # Estabiliza joints da mão: mimic (damping=120) e primários (damping=10).
    hand_body = _stabilize_hand_joints(hand_body)

    # Desativa gravidade e self_collide em todos os links da mão
    hand_link_names = re.findall(r'<link\s+name="([^"]+)"', hand_body)
    for lname in hand_link_names:
        hand_body += (
            f'\n  <gazebo reference="{lname}">'
            f'<gravity>false</gravity>'
            f'<self_collide>false</self_collide>'
            f'<kp>1e6</kp><kd>1.0</kd>'
            f'</gazebo>'
        )

    attach_joint = """
    <joint name="hand_attach_joint" type="fixed">
      <parent link="Link6"/>
      <child link="hand_base_link"/>
      <origin xyz="0 0 0.01" rpy="1.5708 0 0"/>
    </joint>"""

    full_urdf = cr10_urdf.replace('</robot>', hand_body + attach_joint + '</robot>')

    # Desativa self_collide nos links do braço CR10 (evita links adjacentes se penetrarem)
    arm_link_names = re.findall(r'<link\s+name="([^"]+)"', cr10_urdf)
    arm_gazebo_tags = ''
    for lname in arm_link_names:
        arm_gazebo_tags += (
            f'\n  <gazebo reference="{lname}">'
            f'<self_collide>false</self_collide>'
            f'</gazebo>'
        )
    full_urdf = full_urdf.replace('</robot>', arm_gazebo_tags + '\n</robot>')

    # URDF mínimo para o robot_state_publisher:
    # O plugin gazebo_ros2_control lê robot_description do RSP via serviço.
    # Passar o URDF completo quebra o parser de argumentos do plugin (string muito longa).
    # Solução: RSP publica o URDF sem meshes/inertiais/colisões (estrutura cinemática
    # idêntica); o spawn usa o arquivo completo /tmp/*.urdf com todos os assets.
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
    pkg_grasp = get_package_share_directory('grasp_ml_pack')
    pkg_gazebo = get_package_share_directory('gazebo_ros')

    use_yolo_arg = DeclareLaunchArgument(
        'use_yolo', default_value='false',
        description='Usar YOLOv8 em vez de segmentação HSV')
    sim_only_arg = DeclareLaunchArgument(
        'sim_only', default_value='true',
        description='Apenas simulação (sem robô físico)')

    use_yolo = LaunchConfiguration('use_yolo')

    full_urdf, minimal_urdf = _build_robot_urdf()

    # Grava o URDF completo em /tmp para o spawn_entity usar (preserva meshes)
    urdf_spawn_path = '/tmp/cr10_covvi_grasp.urdf'
    with open(urdf_spawn_path, 'w') as f:
        f.write(full_urdf)

    params_file = os.path.join(pkg_grasp, 'config', 'pipeline_params.yaml')
    world_file  = os.path.join(pkg_grasp, 'worlds', 'grasp_experiment.world')

    # -- Gazebo --
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items())

    # -- Robot State Publisher (URDF mínimo — sem meshes) --
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': minimal_urdf,
                     'use_sim_time': True}])

    # -- Spawn robô no Gazebo (URDF completo via arquivo) --
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', urdf_spawn_path,
                   '-entity', 'cr10_covvi',
                   '-x', '0', '-y', '0', '-z', '0.375'],
        parameters=[{'use_sim_time': True}])

    # -- Controllers carregados em cadeia após o spawn --
    load_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'])

    load_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['cr10_group_controller',
                   '--controller-manager', '/controller_manager'])

    load_hand = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hand_position_controller',
                   '--controller-manager', '/controller_manager'])

    after_spawn_load_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[load_jsb]))

    after_jsb_load_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_jsb,
            on_exit=[load_arm]))

    after_arm_load_hand = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_arm,
            on_exit=[load_hand]))

    # -- Nós do pipeline ML (aguardam controllers estarem ativos) --
    detector = Node(
        package='grasp_ml_pack',
        executable='object_detector',
        parameters=[params_file, {'use_yolo': use_yolo}])

    pose_estimator = Node(
        package='grasp_ml_pack',
        executable='pose_estimator',
        parameters=[params_file])

    grasp_planner = Node(
        package='grasp_ml_pack',
        executable='grasp_planner',
        parameters=[params_file])

    grasp_executor = Node(
        package='grasp_ml_pack',
        executable='grasp_executor',
        parameters=[params_file])

    pipeline = Node(
        package='grasp_ml_pack',
        executable='pipeline',
        parameters=[params_file])

    # Sobe os nós ML após o hand controller estar ativo
    after_hand_start_ml = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_hand,
            on_exit=[detector, pose_estimator, grasp_planner,
                     grasp_executor, pipeline]))

    return LaunchDescription([
        use_yolo_arg,
        sim_only_arg,
        gazebo,
        rsp,
        spawn_robot,
        after_spawn_load_jsb,
        after_jsb_load_arm,
        after_arm_load_hand,
        after_hand_start_ml,
    ])
