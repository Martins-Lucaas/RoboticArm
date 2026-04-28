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


def _add_mimic_damping(urdf_body: str,
                       damping: float = 50.0,
                       friction: float = 10.0) -> str:
    """
    Adiciona alto amortecimento/atrito a todos os mimic joints do URDF da mão.

    Gazebo Classic ignora a tag <mimic> — sem amortecimento, os mimic joints
    ficam livres durante os ~3 s de inicialização e explodem fisicamente.
    Com damping=50 N·m·s/rad eles ficam estáticos até o controller ativar.
    """
    def _patch(m: re.Match) -> str:
        jxml = m.group(0)
        if '<mimic' not in jxml:
            return jxml
        dyn_tag = f'<dynamics damping="{damping}" friction="{friction}"/>'
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

    # Estabiliza mimic joints: alto amortecimento evita explosão física no startup
    hand_body = _add_mimic_damping(hand_body)

    # Desativa gravidade em todos os links da mão para evitar desmontagem
    # quando os controllers ainda não carregaram
    hand_link_names = re.findall(r'<link\s+name="([^"]+)"', hand_body)
    for lname in hand_link_names:
        hand_body += (
            f'\n  <gazebo reference="{lname}">'
            f'<gravity>false</gravity>'
            f'<self_collide>false</self_collide>'
            f'</gazebo>'
        )

    attach_joint = """
    <joint name="hand_attach_joint" type="fixed">
      <parent link="Link6"/>
      <child link="hand_base_link"/>
      <origin xyz="0 0 0.01" rpy="1.5708 0 0"/>
    </joint>"""

    full_urdf = cr10_urdf.replace('</robot>', hand_body + attach_joint + '</robot>')

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
