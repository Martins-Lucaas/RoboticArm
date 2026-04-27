"""
Launch file do sistema autônomo de grasp ML/CV.

Inicia:
  1. Gazebo com o mundo grasp_experiment.world (+ robô CR10+COVVI)
  2. robot_state_publisher
  3. Nós do pipeline ML: detector → estimador → planejador → executor → orquestrador

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
                             TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _build_robot_urdf() -> str:
    """Reutiliza a lógica de montagem do URDF combinado CR10 + COVVI."""
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

    attach_joint = """
    <joint name="hand_attach_joint" type="fixed">
      <parent link="Link6"/>
      <child link="hand_base_link"/>
      <origin xyz="0 0 0.02" rpy="0 0 1.5708"/>
    </joint>"""

    combined = cr10_urdf.replace('</robot>', hand_body + attach_joint + '</robot>')
    return combined


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

    robot_urdf = _build_robot_urdf()

    params_file = os.path.join(pkg_grasp, 'config', 'pipeline_params.yaml')
    world_file  = os.path.join(pkg_grasp, 'worlds', 'grasp_experiment.world')

    # -- Gazebo --
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items())

    # -- Robot State Publisher --
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf,
                     'use_sim_time': True}])

    # -- Spawn robô no Gazebo --
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', 'cr10_covvi',
                   '-x', '0', '-y', '0', '-z', '0'],
        parameters=[{'use_sim_time': True}])

    # -- Controller Manager --
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_urdf,
                     'use_sim_time': True},
                    params_file])

    load_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'])

    load_arm = TimerAction(period=2.0, actions=[
        Node(package='controller_manager', executable='spawner',
             arguments=['cr10_group_controller'])])

    load_hand = TimerAction(period=3.0, actions=[
        Node(package='controller_manager', executable='spawner',
             arguments=['hand_position_controller'])])

    # -- Nós do pipeline ML --
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

    return LaunchDescription([
        use_yolo_arg,
        sim_only_arg,
        gazebo,
        rsp,
        spawn_robot,
        controller_manager,
        load_jsb,
        load_arm,
        load_hand,
        TimerAction(period=4.0, actions=[detector]),
        TimerAction(period=4.0, actions=[pose_estimator]),
        TimerAction(period=4.0, actions=[grasp_planner]),
        TimerAction(period=4.0, actions=[grasp_executor]),
        TimerAction(period=5.0, actions=[pipeline]),
    ])
