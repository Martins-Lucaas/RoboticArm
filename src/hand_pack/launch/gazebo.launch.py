import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'hand_pack'
    
    # 1. Caminhos dos ficheiros
    pkg_path = get_package_share_directory(package_name)
    # Usamos o URDF com física que criaste
    urdf_file = os.path.join(pkg_path, 'urdf', 'linear_covvi_hand_gazebo.urdf')
    
    # 2. Processar o URDF (necessário para o robot_state_publisher)
    with open(urdf_file, 'r') as infp:
        robot_description_config = infp.read()

    # 3. Node: Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}]
    )

    # 4. Gazebo: Incluir o launch padrão
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # 5. Node: Spawn Entity (Colocar o robô no Gazebo)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'covvi_hand'],
        output='screen'
    )

    # 6. Controladores: Joint State Broadcaster
    # Publica a posição atual das juntas (telemetria)
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # 7. Controladores: Position Controller
    # Permite enviar comandos de movimento para os dedos
    load_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_position_controller"],
    )

    # Retorna a descrição do Launch
    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        load_joint_state_broadcaster,
        load_position_controller
    ])