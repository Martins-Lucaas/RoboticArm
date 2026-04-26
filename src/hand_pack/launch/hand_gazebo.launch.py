import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Definir o pacote
    package_name = 'hand_pack'
    pkg_share = get_package_share_directory(package_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'linear_covvi_hand_gazebo.urdf')

    # 2. Ler o URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 3. NÓ: Robot State Publisher (O coração do TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 4. INCLUIR: Mundo do Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # 5. NÓ: Spawn Entity (Coloca a mão no Gazebo)
    # Adicionado '-z', '0.1' para garantir que ela não faça spawn afundada no chão
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'covvi_hand', '-z', '0.1'],
        output='screen'
    )

    # 6. NÓS DOS CONTROLADORES (Apenas definidos, não iniciados ainda)
    # O argumento '--controller-manager' força a conexão direta e evita falhas de rede interna
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

    # 7. EVENT HANDLERS (A Mágica da Sincronização)
    
    # A - Só inicia o Broadcaster APÓS o robô terminar de spawnar no Gazebo
    delay_broadcaster_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    # B - Só inicia o Controlador de Posição APÓS o Broadcaster iniciar com sucesso
    delay_controller_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_hand_position_controller],
        )
    )

    # Retorna o Launch final estruturado
    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_entity,
        delay_broadcaster_after_spawn,
        delay_controller_after_broadcaster
    ])