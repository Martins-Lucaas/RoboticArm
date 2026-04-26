import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, AppendEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'hand_pack'
    pkg_share = get_package_share_directory(package_name)

    # 1. Caminhos dos arquivos
    urdf_file = os.path.join(pkg_share, 'urdf', 'linear_covvi_hand_gazebo.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'base_hand.world')

    # A MÁGICA CORRIGIDA: Aponta exatamente para a pasta 'share' do install
    # pkg_share = install/hand_pack/share/hand_pack
    # model_path = install/hand_pack/share  (Onde o Gazebo finalmente acha o 'hand_pack' via package://)
    model_path = os.path.abspath(os.path.join(pkg_share, '..'))

    # Anexa o caminho de forma super segura
    append_model_path = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH', model_path
    )

    # 2. Ler o URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 3. NÓ: Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 4. INCLUIR: Mundo do Gazebo (Lista explícita de argumentos para evitar falhas)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments=[('world', world_file)]
    )

    # 5. NÓ: Spawn Entity (z=0.5 para cair suavemente no chão)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'covvi_hand', '-z', '0.5'],
        output='screen'
    )

    # 6. NÓS DOS CONTROLADORES
    load_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output='screen'
    )

    load_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_position_controller"],
        output='screen'
    )

    # 7. Sincronização Perfeita (Garante que os controllers só liguem após o robô existir)
    delay_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_broadcaster],
        )
    )

    delay_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_broadcaster,
            on_exit=[load_controller],
        )
    )

    return LaunchDescription([
        append_model_path,
        robot_state_publisher,
        gazebo,
        spawn_entity,
        delay_broadcaster,
        delay_controller
    ])