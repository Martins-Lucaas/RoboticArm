import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Defina o nome do seu pacote (assumindo que ainda seja hand_pack)
    package_name = 'hand_pack'
    
    # 2. Caminho exato para o seu novo arquivo URDF
    pkg_path = get_package_share_directory(package_name)
    urdf_file = os.path.join(pkg_path, 'urdf', 'linear_covvi_hand_right.urdf')

    # 3. Lê o conteúdo do arquivo URDF diretamente (não precisa do xacro para .urdf puro)
    with open(urdf_file, 'r') as infp:
        robot_description_raw = infp.read()

    # 4. Declara os Nodes necessários
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Usar simulação (Gazebo) se verdadeiro'),

        # Publica o estado do robô e as transformações (TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_raw,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        # Abre a interface com barras deslizantes para mover as juntas (polegar, indicador, etc)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Abre o RViz2 para visualização 3D
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])