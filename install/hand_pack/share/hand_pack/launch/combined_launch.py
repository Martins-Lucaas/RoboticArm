import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Diretório do pacote ur_description
    ur_description_dir = get_package_share_directory('ur_description')

    # Caminho para o arquivo Xacro do robô (com a mão)
    ur_xacro_file = os.path.join(ur_description_dir, 'urdf', 'ur_macro.xacro')

    # Argumento para selecionar o tipo de robô (ex: ur5, ur10)
    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur5", description="Type of Universal Robot (e.g., ur5, ur10)"),
    ]

    # Obtemos o tipo de robô configurado
    ur_type = LaunchConfiguration("ur_type")

    # Processamos o arquivo Xacro para gerar a descrição URDF
    urdf_content = subprocess.run(
        ["ros2", "run", "xacro", "xacro", ur_xacro_file, "ur_type:=", "ur5"],
        capture_output=True, text=True, check=True
    ).stdout

    # Definimos o parâmetro 'robot_description'
    robot_description = {"robot_description": urdf_content}

    # Nó do robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # Nó do Joint State Publisher (para visualização no RViz)
    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    # RViz2 para visualização do modelo
    rviz_config_file = os.path.join(ur_description_dir, 'rviz', 'ur5.rviz')
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    return LaunchDescription(declared_arguments + [
        robot_state_publisher,
        joint_state_publisher,
        rviz_node,
    ])
