# launch/shadow_hand_rviz.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('covvi')

    # URDF da mão
    urdf_path = os.path.join(pkg_share, 'urdf', 'shadow_hand.urdf')
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f'URDF não encontrado: {urdf_path}')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # RViz config (usa se existir)
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'shadow_hand.rviz')
    rviz_config_exists = os.path.exists(default_rviz_config)

    # Args
    use_gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Usa joint_state_publisher_gui para sliders de juntas'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Abre o RViz2 com o modelo carregado'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=(default_rviz_config if rviz_config_exists else ''),
        description='Caminho para um arquivo .rviz (opcional)'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use /clock (simulação)'
    )

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Nodes
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    jsp_gui_node = Node(
        condition=IfCondition(gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    jsp_node = Node(
        condition=UnlessCondition(gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Se houver um .rviz, passa -d; senão, abre RViz “vazio”
    rviz_args = ['-d', rviz_config] if rviz_config_exists else []

    rviz_node = Node(
        condition=IfCondition(rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args
    )

    return LaunchDescription([
        use_gui_arg,
        use_rviz_arg,
        rviz_config_arg,
        use_sim_time_arg,
        rsp_node,
        jsp_gui_node,
        jsp_node,
        rviz_node
    ])
