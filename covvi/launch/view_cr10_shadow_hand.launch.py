# launch/view_cr10_shadow_hand.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

PKG = 'covvi'

def launch_setup(context, *args, **kwargs):
    pkg_share = Path(get_package_share_directory(PKG))

    # Args
    urdf_arg = LaunchConfiguration('urdf').perform(context)
    use_gui  = LaunchConfiguration('gui').perform(context).lower() in ('1', 'true', 'yes', 'y')
    use_rviz = LaunchConfiguration('rviz').perform(context).lower() in ('1', 'true', 'yes', 'y')

    # Caminho padrão: <pkg_share>/urdf/cr10_shadow_hand.urdf
    urdf_path = Path(urdf_arg) if urdf_arg else (pkg_share / 'urdf' / 'cr10_shadow_hand.urdf')
    if not urdf_path.is_file():
        raise FileNotFoundError(f'URDF não encontrado: {urdf_path}')

    # Carrega URDF como string para robot_description
    robot_description = {'robot_description': urdf_path.read_text()}

    nodes = []

    # Joint states (GUI opcional)
    if use_gui:
        nodes.append(
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
                output='screen'
            )
        )
    else:
        nodes.append(
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='screen'
            )
        )

    # Publica TFs/descrição do robô
    nodes.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        )
    )

    # RViz2 (abre sem config se não existir .rviz no pacote)
    if use_rviz:
        rviz_cfg = pkg_share / 'rviz' / 'view.rviz'
        rviz_args = ['-d', str(rviz_cfg)] if rviz_cfg.is_file() else []
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=rviz_args
            )
        )

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf',
            default_value='',
            description='Caminho absoluto para um URDF alternativo (deixe vazio para usar o do pacote).'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Abrir joint_state_publisher_gui (true/false).'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Abrir RViz2 automaticamente (true/false).'
        ),
        OpaqueFunction(function=launch_setup)
    ])
