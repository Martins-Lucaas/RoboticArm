from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    xacro_file = PathJoinSubstitution([
        FindPackageShare('ur_description'),
        'urdf',
        'ur_mocked.urdf.xacro'
    ])

    robot_description_content = Command([
        'xacro', ' ',
        xacro_file,
        ' ', 'ur_type:=ur5e',
        ' ', 'name:=ur'
    ])

    robot_description = {'robot_description': robot_description_content}

    return LaunchDescription([
        # Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Publisher do URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen'
        ),

        # Spawn do robô no Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'ur5e_with_hand',
                '-x', '0', '-y', '0', '-z', '1'
            ],
            output='screen'
        )
    ])
