from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    declared_arguments = []

    # Argumentos específicos do robô UR
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]
            ),
            description="URDF/XACRO description file (absolute path) with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_description"), "rviz", "view_robot.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching rviz.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix of the joint names for multi-robot setups.",
        )
    )
    # Escolha entre Gazebo, Webots ou RViz
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo",
            default_value="false",
            choices=["true", "false"],
            description="Se 'true', inicia o Gazebo ao invés do RViz.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "webots",
            default_value="false",
            choices=["true", "false"],
            description="Se 'true', inicia o Webots ao invés do RViz ou Gazebo.",
        )
    )

    # Configurações de launch
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    description_file = LaunchConfiguration("description_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    gazebo_arg = LaunchConfiguration("gazebo")
    webots_arg = LaunchConfiguration("webots")

    # Geração da descrição do robô usando xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",
            "safety_limits:=", safety_limits,
            " ",
            "safety_pos_margin:=", safety_pos_margin,
            " ",
            "safety_k_position:=", safety_k_position,
            " ",
            "name:=", "ur",
            " ",
            "ur_type:=", ur_type,
            " ",
            "tf_prefix:=", tf_prefix,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(
            value=robot_description_content, value_type=str
        )
    }

    # Condição combinada para não iniciar nós de RViz e Joint State Publisher se estiver em simulação (Gazebo ou Webots)
    sim_condition = PythonExpression([
        "'", gazebo_arg, "'=='true' or '", webots_arg, "'=='true'"
    ])

    # ---------------------------------------------------------------------
    # Nós ROS
    # ---------------------------------------------------------------------

    # Joint State Publisher GUI só é iniciado se NÃO estiver em simulação (nem Gazebo nem Webots)
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=UnlessCondition(sim_condition)
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # RViz é iniciado somente se não estiver em simulação (nem Gazebo nem Webots)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=UnlessCondition(sim_condition),
    )

    # ---------------------------------------------------------------------
    # Gazebo
    # ---------------------------------------------------------------------
    # Inclui o launch do Gazebo somente se o argumento gazebo for true
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('ur_description'),
                'worlds',
                'gazebo.world'
            ])
        }.items(),
        condition=IfCondition(gazebo_arg)
    )

    # Nó de spawn do robô no Gazebo (será lançado com atraso para garantir que o Gazebo esteja pronto)
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'ur_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    spawn_entity_node_delayed = TimerAction(
        period=20.0,  # Atraso de 20 segundos; ajuste conforme necessário
        actions=[spawn_entity_node],
        condition=IfCondition(gazebo_arg)
    )

    # ---------------------------------------------------------------------
    # Webots
    # ---------------------------------------------------------------------
    # Inclui o launch do Webots somente se o argumento webots for true
    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('webots_ros2'),
                'launch',
                'webots.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('ur_description'),
                'worlds',
                'webots.world'
            ])
        }.items(),
        condition=IfCondition(webots_arg)
    )

    # ---------------------------------------------------------------------
    # Junta todos os componentes
    # ---------------------------------------------------------------------
    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,                    # Inicia se não estiver em simulação
        gazebo_launch,                # Inicia se gazebo==true
        spawn_entity_node_delayed,    # Inicia com atraso se gazebo==true
        webots_launch,                # Inicia se webots==true
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
