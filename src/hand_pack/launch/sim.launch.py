#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('hand_pack')
    webots_home = os.environ.get('WEBOTS_HOME', '/usr/share/webots')

    return LaunchDescription([
        # para achar o libController.so
        SetEnvironmentVariable('WEBOTS_HOME', webots_home),
        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            os.path.join(webots_home, 'lib', 'controller') +
            ':' + os.environ.get('LD_LIBRARY_PATH', '')
        ),
        # aqui diz pro Webots onde estão os controllers customizados
        SetEnvironmentVariable(
            'WEBOTS_CONTROLLER_PATH',
            os.path.join(pkg, 'controllers')
        ),

        # inicia Webots pausado
        ExecuteProcess(
            cmd=[
                'webots',
                '--mode=pause',
                os.path.join(pkg, 'worlds', 'ure.wbt')
            ],
            output='screen'
        ),

        # seu nó ROS que vai chamar step e etc
        Node(
            package='hand_pack',
            executable='hand_node',
            output='screen'
        ),
    ])
