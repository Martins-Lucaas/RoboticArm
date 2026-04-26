from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rqt_gui = Node(
        package='rqt_joint_trajectory_controller',
        executable='rqt_joint_trajectory_controller',
        name='rqt_joint_trajectory_controller',
    )

    return LaunchDescription([
        rqt_gui
    ])