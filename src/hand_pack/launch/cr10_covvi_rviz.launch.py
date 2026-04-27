import os
import re
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _build_combined_urdf():
    # --- CR10 arm (process xacro) ---
    cr10_xacro_path = os.path.join(
        get_package_share_directory('cra_description'), 'urdf', 'cr10_robot.xacro')
    doc = xacro.parse(open(cr10_xacro_path))
    xacro.process_doc(doc)
    cr10_urdf = doc.toxml()

    # --- COVVI hand (raw URDF) ---
    hand_urdf_path = os.path.join(
        get_package_share_directory('hand_pack'), 'urdf', 'linear_covvi_hand_gazebo.urdf')
    with open(hand_urdf_path) as f:
        hand_urdf = f.read()

    # Extract hand content between <robot …> … </robot>
    hand_body = re.search(r'<robot[^>]*>(.*)</robot>', hand_urdf, re.DOTALL).group(1)

    # Remove standalone world / base_footprint links (self-closing or with children)
    hand_body = re.sub(r'<link\s+name="world"\s*/>\s*', '', hand_body)
    hand_body = re.sub(r'<link\s+name="base_footprint"\s*/>\s*', '', hand_body)

    # Remove world_fixed and base_joint joints
    hand_body = re.sub(
        r'<joint\s+name="world_fixed"[^>]*>.*?</joint>', '', hand_body, flags=re.DOTALL)
    hand_body = re.sub(
        r'<joint\s+name="base_joint"[^>]*>.*?</joint>', '', hand_body, flags=re.DOTALL)

    # Rename base_link → hand_base_link everywhere in the hand content
    hand_body = hand_body.replace('"base_link"', '"hand_base_link"')

    # Fixed coupling joint: CR10 Link6 → COVVI hand_base_link
    # Link6 Z axis = flange axis (pointing outward).
    # The hand mesh extends in its local +Y direction from Y=0 (mounting face).
    # rpy="1.5708 0 0" (Rx +90°) aligns hand_Y with Link6_Z so the hand
    # extends along the flange axis. xyz="0 0 0" places the mounting face
    # flush with the Link6 origin (flange center).
    coupling_joint = """
  <joint name="hand_coupling" type="fixed">
    <parent link="Link6"/>
    <child link="hand_base_link"/>
    <origin xyz="0 0 0" rpy="1.5708 0 0"/>
  </joint>
"""

    # Insert hand content + coupling before </robot>
    combined = cr10_urdf.replace('</robot>', hand_body + coupling_joint + '</robot>')
    return combined


def generate_launch_description():
    combined_urdf = _build_combined_urdf()

    rviz_config = os.path.join(
        get_package_share_directory('hand_pack'), 'rviz', 'cr10_covvi.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': combined_urdf}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
