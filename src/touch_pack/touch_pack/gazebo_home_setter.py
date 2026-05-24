"""
gazebo_home_setter.py — Posiciona o robô simulado na Home logo após o spawn.

Lê ~/.config/touch_pack/home_pose.json (graus, convenção URDF) e chama
/gazebo/set_model_configuration para colocar as 6 juntas do CR10 na
posição correta antes dos controllers subirem. Encerra sozinho após a
chamada (uso único no launch).
"""
from __future__ import annotations

import json
import math
import os
import sys

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetModelConfiguration

HOME_POSE_FILE = os.path.expanduser('~/.config/touch_pack/home_pose.json')
_ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
_ARM_HOME_DEG_DEFAULT = {
    'joint1':  0.0, 'joint2':  0.0, 'joint3': -90.0,
    'joint4':  0.0, 'joint5': 90.0, 'joint6':   0.0,
}


class GazeboHomeSetter(Node):
    def __init__(self):
        super().__init__('gazebo_home_setter')
        self._cli = self.create_client(
            SetModelConfiguration, '/gazebo/set_model_configuration')

    def run(self) -> bool:
        home_deg = dict(_ARM_HOME_DEG_DEFAULT)
        try:
            if os.path.exists(HOME_POSE_FILE):
                with open(HOME_POSE_FILE) as fh:
                    data = json.load(fh)
                for j in _ARM_JOINTS:
                    if j in data:
                        home_deg[j] = float(data[j])
                self.get_logger().info(
                    f'[home_setter] Home lida de {HOME_POSE_FILE}')
        except Exception as exc:
            self.get_logger().warn(
                f'[home_setter] Falha ao ler home_pose.json ({exc}) '
                '— usando posição padrão.')

        summary = '  '.join(
            f'{j[-1]}={home_deg[j]:+.1f}°' for j in _ARM_JOINTS)
        self.get_logger().info(f'[home_setter] Posição alvo: {summary}')

        if not self._cli.wait_for_service(timeout_sec=15.0):
            self.get_logger().error(
                '[home_setter] /gazebo/set_model_configuration indisponível.')
            return False

        req = SetModelConfiguration.Request()
        req.model_name = 'cr10_covvi'
        req.urdf_param_name = ''
        req.joint_names = list(_ARM_JOINTS)
        req.joint_positions = [
            math.radians(home_deg[j]) for j in _ARM_JOINTS]

        future = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done():
            self.get_logger().error(
                '[home_setter] Timeout aguardando set_model_configuration.')
            return False

        resp = future.result()
        if resp.success:
            self.get_logger().info('[home_setter] Home aplicada no Gazebo.')
            return True

        self.get_logger().error(
            f'[home_setter] set_model_configuration falhou: '
            f'{resp.status_message}')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = GazeboHomeSetter()
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
