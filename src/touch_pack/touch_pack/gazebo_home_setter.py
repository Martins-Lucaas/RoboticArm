"""
gazebo_home_setter.py — Sincroniza o Gazebo com a posição atual do robô real.

Prioridade:
  1. Robô real conectado → lê juntas via read_joints_urdf() e aplica no Gazebo.
  2. Robô indisponível   → usa ~/.config/touch_pack/home_pose.json (home salva).
  3. Sem home_pose.json  → usa posição padrão [0, 0, -90°, 0, 90°, 0].

Encerra sozinho após a chamada (uso único no launch).
"""
from __future__ import annotations

import json
import math
import os
import sys

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetModelConfiguration

ROBOT_CONFIG_FILE = os.path.expanduser('~/.config/touch_pack/robot.json')
HOME_POSE_FILE    = os.path.expanduser('~/.config/touch_pack/home_pose.json')
_ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
_ARM_HOME_DEG_DEFAULT = {
    'joint1':  0.0, 'joint2':  0.0, 'joint3': -90.0,
    'joint4':  0.0, 'joint5': 90.0, 'joint6':   0.0,
}

try:
    from .real_driver import CR10RealDriver, CR10RealDriverError
    _DRIVER_OK = True
except Exception:
    CR10RealDriver = None
    CR10RealDriverError = Exception
    _DRIVER_OK = False


class GazeboHomeSetter(Node):
    def __init__(self):
        super().__init__('gazebo_home_setter')
        self.declare_parameter('robot_ip', '')
        self._cli = self.create_client(
            SetModelConfiguration, '/gazebo/set_model_configuration')

    def _robot_ip(self) -> str:
        # ROS parameter tem prioridade (permite sobrescrever via launch/CLI).
        param_ip = self.get_parameter('robot_ip').value.strip()
        if param_ip:
            return param_ip
        try:
            if os.path.exists(ROBOT_CONFIG_FILE):
                with open(ROBOT_CONFIG_FILE) as fh:
                    data = json.load(fh)
                ip = data.get('robot_ip', '').strip()
                if ip:
                    return ip
        except Exception as exc:
            self.get_logger().warn(
                f'[home_setter] Falha ao ler robot.json: {exc}')
        return '192.168.5.2'

    def _read_robot_joints_urdf(self, ip: str) -> list[float] | None:
        """Conecta ao robô real e devolve as 6 juntas em rad (URDF).
        Retorna None se o robô não estiver disponível."""
        if not _DRIVER_OK or CR10RealDriver is None:
            return None
        drv = CR10RealDriver(ip=ip)
        try:
            drv.connect()
            q = drv.read_joints_urdf()
            return [float(v) for v in q]
        except Exception as exc:
            self.get_logger().warn(
                f'[home_setter] Robô real em {ip} indisponível: {exc}')
            return None
        finally:
            try:
                drv.close()
            except Exception:
                pass

    def _home_from_file(self) -> list[float]:
        """Lê home_pose.json; retorna posição padrão se ausente/inválido."""
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
        return [math.radians(home_deg[j]) for j in _ARM_JOINTS]

    def run(self) -> bool:
        ip = self._robot_ip()
        joint_rad = self._read_robot_joints_urdf(ip)

        if joint_rad is not None:
            source = f'robô real ({ip})'
        else:
            joint_rad = self._home_from_file()
            source = 'home_pose.json'

        summary = '  '.join(
            f'{j[-1]}={math.degrees(r):+.1f}°'
            for j, r in zip(_ARM_JOINTS, joint_rad))
        self.get_logger().info(
            f'[home_setter] Posição ({source}): {summary}')

        if not self._cli.wait_for_service(timeout_sec=15.0):
            self.get_logger().error(
                '[home_setter] /gazebo/set_model_configuration indisponível.')
            return False

        req = SetModelConfiguration.Request()
        req.model_name = 'cr10_tcp'
        req.urdf_param_name = ''
        req.joint_names = list(_ARM_JOINTS)
        req.joint_positions = joint_rad

        future = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done():
            self.get_logger().error(
                '[home_setter] Timeout aguardando set_model_configuration.')
            return False

        resp = future.result()
        if resp.success:
            self.get_logger().info(
                '[home_setter] Posição real aplicada no Gazebo.')
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
