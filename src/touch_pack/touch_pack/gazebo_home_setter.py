"""
gazebo_home_setter.py — Sincroniza o Gazebo com a posição atual do robô real.

Prioridade (braço):
  1. Robô real conectado → lê juntas via read_joints_urdf() e aplica no Gazebo.
  2. Robô indisponível   → usa ~/.config/touch_pack/home_pose.json (home salva).
  3. Sem home_pose.json  → usa posição padrão [0, 0, -90°, 0, 90°, 0].

Modo hand (end_effector=hand):
  Também define as juntas da mão COVVI na posição de repouso natural
  (HAND_DRIVER_LOWER + mimic joints correspondentes), evitando que o ODE
  aplique forças de limite-joint nas juntas que partem de 0 mas têm lower > 0.

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

# Posição de repouso das juntas primárias da mão (rad) — espelha
# HAND_DRIVER_LOWER de hand_pack.urdf_helpers. Definido aqui para evitar
# dependência de importação em runtime (que pode falhar se hand_pack não
# estiver no PYTHONPATH no momento do launch).
_HAND_PRIMARY_LOWER = {
    'Thumb':  0.08,
    'Index':  0.12,
    'Middle': 0.12,
    'Ring':   0.12,
    'Little': 0.12,
    'Rotate': 0.00,
}

try:
    from .real_driver import CR10RealDriver, CR10RealDriverConfig, CR10RealDriverError
    _DRIVER_OK = True
except Exception:
    CR10RealDriver = None
    CR10RealDriverConfig = None
    CR10RealDriverError = Exception
    _DRIVER_OK = False

try:
    from .kinematics import MIMIC_LIST as _MIMIC_LIST
    _MIMIC_OK = True
except Exception:
    _MIMIC_LIST = []
    _MIMIC_OK = False


def _hand_initial_joints() -> tuple[list[str], list[float]]:
    """Devolve (joint_names, positions_rad) para a mão na posição de repouso.

    Inclui juntas primárias (HAND_DRIVER_LOWER) e mimic joints
    (multiplier × lower do driver). Garante que todas as juntas partem
    acima do lower limit — sem forças ODE de limite no startup.
    """
    names: list[str]  = []
    positions: list[float] = []

    # Juntas primárias
    for jname, lower in _HAND_PRIMARY_LOWER.items():
        names.append(jname)
        positions.append(lower)

    # Mimic joints: multiplier × lower do driver
    for mimic_name, driver, mult in _MIMIC_LIST:
        driver_lower = _HAND_PRIMARY_LOWER.get(driver, 0.0)
        names.append(mimic_name)
        positions.append(mult * driver_lower)

    return names, positions


class GazeboHomeSetter(Node):
    def __init__(self):
        super().__init__('gazebo_home_setter')
        self.declare_parameter('robot_ip', '')
        self.declare_parameter('end_effector', 'touch_tool')
        self._cli = self.create_client(
            SetModelConfiguration, '/gazebo/set_model_configuration')

    def _robot_ip(self) -> str:
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
        """Conecta ao robô real e devolve as 6 juntas em rad (URDF)."""
        if not _DRIVER_OK or CR10RealDriver is None:
            return None
        cfg = CR10RealDriverConfig(readonly=True) if CR10RealDriverConfig else None
        drv = CR10RealDriver(ip=ip, config=cfg)
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

    def _set_configuration(self, joint_names: list[str],
                           joint_positions: list[float]) -> bool:
        req = SetModelConfiguration.Request()
        req.model_name = 'cr10_tcp'
        req.urdf_param_name = ''
        req.joint_names = joint_names
        req.joint_positions = joint_positions

        future = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done():
            self.get_logger().error(
                '[home_setter] Timeout aguardando set_model_configuration.')
            return False

        resp = future.result()
        if not resp.success:
            self.get_logger().error(
                f'[home_setter] set_model_configuration falhou: '
                f'{resp.status_message}')
        return resp.success

    def run(self) -> bool:
        ip = self._robot_ip()
        end_effector = self.get_parameter('end_effector').value.strip().lower()

        arm_rad = self._read_robot_joints_urdf(ip)
        if arm_rad is None:
            arm_rad = self._home_from_file()

        if not self._cli.wait_for_service(timeout_sec=60.0):
            self.get_logger().error(
                '[home_setter] /gazebo/set_model_configuration indisponível.')
            return False

        # Braço
        joint_names = list(_ARM_JOINTS)
        joint_positions = list(arm_rad)

        # Mão: inclui juntas primárias + mimic na posição de repouso natural.
        # Sem isso, as juntas partem de 0 mas têm lower > 0 → ODE aplica
        # forças de limite e a mão "move sozinha" ao spawnar.
        if end_effector == 'hand':
            h_names, h_pos = _hand_initial_joints()
            joint_names.extend(h_names)
            joint_positions.extend(h_pos)

        ok = self._set_configuration(joint_names, joint_positions)
        if ok:
            self.get_logger().info('[home_setter] Posição inicial aplicada no Gazebo.')
        return ok


def main(args=None):
    rclpy.init(args=args)
    node = GazeboHomeSetter()
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
