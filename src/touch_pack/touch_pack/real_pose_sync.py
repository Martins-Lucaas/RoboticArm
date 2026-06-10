"""
real_pose_sync.py — Sincroniza o Gazebo com a posição atual do robô real.

Substitui o antigo gazebo_home_setter: o serviço /gazebo/set_model_configuration
do Gazebo Classic nunca foi portado para o ROS 2, então aquele nó queimava 60 s
de timeout e falhava sempre. Aqui a pose real é aplicada via
FollowJointTrajectory no cr10_group_controller já ativo — o braço simulado
move-se até a pose real em ~3 s, sem bloquear o resto do launch.

Fluxo:
  1. Lê as juntas do robô real (CR10RealDriver readonly; connect timeout 3 s).
  2. Robô indisponível → encerra sem erro; o Gazebo permanece na pose inicial
     do URDF (definida pelos <param name="initial_value"> do ros2_control).
  3. Robô disponível → envia trajetória de 3 s ao cr10_group_controller e
     aguarda o resultado.

Encerra sozinho após a sincronização (uso único no launch).
"""
from __future__ import annotations

import json
import math
import os
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ROBOT_CONFIG_FILE = os.path.expanduser('~/.config/touch_pack/robot.json')
_ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
_CONTROLLER_ACTION = '/cr10_group_controller/follow_joint_trajectory'
_MOVE_DURATION_S = 3.0

try:
    from .real_driver import CR10RealDriver, CR10RealDriverConfig
    _DRIVER_OK = True
except Exception:
    CR10RealDriver = None
    CR10RealDriverConfig = None
    _DRIVER_OK = False


class RealPoseSync(Node):
    def __init__(self):
        super().__init__('real_pose_sync')
        self.declare_parameter('robot_ip', '')
        self._client = ActionClient(
            self, FollowJointTrajectory, _CONTROLLER_ACTION)

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
                f'[pose_sync] Falha ao ler robot.json: {exc}')
        return '192.168.5.2'

    def _read_robot_joints_urdf(self, ip: str) -> list[float] | None:
        """Conecta ao robô real e devolve as 6 juntas em rad (URDF)."""
        if not _DRIVER_OK or CR10RealDriver is None:
            self.get_logger().warn(
                '[pose_sync] real_driver indisponível — sync ignorado.')
            return None
        cfg = CR10RealDriverConfig(readonly=True) if CR10RealDriverConfig else None
        drv = CR10RealDriver(ip=ip, config=cfg)
        try:
            drv.connect()
            q = drv.read_joints_urdf()
            return [float(v) for v in q]
        except Exception as exc:
            self.get_logger().info(
                f'[pose_sync] Robô real em {ip} indisponível ({exc}) — '
                'Gazebo permanece na pose inicial do URDF.')
            return None
        finally:
            try:
                drv.close()
            except Exception:
                pass

    def _move_sim_to(self, arm_rad: list[float]) -> bool:
        if not self._client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error(
                f'[pose_sync] Action {_CONTROLLER_ACTION} indisponível.')
            return False

        traj = JointTrajectory()
        traj.joint_names = list(_ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = list(arm_rad)
        pt.velocities = [0.0] * len(_ARM_JOINTS)
        pt.time_from_start = Duration(sec=int(_MOVE_DURATION_S))
        traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        goal_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=10.0)
        handle = goal_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('[pose_sync] Goal rejeitado pelo controller.')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=_MOVE_DURATION_S + 15.0)
        if not result_future.done():
            self.get_logger().error('[pose_sync] Timeout aguardando trajetória.')
            return False

        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f'[pose_sync] Trajetória falhou: {result.error_string}')
            return False
        return True

    def run(self) -> bool:
        ip = self._robot_ip()
        arm_rad = self._read_robot_joints_urdf(ip)
        if arm_rad is None:
            return True  # robô off não é erro — pose do URDF já vale

        deg = '  '.join(
            f'j{i + 1}={math.degrees(v):+.1f}°' for i, v in enumerate(arm_rad))
        self.get_logger().info(
            f'[pose_sync] Pose real lida de {ip}: {deg} — movendo Gazebo.')

        ok = self._move_sim_to(arm_rad)
        if ok:
            self.get_logger().info(
                '[pose_sync] Gazebo sincronizado com a pose real do braço.')
        return ok


def main(args=None):
    rclpy.init(args=args)
    node = RealPoseSync()
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
