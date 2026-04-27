"""
Nó executor de grasp — usa o módulo kinematics.py para IK completo.

Pipeline de execução:
  1. Abrir mão (configuração 'open')
  2. Mover braço → pose de pré-abordagem (15 cm acima do alvo)
  3. Pré-configurar dedos (hand_ik por tipo + diâmetro)
  4. Descer → pose de preensão (descida suave 5 cm/passo)
  5. Fechar mão (+15% closure para compensar compliance)
  6. Levantar objeto

Tópicos:
  /selected_grasp  (String JSON) → entrada
  /grasp_result    (String JSON) → saída
  /cr10_group_controller/follow_joint_trajectory  → action CR10
  /hand_position_controller/follow_joint_trajectory → action COVVI
"""

from __future__ import annotations

import json
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration

from .kinematics import (
    inverse_kinematics, forward_kinematics,
    HAND_CONFIGS, HAND_LIMITS, hand_ik,
    JOINT_MIN, JOINT_MAX,
)

# Juntas primárias da mão COVVI (ordem para o controlador)
_HAND_JOINTS = ['Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Rotate']

# Juntas do CR10 (ordem para o controlador)
_ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# Pose de home segura (rad) — cotovelo-acima, braço vertical
_HOME_Q = np.array([0.0, -math.pi / 4, math.pi / 2, -math.pi / 4, -math.pi / 2, 0.0])

# Diâmetros dos objetos para IK da mão (m)
_OBJ_DIAMETERS = {'pencil': 0.007, 'cup': 0.070, 'ball': 0.064}

# Altura de pré-abordagem acima do objeto (m)
_APPROACH_CLEARANCE = 0.15

# Incremento extra de fechamento na fase de preensão
_CLOSE_EXTRA = 0.15


def _make_arm_goal(q: np.ndarray, duration: float) -> FollowJointTrajectory.Goal:
    traj = JointTrajectory()
    traj.joint_names = _ARM_JOINTS
    pt = JointTrajectoryPoint()
    pt.positions = [float(a) for a in q]
    sec = int(duration)
    ns  = int((duration - sec) * 1e9)
    pt.time_from_start = Duration(sec=sec, nanosec=ns)
    traj.points.append(pt)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    return goal


def _make_hand_goal(cfg: dict[str, float], duration: float) -> FollowJointTrajectory.Goal:
    traj = JointTrajectory()
    traj.joint_names = _HAND_JOINTS
    pt = JointTrajectoryPoint()
    pt.positions = [float(cfg.get(j, 0.0)) for j in _HAND_JOINTS]
    sec = int(duration)
    ns  = int((duration - sec) * 1e9)
    pt.time_from_start = Duration(sec=sec, nanosec=ns)
    traj.points.append(pt)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    return goal


def _close_extra(cfg: dict[str, float]) -> dict[str, float]:
    """Aumenta fechamento em _CLOSE_EXTRA vezes o limite máximo."""
    result = {}
    for j in _HAND_JOINTS:
        result[j] = float(min(cfg.get(j, 0.0) + _CLOSE_EXTRA * HAND_LIMITS[j],
                              HAND_LIMITS[j]))
    return result


class GraspExecutorNode(Node):

    def __init__(self):
        super().__init__('grasp_executor')

        self.declare_parameter('simulation_mode', True)
        self._sim = self.get_parameter('simulation_mode').value

        self._arm_ac = ActionClient(
            self, FollowJointTrajectory,
            '/cr10_group_controller/follow_joint_trajectory')
        self._hand_ac = ActionClient(
            self, FollowJointTrajectory,
            '/hand_position_controller/follow_joint_trajectory')

        self._sub = self.create_subscription(
            String, '/selected_grasp', self._cb_grasp, 10)
        self._pub_result = self.create_publisher(String, '/grasp_result', 10)

        self._busy = False
        self._pending: dict | None = None

        self.get_logger().info('GraspExecutor — aguardando action servers...')
        self._arm_ac.wait_for_server(timeout_sec=15.0)
        self._hand_ac.wait_for_server(timeout_sec=15.0)
        self.get_logger().info('Action servers conectados.')

    # ──────────────────────────────────────────────────────────────────
    def _cb_grasp(self, msg: String):
        if self._busy:
            self.get_logger().warn('Já executando — comando ignorado.')
            return
        self._pending = json.loads(msg.data)
        self._busy = True
        # Timer de disparo único para não bloquear o spin
        self.create_timer(0.05, self._run_pipeline)

    # ──────────────────────────────────────────────────────────────────
    def _run_pipeline(self):
        grasp  = self._pending
        label  = grasp['object_label']
        gtype  = grasp['grasp_type']
        obj_pos = np.array(grasp['object_position'], dtype=float)
        av      = np.array(grasp['approach_vector'],  dtype=float)
        av      = av / (np.linalg.norm(av) + 1e-9)
        offset  = np.array(grasp.get('grasp_offset', [0.0, 0.0, 0.0]), dtype=float)
        diam    = _OBJ_DIAMETERS.get(label, 0.04)

        success = False
        try:
            # ── 1. Calcular poses do braço via IK ──────────────────────
            grasp_pos  = obj_pos + offset
            approach_pos = grasp_pos - av * _APPROACH_CLEARANCE

            q_approach, ok1 = inverse_kinematics(approach_pos, av, _HOME_Q)
            q_grasp,    ok2 = inverse_kinematics(grasp_pos,    av, q_approach)
            if not ok1:
                raise RuntimeError(f'IK falhou para pose de abordagem em {approach_pos}')
            if not ok2:
                raise RuntimeError(f'IK falhou para pose de preensão em {grasp_pos}')

            # Pose de levantamento: 15 cm acima da pose de preensão
            lift_pos = grasp_pos + np.array([0.0, 0.0, 0.15])
            q_lift, _ = inverse_kinematics(lift_pos, av, q_grasp)

            # ── 2. Calcular configurações da mão via hand_ik ───────────
            cfg_open    = HAND_CONFIGS['open']
            cfg_grasp   = hand_ik(gtype, diam)
            cfg_closed  = _close_extra(cfg_grasp)

            # ── 3. Executar pipeline ───────────────────────────────────
            self.get_logger().info(
                f'[{label}] {gtype} | approach OK={ok1} grasp OK={ok2}')

            # Abrir mão
            self._send_hand(cfg_open, 1.5);            time.sleep(2.0)

            # Ir para pré-abordagem
            self._send_arm(q_approach, 4.0);           time.sleep(4.5)

            # Pré-configurar dedos
            self._send_hand(cfg_grasp, 1.0);           time.sleep(1.5)

            # Descer para pose de preensão (2 passos intermediários)
            mid_pos = 0.5 * (approach_pos + grasp_pos)
            q_mid, _ = inverse_kinematics(mid_pos, av, q_approach)
            self._send_arm(q_mid,   2.0);              time.sleep(2.2)
            self._send_arm(q_grasp, 1.5);              time.sleep(1.8)

            # Fechar mão
            self._send_hand(cfg_closed, 1.0);          time.sleep(1.5)

            # Levantar
            self._send_arm(q_lift, 2.0);               time.sleep(2.5)

            success = True
            self.get_logger().info(f'[SUCESSO] {label} ({gtype})')

        except Exception as exc:
            self.get_logger().error(f'Falha na execução: {exc}')
            self._send_hand(HAND_CONFIGS['open'], 1.0)
            time.sleep(1.2)
            self._send_arm(_HOME_Q, 3.0)
            time.sleep(3.5)

        finally:
            result = {
                'object_label': label,
                'grasp_type':   gtype,
                'success':      success,
                'score':        grasp.get('score', 0.0),
            }
            self._pub_result.publish(String(data=json.dumps(result)))
            self._busy = False

    # ──────────────────────────────────────────────────────────────────
    def _send_arm(self, q: np.ndarray, duration: float):
        goal = _make_arm_goal(q, duration)
        self._arm_ac.send_goal(goal)

    def _send_hand(self, cfg: dict[str, float], duration: float):
        goal = _make_hand_goal(cfg, duration)
        self._hand_ac.send_goal(goal)


def main(args=None):
    rclpy.init(args=args)
    node = GraspExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
