"""
Executor de grasp: pick → lift → giro de pulso → place back → home.

Pipeline completo (por objeto):
  1. Abrir mão
  2. Abordagem suave (15 cm acima)
  3. Pré-configurar dedos
  4. Descida em 2 passos
  5. Fechar mão
  6. Levantar (20 cm)
  7. Girar pulso (joint6 ±135°)
  8. Descer de volta à pose de preensão
  9. Abrir mão (soltar)
  10. Recuar para pré-abordagem
  11. Retornar ao home

Trajetória: ease-in/out sinusoidal com 10 waypoints por segmento.
Mão: 31 juntas (6 primárias + 25 mimic) — evita explosão física no Gazebo.
"""

from __future__ import annotations

import json
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration

from .kinematics import (
    inverse_kinematics,
    HAND_CONFIGS, HAND_LIMITS, hand_ik,
    JOINT_MIN, JOINT_MAX,
)

# ── Juntas do braço CR10 ──────────────────────────────────────────────
_ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# ── Juntas primárias da mão ───────────────────────────────────────────
_HAND_PRIMARY = ['Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Rotate']

# ── Mapa de mimic joints: nome → (junta primária, multiplicador) ──────
# Extraído do URDF linear_covvi_hand_gazebo.urdf
_MIMIC_MAP: dict[str, tuple[str, float]] = {
    '_lisa_j01':            ('Rotate', 1.07338),
    '_thumb_chassis_j01':   ('Rotate', 1.53340),
    '_thumb_proximal_j01':  ('Thumb',  0.72022),
    '_thumb_distal_j01':    ('Thumb',  1.06686),
    '_thumb_link_j01':      ('Thumb',  0.76799),
    '_thumb_follower_j01':  ('Thumb',  0.93733),
    '_index_proximal_j01':  ('Index',  1.51604),
    '_index_distal_j01':    ('Index',  1.33574),
    '_index_knuckle_j01':   ('Index',  1.25182),
    '_index_follower_j01':  ('Index',  0.26423),
    '_index_link_j01':      ('Index',  1.33574),
    '_middle_proximal_j01': ('Middle', 1.51604),
    '_middle_distal_j01':   ('Middle', 1.34986),
    '_middle_knuckle_j01':  ('Middle', 1.25181),
    '_middle_follower_j01': ('Middle', 0.26423),
    '_middle_link_j01':     ('Middle', 1.34986),
    '_ring_proximal_j01':   ('Ring',   1.51604),
    '_ring_distal_j01':     ('Ring',   1.34878),
    '_ring_knuckle_j01':    ('Ring',   1.25182),
    '_ring_follower_j01':   ('Ring',   0.26423),
    '_ring_link_j01':       ('Ring',   1.34878),
    '_little_proximal_j01': ('Little', 1.51604),
    '_little_distal_j01':   ('Little', 1.31664),
    '_little_knuckle_j01':  ('Little', 1.25182),
    '_little_follower_j01': ('Little', 0.26423),
    '_little_link_j01':     ('Little', 1.31664),
}

# ── Parâmetros de movimento ───────────────────────────────────────────
_HOME_Q           = np.array([0.0, -math.pi/4, math.pi/2,
                               -math.pi/4, -math.pi/2, 0.0])
_OBJ_DIAMETERS    = {'pencil': 0.007, 'cup': 0.070, 'ball': 0.064}
_APPROACH_CLEAR   = 0.15    # m — altura de pré-abordagem
_LIFT_HEIGHT      = 0.20    # m — altura de levantamento
_CLOSE_EXTRA      = 0.15    # fração extra de fechamento para compliance
_MAX_JOINT_VEL    = 0.15    # rad/s — velocidade lenta para visualização
_N_TRAJ_STEPS     = 12      # waypoints por segmento
_WRIST_ROTATION   = math.pi * 0.75   # 135° de giro de pulso


# ── Helpers de trajetória ─────────────────────────────────────────────

def _make_smooth_arm_goal(q_start: np.ndarray,
                          q_end: np.ndarray) -> tuple[FollowJointTrajectory.Goal, float]:
    """Trajetória multi-ponto com ease-in/out sinusoidal.
    Duração calculada pelo maior ∆θ / velocidade máxima."""
    max_delta = float(np.max(np.abs(q_end - q_start)))
    duration  = max(max_delta / _MAX_JOINT_VEL, 5.0)

    traj = JointTrajectory()
    traj.joint_names = _ARM_JOINTS

    for i in range(1, _N_TRAJ_STEPS + 1):
        alpha  = i / _N_TRAJ_STEPS
        smooth = 0.5 * (1.0 - math.cos(math.pi * alpha))
        q_i    = q_start + smooth * (q_end - q_start)
        t      = duration * alpha
        pt     = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q_i]
        if i == _N_TRAJ_STEPS:
            pt.velocities    = [0.0] * 6
            pt.accelerations = [0.0] * 6
        sec = int(t)
        pt.time_from_start = Duration(sec=sec, nanosec=int((t - sec) * 1e9))
        traj.points.append(pt)

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    return goal, duration


def _make_hand_goal(cfg: dict[str, float],
                    duration: float) -> FollowJointTrajectory.Goal:
    """Goal para todas as 31 juntas da mão (6 primárias + 25 mimic).
    Mimic positions = primary_pos × multiplier."""
    all_names: list[str] = list(_HAND_PRIMARY) + list(_MIMIC_MAP.keys())
    positions: list[float] = []

    for j in _HAND_PRIMARY:
        positions.append(float(cfg.get(j, 0.0)))
    for j, (primary, mult) in _MIMIC_MAP.items():
        positions.append(float(cfg.get(primary, 0.0) * mult))

    pt = JointTrajectoryPoint()
    pt.positions = positions
    sec = int(duration)
    pt.time_from_start = Duration(sec=sec, nanosec=int((duration - sec) * 1e9))

    traj = JointTrajectory()
    traj.joint_names = all_names
    traj.points.append(pt)

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    return goal


def _close_extra(cfg: dict[str, float]) -> dict[str, float]:
    return {j: float(min(cfg.get(j, 0.0) + _CLOSE_EXTRA * HAND_LIMITS[j],
                         HAND_LIMITS[j]))
            for j in _HAND_PRIMARY}


# ── Nó executor ──────────────────────────────────────────────────────

class GraspExecutorNode(Node):

    def __init__(self):
        super().__init__('grasp_executor')
        self.declare_parameter('simulation_mode', True)

        self._arm_ac = ActionClient(
            self, FollowJointTrajectory,
            '/cr10_group_controller/follow_joint_trajectory')
        self._hand_ac = ActionClient(
            self, FollowJointTrajectory,
            '/hand_position_controller/follow_joint_trajectory')

        self._sub = self.create_subscription(
            String, '/selected_grasp', self._cb_grasp, 10)
        self._pub_result = self.create_publisher(String, '/grasp_result', 10)

        # Posição atual das juntas (atualizada via /joint_states)
        self._current_q   = _HOME_Q.copy()
        self._sub_js = self.create_subscription(
            JointState, '/joint_states', self._cb_joint_state, 10)

        self._busy   = False
        self._pending: dict | None = None

        self.get_logger().info('GraspExecutor — aguardando action servers...')
        self._arm_ac.wait_for_server(timeout_sec=20.0)
        self._hand_ac.wait_for_server(timeout_sec=20.0)
        self.get_logger().info('Action servers prontos.')

    # ──────────────────────────────────────────────────────────────────
    def _cb_joint_state(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in _ARM_JOINTS:
                self._current_q[_ARM_JOINTS.index(name)] = msg.position[i]

    def _cb_grasp(self, msg: String):
        if self._busy:
            self.get_logger().warn('Já executando — ignorado.')
            return
        self._pending = json.loads(msg.data)
        self._busy    = True
        self.create_timer(0.05, self._run_pipeline)

    # ──────────────────────────────────────────────────────────────────
    def _run_pipeline(self):
        grasp    = self._pending
        label    = grasp['object_label']
        gtype    = grasp['grasp_type']
        obj_pos  = np.array(grasp['object_position'], dtype=float)
        av       = np.array(grasp['approach_vector'],  dtype=float)
        av      /= (np.linalg.norm(av) + 1e-9)
        offset   = np.array(grasp.get('grasp_offset', [0.0, 0.0, 0.0]), dtype=float)
        diam     = _OBJ_DIAMETERS.get(label, 0.04)

        success = False
        try:
            # ── 1. IK para todas as poses ─────────────────────────────
            grasp_pos    = obj_pos + offset
            approach_pos = grasp_pos - av * _APPROACH_CLEAR
            lift_pos     = grasp_pos + np.array([0.0, 0.0, _LIFT_HEIGHT])
            mid_pos      = 0.5 * (approach_pos + grasp_pos)

            q_approach, ok1 = inverse_kinematics(approach_pos, av, _HOME_Q)
            q_grasp,    ok2 = inverse_kinematics(grasp_pos,    av, q_approach)
            if not ok1:
                raise RuntimeError(f'IK abordagem falhou em {approach_pos}')
            if not ok2:
                raise RuntimeError(f'IK preensão falhou em {grasp_pos}')

            q_mid,  _  = inverse_kinematics(mid_pos,  av, q_approach)
            q_lift, _  = inverse_kinematics(lift_pos, av, q_grasp)

            # Giro de pulso: joint6 += ±_WRIST_ROTATION (dentro dos limites)
            q_rotated    = q_lift.copy()
            j6_new       = np.clip(q_lift[5] + _WRIST_ROTATION,
                                   JOINT_MIN[5], JOINT_MAX[5])
            if abs(j6_new - q_lift[5]) < _WRIST_ROTATION * 0.5:
                j6_new = np.clip(q_lift[5] - _WRIST_ROTATION,
                                 JOINT_MIN[5], JOINT_MAX[5])
            q_rotated[5] = j6_new
            rot_deg      = math.degrees(j6_new - q_lift[5])

            # ── 2. Configurações da mão ───────────────────────────────
            cfg_open   = HAND_CONFIGS['open']
            cfg_grasp  = hand_ik(gtype, diam)
            cfg_closed = _close_extra(cfg_grasp)

            # ── 3. PICK ───────────────────────────────────────────────
            self.get_logger().info(f'[{label}] {gtype} pick-rotate-place | '
                                   f'giro={rot_deg:.0f}°')
            self._send_hand(cfg_open, 2.0);    time.sleep(3.0)

            dur = self._send_arm(q_approach);  time.sleep(dur + 1.5)
            self._send_hand(cfg_grasp, 1.5);   time.sleep(2.5)

            dur = self._send_arm(q_mid);       time.sleep(dur + 1.0)
            dur = self._send_arm(q_grasp);     time.sleep(dur + 1.0)
            self._send_hand(cfg_closed, 1.5);  time.sleep(3.0)

            # ── 4. LIFT ───────────────────────────────────────────────
            dur = self._send_arm(q_lift);      time.sleep(dur + 1.5)

            # ── 5. GIRO DE PULSO ──────────────────────────────────────
            dur = self._send_arm(q_rotated);   time.sleep(dur + 2.0)

            # ── 6. PLACE BACK (retorna à pose de preensão) ────────────
            dur = self._send_arm(q_grasp);     time.sleep(dur + 1.5)
            self._send_hand(cfg_open, 2.0);    time.sleep(3.5)  # solta

            # ── 7. RECUO → HOME ───────────────────────────────────────
            dur = self._send_arm(q_approach);  time.sleep(dur + 1.5)
            dur = self._send_arm(_HOME_Q);     time.sleep(dur + 1.5)

            success = True
            self.get_logger().info(f'[SUCESSO] {label} ({gtype})')

        except Exception as exc:
            self.get_logger().error(f'Falha: {exc}')
            self._send_hand(HAND_CONFIGS['open'], 2.0)
            time.sleep(2.5)
            dur = self._send_arm(_HOME_Q)
            time.sleep(dur + 1.5)

        finally:
            self._pub_result.publish(String(data=json.dumps({
                'object_label': label,
                'grasp_type':   gtype,
                'success':      success,
                'score':        grasp.get('score', 0.0),
            })))
            self._busy = False

    # ──────────────────────────────────────────────────────────────────
    def _send_arm(self, q: np.ndarray, _=None) -> float:
        goal, duration = _make_smooth_arm_goal(self._current_q, q)
        self._arm_ac.send_goal(goal)
        self._current_q = q.copy()
        return duration

    def _send_hand(self, cfg: dict[str, float], duration: float):
        self._hand_ac.send_goal(_make_hand_goal(cfg, duration))


def main(args=None):
    rclpy.init(args=args)
    node = GraspExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
