"""
Executor de grasp para célula de manufatura com esteira.

Máquina de estados por ciclo (acionado via serviço /cell/execute_grasp):
  IDLE → PICK → LIFT → PLACE → HOME

Grips determinísticos (sem ML):
  ball   → palm_grip    → Box 1
  cup    → claw_grip    → Box 2
  pencil → fingertip_grip → Box 3

Serviços expostos (não bloqueiam a GUI):
  /cell/execute_grasp  (std_srvs/Trigger) → inicia ciclo completo em thread
  /cell/go_home        (std_srvs/Trigger) → envia braço ao home

Publica:
  /cell/status   (std_msgs/String JSON) — estado e progresso do executor

Subscreve:
  /detected_objects  (vision_msgs/Detection2DArray) — classe do objeto atual
  /joint_states      (sensor_msgs/JointState)        — posição das juntas

Mão: 31 juntas (6 primárias + 25 mimic) com as razões de mimic do URDF COVVI.
Cinemática: IK analítica + refinamento numérico DLS do módulo kinematics.
"""

from __future__ import annotations

import json
import math
import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from vision_msgs.msg import Detection2DArray

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
_HOME_Q         = np.array([0.0, -math.pi / 4, math.pi / 2,
                             -math.pi / 4, -math.pi / 2, 0.0])
_APPROACH_CLEAR = 0.15    # m — altura de pré-abordagem
_LIFT_HEIGHT    = 0.22    # m — altura de levantamento pós-grasp
_CLOSE_EXTRA    = 0.05    # fração extra de fechamento sobre o cfg nominal
_MAX_JOINT_VEL  = 0.50    # rad/s
_N_TRAJ_STEPS   = 12      # waypoints por segmento de trajetória

# Approach vector padrão: de cima para baixo (esteira horizontal)
_AV_DOWN = np.array([0.0, 0.0, -1.0])

# Altura do base_link do robô no world frame do Gazebo (spawn z=0.375).
# Todas as posições world frame devem ter esse offset subtraído antes do IK,
# pois o módulo kinematics trabalha no frame da base do robô.
_ROBOT_BASE_Z = 0.375


def _w2r(pos: np.ndarray) -> np.ndarray:
    """World frame → robot base frame (subtrai altura da base no mundo)."""
    return np.array([pos[0], pos[1], pos[2] - _ROBOT_BASE_Z])

# Mapeamento classe → (grip_type, box_key, obj_diameter_m)
# frasco (frasco de medicamento) → palm_grip  → Box 1  (recipiente volumoso)
# tubo   (tubo de ensaio)        → claw_grip  → Box 2  (cilindro médio)
# ampola (ampola farmacêutica)   → fingertip_grip → Box 3 (objeto delicado fino)
_OBJECT_MAP: dict[str, tuple[str, str, float]] = {
    # terceiro elemento: "diâmetro efetivo de preensão" (não o diâmetro externo do objeto,
    # mas a zona de contato dos dedos — palm_grip vem de cima, não envolve o perímetro total)
    'frasco': ('palm_grip',      'box1', 0.060),
    'tubo':   ('claw_grip',      'box2', 0.024),
    'ampola': ('fingertip_grip', 'box3', 0.010),
}


# ── Helpers de trajetória ─────────────────────────────────────────────

def _make_smooth_arm_goal(q_start: np.ndarray,
                          q_end: np.ndarray) -> tuple[FollowJointTrajectory.Goal, float]:
    """Trajetória multi-ponto com ease-in/out sinusoidal."""
    max_delta = float(np.max(np.abs(q_end - q_start)))
    duration  = max(max_delta / _MAX_JOINT_VEL, 2.0)

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
    """Goal para as 31 juntas da mão (6 primárias + 25 mimic)."""
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
    """Adiciona margem de fechamento sobre a configuração de grasp."""
    return {j: float(min(cfg.get(j, 0.0) + _CLOSE_EXTRA * HAND_LIMITS[j],
                         HAND_LIMITS[j]))
            for j in _HAND_PRIMARY}


# ── Nó executor ──────────────────────────────────────────────────────

class GraspExecutorNode(Node):

    def __init__(self):
        super().__init__('grasp_executor')

        # Parâmetros do sistema (todos declarados antes do uso)
        self.declare_parameter('sim_only', True)
        self.declare_parameter('pick_x', 0.65)
        self.declare_parameter('pick_y', 0.00)
        self.declare_parameter('pick_z_frasco', 0.851)
        self.declare_parameter('pick_z_tubo',   0.866)
        self.declare_parameter('pick_z_ampola', 0.844)
        self.declare_parameter('box1_x', -0.05)
        self.declare_parameter('box1_y',  0.65)
        self.declare_parameter('box1_z',  0.60)
        self.declare_parameter('box2_x',  0.25)
        self.declare_parameter('box2_y',  0.65)
        self.declare_parameter('box2_z',  0.60)
        self.declare_parameter('box3_x',  0.55)
        self.declare_parameter('box3_y',  0.65)
        self.declare_parameter('box3_z',  0.60)

        self._pick_xy = np.array([
            self.get_parameter('pick_x').value,
            self.get_parameter('pick_y').value,
        ])
        self._pick_z: dict[str, float] = {
            'frasco': self.get_parameter('pick_z_frasco').value,
            'tubo':   self.get_parameter('pick_z_tubo').value,
            'ampola': self.get_parameter('pick_z_ampola').value,
        }
        self._boxes: dict[str, np.ndarray] = {
            'box1': np.array([self.get_parameter('box1_x').value,
                              self.get_parameter('box1_y').value,
                              self.get_parameter('box1_z').value]),
            'box2': np.array([self.get_parameter('box2_x').value,
                              self.get_parameter('box2_y').value,
                              self.get_parameter('box2_z').value]),
            'box3': np.array([self.get_parameter('box3_x').value,
                              self.get_parameter('box3_y').value,
                              self.get_parameter('box3_z').value]),
        }

        cb = ReentrantCallbackGroup()

        # Action clients para os controllers do braço e mão
        self._arm_ac = ActionClient(
            self, FollowJointTrajectory,
            '/cr10_group_controller/follow_joint_trajectory',
            callback_group=cb)
        self._hand_ac = ActionClient(
            self, FollowJointTrajectory,
            '/hand_position_controller/follow_joint_trajectory',
            callback_group=cb)

        # Cliente para retirar o objeto da pick station após grasp
        self._retreat_cli = self.create_client(
            Trigger, '/conveyor/retreat', callback_group=cb)

        # Serviços expostos à GUI (não bloqueantes — iniciam thread)
        self.create_service(Trigger, '/cell/execute_grasp',
                            self._cb_execute, callback_group=cb)
        self.create_service(Trigger, '/cell/go_home',
                            self._cb_home, callback_group=cb)

        # Estado interno
        self._current_q      = _HOME_Q.copy()
        self._last_detection: str | None = None
        self._last_pick_pos:  np.ndarray | None = None  # posição 3D da câmera
        self._busy           = False
        self._status_msg     = 'IDLE'

        # Subscriptions
        self.create_subscription(
            JointState, '/joint_states', self._cb_joint_state, 10,
            callback_group=cb)
        self.create_subscription(
            Detection2DArray, '/detected_objects', self._cb_detection, 10,
            callback_group=cb)

        self._pub_status = self.create_publisher(
            String, '/cell/status', 10)
        self.create_timer(0.5, self._tick_status)

        self.get_logger().info('GraspExecutor — aguardando action servers...')
        self._arm_ac.wait_for_server(timeout_sec=20.0)
        self._hand_ac.wait_for_server(timeout_sec=20.0)
        self.get_logger().info('GraspExecutor pronto.')

    # ──────────────────────────────────────────────────────────────────
    def _tick_status(self):
        self._pub_status.publish(String(data=json.dumps({
            'state': self._status_msg,
            'busy': self._busy,
            'last_obj': self._last_detection,
        })))

    def _cb_joint_state(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in _ARM_JOINTS:
                self._current_q[_ARM_JOINTS.index(name)] = msg.position[i]

    def _cb_detection(self, msg: Detection2DArray):
        if msg.detections:
            det  = msg.detections[0]
            hyp  = det.results[0]
            self._last_detection = hyp.hypothesis.class_id
            pos = hyp.pose.pose.position
            # Aceita posição 3D somente se o detector a estimou (z != 0)
            if abs(pos.z) > 1e-3:
                self._last_pick_pos = np.array([pos.x, pos.y, pos.z])
            else:
                self._last_pick_pos = None
        else:
            self._last_detection = None
            self._last_pick_pos  = None

    # ──────────────────────────────────────────────────────────────────
    def _cb_execute(self, _request, response: Trigger.Response):
        if self._busy:
            response.success = False
            response.message = 'Executor ocupado — aguarde o ciclo atual terminar.'
            return response

        obj_class = self._last_detection
        if obj_class is None or obj_class not in _OBJECT_MAP:
            response.success = False
            response.message = (
                f'Nenhum objeto válido detectado '
                f'(detectado: {self._last_detection!r}). '
                'Verifique a esteira antes de agarrar.')
            return response

        self._busy = True
        threading.Thread(
            target=self._run_cycle, args=(obj_class,), daemon=True).start()

        response.success = True
        response.message = f'Ciclo iniciado para: {obj_class}'
        return response

    def _cb_home(self, _request, response: Trigger.Response):
        if self._busy:
            response.success = False
            response.message = 'Executor ocupado.'
            return response
        self._busy = True
        threading.Thread(target=self._do_home, daemon=True).start()
        response.success = True
        response.message = 'Indo para home.'
        return response

    # ──────────────────────────────────────────────────────────────────
    def _run_cycle(self, obj_class: str):
        """Ciclo completo: Pick → Lift → Place → Home."""
        grip_type, box_key, obj_diam = _OBJECT_MAP[obj_class]

        # Posição de pick em world frame (câmera ou parâmetro fixo)
        if self._last_pick_pos is not None:
            pick_w = self._last_pick_pos.copy()
        else:
            pick_w = np.array([self._pick_xy[0], self._pick_xy[1],
                               self._pick_z[obj_class]])
        box_w = self._boxes[box_key]

        # Converter world → robot base frame antes de chamar o IK.
        # O IK calcula posições relativas ao base_link do robô, que está
        # em world z = _ROBOT_BASE_Z (0.375 m).
        p_pick = _w2r(pick_w)
        p_box  = _w2r(box_w)

        success = False
        self._status_msg = f'PICKING:{obj_class}'
        self.get_logger().info(
            f'[CICLO] {obj_class} → {grip_type} → {box_key} | '
            f'pick_robot={np.round(p_pick, 3).tolist()}  '
            f'box_robot={np.round(p_box, 3).tolist()}')

        try:
            # ── Calcular todas as poses IK em robot frame ──────────────
            approach_pick = p_pick + np.array([0.0, 0.0, _APPROACH_CLEAR])
            lift_pos      = p_pick + np.array([0.0, 0.0, _LIFT_HEIGHT])
            approach_box  = p_box  + np.array([0.0, 0.0, _APPROACH_CLEAR])

            q_ap,   ok1 = inverse_kinematics(approach_pick, _AV_DOWN, _HOME_Q)
            if not ok1:
                raise RuntimeError(f'IK abordagem pick falhou: {approach_pick}')
            q_pick, ok2 = inverse_kinematics(p_pick, _AV_DOWN, q_ap)
            if not ok2:
                raise RuntimeError(f'IK pick falhou: {p_pick}')
            q_lift, ok_lift = inverse_kinematics(lift_pos,     _AV_DOWN, q_pick)
            if not ok_lift:
                raise RuntimeError(f'IK lift falhou: {lift_pos}')
            q_ab,   ok3 = inverse_kinematics(approach_box,  _AV_DOWN, q_lift)
            if not ok3:
                raise RuntimeError(f'IK abordagem box falhou: {approach_box}')
            q_box,  ok4 = inverse_kinematics(p_box,         _AV_DOWN, q_ab)
            if not ok4:
                raise RuntimeError(f'IK box falhou: {p_box}')

            # ── Configurações da mão ───────────────────────────────────
            cfg_open   = HAND_CONFIGS['open']
            cfg_grasp  = hand_ik(grip_type, obj_diam)
            cfg_closed = _close_extra(cfg_grasp)

            # ── FASE 1: Abrir mão + ir para abordagem pick ─────────────
            self._status_msg = f'APPROACH_PICK:{obj_class}'
            self.get_logger().info('[F1] Abrindo mão + abordagem pick')
            self._send_hand(cfg_open, 2.0)
            self._send_arm(q_ap)
            time.sleep(0.3)

            # ── FASE 2: Pré-configurar dedos + descer ao objeto ─────────
            self.get_logger().info('[F2] Pré-grip + descida')
            self._send_hand(cfg_grasp, 1.5)
            self._send_arm(q_pick)
            time.sleep(0.3)

            # ── FASE 3: Fechar mão (grasp) ─────────────────────────────
            self._status_msg = f'GRASPING:{obj_class}'
            self.get_logger().info('[F3] Fechando mão — grasp')
            self._send_hand(cfg_closed, 1.5)
            time.sleep(0.5)

            self._call_retreat()

            # ── FASE 4: Levantar com objeto ────────────────────────────
            self._status_msg = f'LIFTING:{obj_class}'
            self.get_logger().info('[F4] Levantando')
            self._send_arm(q_lift)
            time.sleep(0.3)

            # ── FASE 5: Trânsito para a caixa ──────────────────────────
            self._status_msg = f'TRANSIT:{obj_class}→{box_key}'
            self.get_logger().info(f'[F5] Trânsito → {box_key}')
            self._send_arm(q_ab)
            time.sleep(0.3)

            # ── FASE 6: Descer na caixa e soltar ──────────────────────
            self._status_msg = f'PLACING:{obj_class}'
            self.get_logger().info(f'[F6] Descendo na caixa e soltando')
            self._send_arm(q_box)
            self._send_hand(cfg_open, 2.0)
            time.sleep(0.3)

            # ── FASE 7: Recuar + home ──────────────────────────────────
            self._status_msg = 'HOMING'
            self.get_logger().info('[F7] Recuando + home')
            self._send_arm(q_ab)
            self._do_home()
            success = True
            self.get_logger().info(f'[SUCESSO] {obj_class} ({grip_type}) → {box_key}')

        except Exception as exc:
            self.get_logger().error(f'[FALHA] {exc}')
            self._send_hand(HAND_CONFIGS['open'], 2.0)
            self._do_home()

        finally:
            self._status_msg = 'IDLE'
            self._busy = False
            self._pub_status.publish(String(data=json.dumps({
                'state': 'CYCLE_DONE',
                'object': obj_class,
                'success': success,
                'grip': grip_type,
                'box': box_key if success else 'none',
            })))

    # ──────────────────────────────────────────────────────────────────
    def _call_retreat(self):
        """Pede ao conveyor_controller para deletar o objeto da pick station."""
        if not self._retreat_cli.service_is_ready():
            self.get_logger().warn('[GRASP] /conveyor/retreat indisponível — objeto não removido.')
            return
        future = self._retreat_cli.call_async(Trigger.Request())
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 5.0:
            time.sleep(0.05)
        if future.done() and future.result() is not None:
            result = future.result()
            if result.success:
                self.get_logger().info('[GRASP] Objeto removido da pick station.')
            else:
                self.get_logger().warn(f'[GRASP] Retreat: {result.message}')

    # ──────────────────────────────────────────────────────────────────
    def _do_home(self):
        """Envia braço ao home e libera busy quando chamado via serviço direto."""
        self._send_arm(_HOME_Q)
        time.sleep(0.3)
        if self._busy and self._status_msg == 'HOMING':
            self._status_msg = 'IDLE'
            self._busy = False

    # ──────────────────────────────────────────────────────────────────
    def _send_arm(self, q: np.ndarray):
        """Envia trajetória ao braço e bloqueia até o goal ser concluído."""
        goal, _ = _make_smooth_arm_goal(self._current_q, q)
        self._arm_ac.send_goal(goal)  # blocking in this rclpy version
        self._current_q = q.copy()

    def _send_hand(self, cfg: dict[str, float], duration: float):
        """Envia posição à mão e bloqueia até o goal ser concluído."""
        self._hand_ac.send_goal(_make_hand_goal(cfg, duration))  # blocking


def main(args=None):
    rclpy.init(args=args)
    node = GraspExecutorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
