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
    forward_kinematics, fk_partial,
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
# Home seguro — braço erguido para trás (TCP ≈ (−0.69, −0.19, 1.31) m robot frame).
# q2=0 URDF ↔ braço superior vertical; q3=π/2 dobra o antebraço para trás.
# Validado: nenhum link colide com belt_frame, belt_guides, sort_shelf ou caixas.
_HOME_Q         = np.array([0.0,  0.0,  math.pi/2,
                             -math.pi/2, -math.pi/2, 0.0])
# Seeds compactos por objeto para IK de pick (pick_xy=(0.75,0)).
# q_pick é calculado primeiro com o seed do objeto; approach/via/lift seeded
# a partir de q_pick. frasco/ampola usam ramo q4<0; tubo usa o único ramo estável.
_PICK_SEED_Q = {
    'frasco': np.array([0.411, -0.277, -1.335, -1.529,  0.411,  0.0  ]),
    'tubo':   np.array([0.411, -0.381, -1.652,  2.032, -0.411,  3.142]),
    'ampola': np.array([0.411, -0.277, -1.341, -1.524,  0.411,  0.0  ]),
}
# Seed compacto para IK de approach_box: q2/q3 negativos mantêm Link2/Link3
# acima da sort_shelf (z≤0.48m) para todos os três boxes.
_APPROACH_BOX_SEED_Q = np.array([0.0, -0.4, -1.5, -1.3, 0.0, 0.0])
# Seed para via_box (z=1.15m world, diretamente acima do box em transit altitude).
# Seed [0.5,-0.5,-0.8,-1.5,0.5,0] converge para o ramo compacto (q2≈-0.5,q3≈-0.8)
# que mantém Link2/Link3 com y_max<0.52 (abaixo da parede frontal das caixas
# y=0.52 world) para todos os três boxes. Outros seeds convergem para ramo errado
# (q2≈-1.6, q3≈+1.2) onde Link2 invade a zona das caixas.
_VIA_BOX_SEED_Q = np.array([0.5, -0.5, -0.8, -1.5, 0.5, 0.0])
_APPROACH_CLEAR = 0.15    # m — altura de pré-abordagem (acima da parede das caixas: 0.705m)
_LIFT_HEIGHT    = 0.22    # m — altura de levantamento pós-grasp
_CLOSE_EXTRA    = 0.05    # fração extra de fechamento sobre o cfg nominal
_MAX_JOINT_VEL  = 0.50    # rad/s
_N_TRAJ_STEPS   = 12      # waypoints por segmento de trajetória
_N_CART_VIA     = 10      # waypoints Cartesianos para transição HOME → via_pick

# Approach vector padrão: de cima para baixo (esteira horizontal).
# Para IK de pick, usar elbow_up=False: cotovelo fisicamente acima da correia
# (~1.15 m world), Link6 + mão COVVI chegam ao ponto de captura.
_AV_DOWN = np.array([0.0, 0.0, -1.0])

# Altura do base_link do robô no world frame do Gazebo.
# Spawn z=0.375; URDF world_joint xyz=[0,0,0.03] → base_link em z=0.405.
# Todas as posições world frame devem ter esse offset subtraído antes do IK,
# pois o módulo kinematics trabalha no frame da base do robô.
_ROBOT_BASE_Z = 0.405

# Altura de trânsito via_box em robot frame (= 1.15m world).
# Deve estar acima de: belt_guides (0.935m) e de uma zona de descontinuidade
# na solução IK para box2/box3 entre 1.03-1.085m que mergulha joint3/joint4
# dentro da sort_shelf. Validado numericamente para frasco, tubo e ampola.
_TRANSIT_Z = 1.15 - _ROBOT_BASE_Z   # 0.745 m robot frame


# ── Bounding boxes dos links STL (frame local do link, em metros) ─────
# Usados para verificação em tempo de execução: proibido qualquer link do
# braço (exceto mão) tocar objeto spawnado.
_LINK_STL_BOUNDS: dict[str, tuple] = {
    'base_link': (-0.139, +0.093, -0.093, +0.093,  0.000, +0.093),
    'Link1':     (-0.077, +0.077, -0.102, +0.077, -0.097, +0.109),
    'Link2':     (-0.669, +0.077, -0.077, +0.077, +0.102, +0.302),
    'Link3':     (-0.614, +0.061, -0.061, +0.061, -0.023, +0.124),
    'Link4':     (-0.046, +0.046, -0.068, +0.089, -0.067, +0.046),
    'Link5':     (-0.046, +0.046, -0.101, +0.067, -0.057, +0.046),
    'Link6':     (-0.045, +0.045, -0.055, +0.045, -0.042,  0.000),
}

# ── Bounding boxes dos objetos spawnados (world frame, cx cy cz sx sy sz) ─
# Frasco r=42mm h=90mm: centro z = belt_top(0.806) + h/2 = 0.851
# Tubo   r=12mm h=120mm: centro z = 0.806 + 0.060 = 0.866
# Ampola r=5mm  h=75mm:  centro z = 0.806 + 0.0375 = 0.844
_PICK_OBJ_BBOX: dict[str, tuple] = {
    'frasco': (0.75, 0.00, 0.851, 0.090, 0.090, 0.090),
    'tubo':   (0.75, 0.00, 0.866, 0.030, 0.030, 0.120),
    'ampola': (0.75, 0.00, 0.844, 0.015, 0.015, 0.075),
}

# ── Bounding boxes das caixas de destino (world frame, cx cy cz sx sy sz) ─
# Caixas: pose=(box_x, 0.65, 0.535), paredes h=0.17, topo em z=0.705m
# Hull exterior: x±0.135, y±0.130 relativo ao centro, z=[0.500, 0.705]
_BIN_BBOX: dict[str, tuple] = {
    'box1': (-0.05, 0.65, 0.603, 0.270, 0.260, 0.205),
    'box2': ( 0.25, 0.65, 0.603, 0.270, 0.260, 0.205),
    'box3': ( 0.55, 0.65, 0.603, 0.270, 0.260, 0.205),
}


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


# ── Verificação de colisão em tempo de execução ───────────────────────

def _link_world_aabb(q: np.ndarray, link_idx: int) -> tuple:
    """
    Calcula AABB world (cx,cy,cz,sx,sy,sz) de um link do braço.
    link_idx: 0=base_link, 1..6=Link1..Link6.
    """
    T_base = np.eye(4)
    T_base[2, 3] = _ROBOT_BASE_Z

    if link_idx == 0:
        T_world = T_base
        key = 'base_link'
    else:
        T_world = T_base @ fk_partial(q, link_idx)
        key = f'Link{link_idx}'

    xmin, xmax, ymin, ymax, zmin, zmax = _LINK_STL_BOUNDS[key]
    pts = []
    for x in (xmin, xmax):
        for y in (ymin, ymax):
            for z in (zmin, zmax):
                p = T_world @ np.array([x, y, z, 1.0])
                pts.append(p[:3])
    pts = np.array(pts)
    cx = float((pts[:, 0].min() + pts[:, 0].max()) / 2)
    cy = float((pts[:, 1].min() + pts[:, 1].max()) / 2)
    cz = float((pts[:, 2].min() + pts[:, 2].max()) / 2)
    sx = float(pts[:, 0].max() - pts[:, 0].min())
    sy = float(pts[:, 1].max() - pts[:, 1].min())
    sz = float(pts[:, 2].max() - pts[:, 2].min())
    return (cx, cy, cz, sx, sy, sz)


def _bbox_overlap(a: tuple, b: tuple, margin: float = 0.005) -> tuple[bool, float]:
    """Verifica overlap entre dois AABBs. Retorna (colide, clearance_mm)."""
    cx1, cy1, cz1, sx1, sy1, sz1 = a
    cx2, cy2, cz2, sx2, sy2, sz2 = b
    ox = abs(cx1 - cx2) - (sx1 + sx2) / 2 - margin
    oy = abs(cy1 - cy2) - (sy1 + sy2) / 2 - margin
    oz = abs(cz1 - cz2) - (sz1 + sz2) / 2 - margin
    collides = ox < 0 and oy < 0 and oz < 0
    return collides, max(ox, oy, oz) * 1000.0


def _arm_clears_bbox(q: np.ndarray, bbox: tuple,
                     links: tuple = (1, 2, 3, 4, 5, 6)) -> tuple[bool, str]:
    """
    Verifica que os links do braço (sem mão) não colidem com bbox.
    Retorna (tudo_livre, mensagem_de_diagnóstico).
    """
    for li in links:
        la = _link_world_aabb(q, li)
        coll, clr = _bbox_overlap(la, bbox)
        if coll:
            return False, f'Link{li} penetra objeto (clearance={clr:.1f}mm)'
    return True, 'OK'


# ── Trajetória Cartesiana ─────────────────────────────────────────────

def _cartesian_arm_goal(q_start: np.ndarray,
                        q_end: np.ndarray,
                        n_via: int = _N_CART_VIA
                        ) -> tuple[FollowJointTrajectory.Goal, float]:
    """
    Trajetória com n_via waypoints interpolados no espaço Cartesiano do TCP.

    Cada waypoint é calculado por IK ao longo da linha reta entre FK(q_start)
    e FK(q_end), com seed propagado da solução anterior. Isso garante que:
      1. O TCP percorra uma trajetória previsível (linha reta Cartesiana).
      2. As mudanças de junta por passo sejam pequenas (seed contínuo).
      3. Nenhum link do braço varra regiões de obstáculos durante transições
         de grande amplitude no espaço de juntas (ex.: HOME → via_pick).

    A mudança de branch de IK (e.g., "home branch" → "pick branch") ocorre
    gradualmente ao longo dos n_via passos, nunca num único salto de 3 rad.
    """
    T0 = forward_kinematics(q_start)
    T1 = forward_kinematics(q_end)
    p0 = T0[:3, 3]
    p1 = T1[:3, 3]

    configs: list[np.ndarray] = []
    q_prev = q_start.copy()

    for i in range(1, n_via + 1):
        alpha = float(i) / n_via
        p_i = p0 + alpha * (p1 - p0)
        q_i, _ = inverse_kinematics(p_i, _AV_DOWN, q_seed=q_prev, elbow_up=False)
        configs.append(q_i)
        q_prev = q_i

    # Duração: maior delta de junta em qualquer passo consecutivo
    q_all = [q_start] + configs
    max_delta = max(
        float(np.max(np.abs(q_all[i + 1] - q_all[i])))
        for i in range(len(q_all) - 1)
    )
    step_dur = max(max_delta / _MAX_JOINT_VEL, 0.40)
    total_dur = max(step_dur * len(configs), 3.0)

    traj = JointTrajectory()
    traj.joint_names = _ARM_JOINTS

    for i, q in enumerate(configs, 1):
        smooth = 0.5 * (1.0 - math.cos(math.pi * i / len(configs)))
        t = total_dur * smooth
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        if i == len(configs):
            pt.velocities    = [0.0] * 6
            pt.accelerations = [0.0] * 6
        sec = int(t)
        pt.time_from_start = Duration(sec=sec, nanosec=int((t - sec) * 1e9))
        traj.points.append(pt)

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    return goal, total_dur


# ── Nó executor ──────────────────────────────────────────────────────

class GraspExecutorNode(Node):

    def __init__(self):
        super().__init__('grasp_executor')

        # Parâmetros do sistema (todos declarados antes do uso)
        self.declare_parameter('sim_only', True)
        self.declare_parameter('pick_x', 0.65)
        self.declare_parameter('pick_y', 0.00)
        self.declare_parameter('pick_z_frasco', 0.916)
        self.declare_parameter('pick_z_tubo',   0.946)
        self.declare_parameter('pick_z_ampola', 0.913)
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
        # em world z = _ROBOT_BASE_Z (0.405 m).
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
            # via_pick: altura TRANSIT_Z diretamente sobre a pick station.
            # Usar _PICK_SEED_Q garante que via_pick, approach_pick e pick
            # estejam todos no mesmo branch de IK (configuração compacta).
            # A transição HOME → via_pick é feita por caminho Cartesiano
            # (ver FASE 0) para evitar que Link2/Link3 varram a zona do objeto.
            via_pick_pos  = np.array([p_pick[0], p_pick[1], _TRANSIT_Z])
            # via_box: altura fixa _TRANSIT_Z (1.15m world) sobre a caixa.
            via_pos       = np.array([p_box[0],  p_box[1],  _TRANSIT_Z])
            approach_box  = p_box + np.array([0.0, 0.0, _APPROACH_CLEAR])

            # Ordem de seed para o lado do pick (pick primeiro com seed por objeto,
            # depois approach/via/lift derivados de q_pick):
            #   pick          ← _PICK_SEED_Q[obj_name]  (seed compacto por objeto)
            #   approach_pick ← q_pick (branch consistente com pick)
            #   via_pick      ← q_ap   (mesmo branch, z mais alto)
            #   lift          ← q_vp   (via_pick próximo em z)
            q_pick, ok2     = inverse_kinematics(p_pick,         _AV_DOWN, _PICK_SEED_Q[obj_class],      elbow_up=False)
            if not ok2:
                raise RuntimeError(f'IK pick falhou: {p_pick}')
            q_ap,   ok1     = inverse_kinematics(approach_pick,  _AV_DOWN, q_pick,                       elbow_up=False)
            if not ok1:
                raise RuntimeError(f'IK abordagem pick falhou: {approach_pick}')
            q_vp,   ok_vp   = inverse_kinematics(via_pick_pos,   _AV_DOWN, q_ap,                         elbow_up=False)
            if not ok_vp:
                raise RuntimeError(f'IK via_pick falhou: {via_pick_pos}')
            q_lift, ok_lift = inverse_kinematics(lift_pos,       _AV_DOWN, q_vp,                         elbow_up=False)
            if not ok_lift:
                raise RuntimeError(f'IK lift falhou: {lift_pos}')

            # Lado da caixa: approach_box com seed compacto; via_box com seed
            # _VIA_BOX_SEED_Q que converge para ramo compacto (q2≈-0.5, q3≈-0.8)
            # para TODOS os três boxes. Seed encadeado (ab→via) divergia para ramo
            # errado (q2≈-1.6) em box2/box3 na altitude z=1.15m.
            q_ab,   ok3     = inverse_kinematics(approach_box,   _AV_DOWN, _APPROACH_BOX_SEED_Q, elbow_up=False)
            if not ok3:
                raise RuntimeError(f'IK abordagem box falhou: {approach_box}')
            q_via,  ok_via  = inverse_kinematics(via_pos,        _AV_DOWN, _VIA_BOX_SEED_Q,      elbow_up=False)
            if not ok_via:
                raise RuntimeError(f'IK via_box falhou: {via_pos}')

            # ── Verificação estática: nenhum link do braço toca o objeto ─
            # O check é feito nos waypoints IK calculados. A FASE 0 (caminho
            # Cartesiano) garante que nenhum waypoint intermédio viola esses
            # limites. Qualquer violação nos waypoints fixos levanta RuntimeError.
            obj_bbox = _PICK_OBJ_BBOX.get(obj_class)
            if obj_bbox is not None:
                for wp_name, q_wp in (('via_pick', q_vp), ('approach_pick', q_ap)):
                    ok_c, msg_c = _arm_clears_bbox(q_wp, obj_bbox)
                    if not ok_c:
                        raise RuntimeError(
                            f'Colisão de braço com objeto [{obj_class}] em {wp_name}: {msg_c}')
                # pick: mão toca o objeto (esperado); braço NÃO deve tocar
                ok_c, msg_c = _arm_clears_bbox(q_pick, obj_bbox, links=(1, 2, 3, 4, 5))
                if not ok_c:
                    raise RuntimeError(
                        f'Colisão de braço com objeto [{obj_class}] em pick: {msg_c}')

            bin_bbox = _BIN_BBOX.get(box_key)
            if bin_bbox is not None:
                for wp_name, q_wp in (('via_box', q_via), ('approach_box', q_ab)):
                    ok_c, msg_c = _arm_clears_bbox(q_wp, bin_bbox)
                    if not ok_c:
                        raise RuntimeError(
                            f'Colisão de braço com caixa [{box_key}] em {wp_name}: {msg_c}')

            # ── Configurações da mão ───────────────────────────────────
            cfg_open   = HAND_CONFIGS['open']
            cfg_grasp  = hand_ik(grip_type, obj_diam)
            cfg_closed = _close_extra(cfg_grasp)

            # ── FASE 0: HOME → via_pick (caminho Cartesiano) ────────────
            # Interpola n_via posições TCP ao longo da linha reta de HOME_TCP
            # até via_pick_TCP. IK de cada posição usa o resultado anterior
            # como seed: garante transição suave (delta ≈ 3.1 rad em q3 dividido
            # por _N_CART_VIA passos) e mantém o TCP acima de via_pick_z ≥ 1.15m
            # world durante toda a transição — zona do objeto fica em z ≤ 0.946m.
            self._status_msg = f'APPROACH_PICK:{obj_class}'
            self.get_logger().info('[F0] HOME → via_pick (caminho Cartesiano)')
            self._send_arm_cartesian(q_vp)
            time.sleep(0.2)

            # ── FASE 1: Abrir mão + via_pick → approach_pick ─────────────
            # Pequeno descenso dentro do mesmo branch de IK: delta q3 ≈ 0.36 rad.
            self.get_logger().info('[F1] Abrindo mão + via_pick → approach_pick')
            self._send_hand(cfg_open, 2.0)
            self._send_arm(q_ap)
            time.sleep(0.3)

            # ── FASE 2: Descer com mão aberta + configurar grip no ponto ──
            # Mão aberta durante a descida: dedos se estendem horizontalmente,
            # sem componente z_mcp (downward). Grip só se fecha ao chegar no
            # pick_z, evitando colisão das pontas com a esteira durante descida.
            self.get_logger().info('[F2] Descida (mão aberta) + configuração grip')
            self._send_arm(q_pick)
            self._send_hand(cfg_grasp, 1.5)
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

            # ── FASE 5: Trânsito lateral — pick area → via_box ─────────
            # Caminho Cartesiano: TCP percorre linha reta de lift_pos até via_box.
            # Evita que Link2/Link3 varram a zona das caixas (z ≤ 0.705 m) durante
            # a transição de branch PICK→HOME que ocorre neste segmento.
            self._status_msg = f'TRANSIT:{obj_class}→{box_key}'
            self.get_logger().info(f'[F5] Trânsito lateral → {box_key} (Cartesiano)')
            self._send_arm_cartesian(q_via)
            time.sleep(0.2)

            # ── FASE 6: Descer para abordagem da caixa ─────────────────
            # Caminho Cartesiano: garante descida vertical direta sobre a abertura
            # da caixa (TCP passa pelo centro da abertura), evitando paredes laterais.
            self.get_logger().info(f'[F6] Descida abordagem → {box_key} (Cartesiano)')
            self._send_arm_cartesian(q_ab)
            time.sleep(0.2)

            # ── FASE 7: Soltar acima da caixa ─────────────────────────
            # TCP já em approach_box_z = box_z + _APPROACH_CLEAR ≈ 0.75 m world,
            # que está acima das paredes das caixas (top ≈ 0.705 m).
            # O objeto cai livremente para dentro da caixa sem que o braço desça.
            self._status_msg = f'PLACING:{obj_class}'
            self.get_logger().info(f'[F7] Soltando acima de {box_key}')
            self._send_hand(cfg_open, 2.0)
            time.sleep(0.3)

            # ── FASE 8: Subir ao via_box ───────────────────────────────
            # Caminho Cartesiano: subida vertical até altura de segurança,
            # espelho da descida da Fase 6.
            self.get_logger().info('[F8] Subindo ao via_box (Cartesiano)')
            self._send_arm_cartesian(q_via)
            time.sleep(0.1)

            # ── FASE 9: Retorno → HOME ─────────────────────────────────
            self._status_msg = 'HOMING'
            self.get_logger().info('[F9] Retorno → HOME')
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
        """Envia braço ao home via caminho Cartesiano e libera busy."""
        # Caminho Cartesiano para HOME: evita que o braço desça abaixo
        # de obstáculos durante a transição de qualquer configuração para HOME.
        self._send_arm_cartesian(_HOME_Q)
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

    def _send_arm_cartesian(self, q_target: np.ndarray,
                             n_via: int = _N_CART_VIA):
        """
        Envia o braço para q_target via trajetória Cartesiana.

        Calcula n_via waypoints intermediários ao longo da linha reta
        TCP_atual → TCP_alvo, resolvendo IK com seed propagado.
        Usado para transições de grande amplitude de junta (HOME → via_pick)
        que poderiam fazer links varreram a zona de objetos spawnados.
        """
        goal, _ = _cartesian_arm_goal(self._current_q, q_target, n_via)
        self._arm_ac.send_goal(goal)  # blocking
        self._current_q = q_target.copy()

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
