"""
tactile_explorer.py — Backend ROS 2 da célula de palpação tátil.

Coreografia (Gupta et al., 2021) sobre uma SUPERFÍCIE HORIZONTAL — sem
test_bench instanciado no mundo, todas as poses-alvo derivam da Home
customizada que a GUI envia em /palpation/start.

    IDLE  →  HOME  →  CONTACT  →  HOLD  →  SLIDING  →  RETRACT  →  IDLE
              ↑                                                       ↓
              └────────────────  /palpation/start  ─────────────────┘

Transições:

  HOME      Move o braço para a Home customizada (graus URDF) e fecha a
            mão na pose "apontar" (apenas o Index estendido).

  CONTACT   Desce o TCP em −Z por `target_distance_cm` cm a partir da
            Home. Trajetória ÚNICA com perfil fast→slow (acelera no
            início, desacelera ao se aproximar do alvo). Durante a
            execução o nó monitora |F_z| e cancela o goal assim que
            atinge `target_force_n` — não aborta por timeout até o
            tempo planejado da trajetória + margem de segurança.

  HOLD      Mantém a força normal alvo via PID de Fz por
            `hold_seconds`. A cada tick (~30 ms) calcula
            v_cmd = Kp·err + Ki·∫err + Kd·d(err)/dt (err = F* − |Fz|),
            converte em Δz, resolve Δq via Jacobian-DLS e publica um
            JointTrajectory de 1 ponto no controller. Se |Fz| nunca
            ultrapassar `_FORCE_CONTACT_FLOOR_N` durante toda a fase,
            interpreta como "sem contato — modo calibração" e segue para
            SLIDING SEM ABORTAR (essa é a saída desejada quando a
            superfície de palpação ainda não foi posicionada).

  SLIDING   Desliza lateralmente girando APENAS joint1 (todas as outras
            juntas, incluindo joint6, ficam congeladas). O arco no plano
            XY tem raio = distância XY do TCP ao eixo da base; com
            velocidade angular constante o TCP segue um arco com
            velocidade tangencial constante (`slide_speed_mms`).

  RETRACT   Sobe `retract_mm` mm em +Z via planejamento Jacobiano DLS
            (mesmo perfil suave do CONTACT, mas só uma vez sem feedback
            de força).

Interface ROS:
  sub /palpation/start    std_msgs/String   JSON params
  sub /ft_sensor/wrench   geometry_msgs/WrenchStamped
  sub /joint_states       sensor_msgs/JointState
  pub /palpation/status   std_msgs/String   JSON {phase, ...}
  action client → controller_action (default cr10_group_controller).

Parâmetros ROS:
  arm_base_z           0.78   (m) altura da base_link em world
  hold_seconds         5.0    duração da fase HOLD
  retract_mm           80.0   recuo em RETRACT
  approach_v_max_mms   50.0   velocidade inicial da descida (mm/s)
  approach_v_min_mms    5.0   velocidade final da descida (mm/s)
  controller_action    /cr10_group_controller/follow_joint_trajectory
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

from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration

from .kinematics import (
    forward_kinematics, jacobian, JOINT_MIN, JOINT_MAX,
)


_ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# Pose "apontar para baixo" (URDF rad). joint3 e joint5 em π/2 levantam o
# upper-arm + flexionam o cotovelo + giram o pulso, formando uma postura
# em que o TCP fica em frente ao robô com Z apontando para baixo. Usada
# como seed forte para a IK do `_phase_approach_pointing` (sem isso, o
# IK arranca de [0,0,0,0,0,0] que é uma singularidade do CR10 — braço
# totalmente esticado para trás e para cima).
_POINTING_SEED_Q = np.array([
    0.0,
    math.radians(-20.0),   # ombro levanta para trás
    math.radians(90.0),    # cotovelo flexionado (J3 +π/2)
    0.0,
    math.radians(90.0),    # pulso vira o TCP de "frente" para "baixo"
    0.0,
])

# Pose da mão durante a palpação — apenas o INDEX estendido, os demais
# dedos curled para fora do caminho do contato. O Rotate fica em 0.
_HAND_POINTING_RAD = {
    'Thumb':  math.radians(30.0),
    'Index':  0.0,                  # estendido (dedo de contato)
    'Middle': math.radians(80.0),
    'Ring':   math.radians(80.0),
    'Little': math.radians(80.0),
    'Rotate': 0.0,
}

_HAND_PRIMARY = ['Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Rotate']

# Limiar abaixo do qual consideramos "sem contato" — usado para decidir
# se o HOLD-PID está em modo regulação (ever_in_contact=True) ou em modo
# calibração (nunca tocou nada). 0.05 N é maior que o ruído típico da
# estimativa de força do CR10 por torque articular, mas menor que o
# menor target_force_n permitido (0.2 N na GUI).
_FORCE_CONTACT_FLOOR_N = 0.05

# Saturações de segurança do PID de força no HOLD.
_PID_V_MAX_MS  = 0.005   # 5 mm/s — velocidade máxima aplicada pelo PID
_PID_I_MAX_Ns  = 5.0     # anti-windup do termo integral
_PID_DT_S      = 0.030   # período do loop (33 Hz, mesmo do ServoJ mirror)

# Mimic joints da COVVI (26 — extraído de linear_covvi_hand_gazebo.urdf).
_MIMIC_LIST = [
    ('_lisa_j01',            'Rotate', 1.07338),
    ('_thumb_chassis_j01',   'Rotate', 1.53340),
    ('_thumb_proximal_j01',  'Thumb',  0.72022),
    ('_thumb_distal_j01',    'Thumb',  1.06686),
    ('_thumb_link_j01',      'Thumb',  0.76799),
    ('_thumb_follower_j01',  'Thumb',  0.93733),
    ('_index_proximal_j01',  'Index',  1.51604),
    ('_index_distal_j01',    'Index',  1.33574),
    ('_index_knuckle_j01',   'Index',  1.25182),
    ('_index_follower_j01',  'Index',  0.26423),
    ('_index_link_j01',      'Index',  1.33574),
    ('_middle_proximal_j01', 'Middle', 1.51604),
    ('_middle_distal_j01',   'Middle', 1.34986),
    ('_middle_knuckle_j01',  'Middle', 1.25181),
    ('_middle_follower_j01', 'Middle', 0.26423),
    ('_middle_link_j01',     'Middle', 1.34986),
    ('_ring_proximal_j01',   'Ring',   1.51604),
    ('_ring_distal_j01',     'Ring',   1.34878),
    ('_ring_knuckle_j01',    'Ring',   1.25182),
    ('_ring_follower_j01',   'Ring',   0.26423),
    ('_ring_link_j01',       'Ring',   1.34878),
    ('_little_proximal_j01', 'Little', 1.51604),
    ('_little_distal_j01',   'Little', 1.31664),
    ('_little_knuckle_j01',  'Little', 1.25182),
    ('_little_follower_j01', 'Little', 0.26423),
    ('_little_link_j01',     'Little', 1.31664),
]

class TactileExplorer(Node):

    def __init__(self):
        super().__init__('tactile_explorer')

        # ─── Parâmetros declarados ────────────────────────────────────
        self.declare_parameter('hold_seconds',         5.0)
        self.declare_parameter('retract_mm',           80.0)
        self.declare_parameter('arm_base_z',           0.78)
        # Perfil fast→slow do CONTACT: a velocidade vertical decai com
        # (1−u)² entre v_max (início) e v_min (chegada).
        self.declare_parameter('approach_v_max_mms',   50.0)
        self.declare_parameter('approach_v_min_mms',    5.0)
        self.declare_parameter('controller_action',
            '/cr10_group_controller/follow_joint_trajectory')
        # Ganhos PID padrão (v_cmd em m/s a partir do erro em N). Valores
        # conservadores — usuário deve ajustar via GUI durante calibração.
        self.declare_parameter('kp', 0.001)   # (m/s)/N
        self.declare_parameter('ki', 0.0)     # (m/s)/(N·s)
        self.declare_parameter('kd', 0.0)     # m/N

        # ─── Estado interno ──────────────────────────────────────────
        self._phase: str = 'IDLE'
        self._busy = threading.Event()
        self._params_lock = threading.Lock()
        self._target_force_n: float = 1.0
        self._slide_speed_mms: float = 10.0
        self._slide_distance_mm: float = 90.0
        # Descida em CM a partir da Home customizada (vem do payload da
        # GUI; default 5 cm — usado quando a GUI antiga não envia o campo).
        self._target_distance_cm: float = 5.0
        # Direção XY (mundo) desejada do sliding — vetor unitário no plano
        # da base. O explorer escolhe o sinal de Δθ_joint1 que melhor
        # alinha o arco com este vetor (joint1 sozinho não consegue
        # produzir linha reta arbitrária — segue um arco).
        self._slide_dir_vec: np.ndarray = np.array([0.0, 1.0])
        # Home customizada vinda da GUI (graus URDF por junta); None até a
        # GUI publicar /palpation/start pela primeira vez.
        self._user_home_q: np.ndarray | None = None
        self._kp: float = float(self.get_parameter('kp').value)
        self._ki: float = float(self.get_parameter('ki').value)
        self._kd: float = float(self.get_parameter('kd').value)
        self._ft_lock = threading.Lock()
        self._ft_force = np.zeros(3, dtype=np.float64)
        self._ft_torque = np.zeros(3, dtype=np.float64)
        self._ft_stamp_s: float = 0.0
        self._q_lock = threading.Lock()
        self._current_q = _POINTING_SEED_Q.copy()

        cb = ReentrantCallbackGroup()

        self.create_subscription(String, '/palpation/start',
                                  self._cb_start, 5, callback_group=cb)
        self.create_subscription(WrenchStamped, '/ft_sensor/wrench',
                                  self._cb_wrench, 50, callback_group=cb)
        self.create_subscription(JointState, '/joint_states',
                                  self._cb_joints, 50, callback_group=cb)

        self._status_pub = self.create_publisher(
            String, '/palpation/status', 10)

        action_name = str(self.get_parameter('controller_action').value)
        self._arm_ac = ActionClient(
            self, FollowJointTrajectory, action_name, callback_group=cb)
        # Publisher direto para a mão — usado em _phase_approach_pointing
        # para deixar APENAS o Index estendido durante a palpação.
        self._hand_pub = self.create_publisher(
            JointTrajectory,
            '/hand_position_controller/joint_trajectory', 5)
        # Publisher direto no tópico do joint_trajectory_controller do
        # braço — usado pelo HOLD-PID para fazer streaming de setpoints
        # curtos (1 ponto, t≈dt+0.1s) a cada tick, em vez de despachar
        # um goal de action por correção.
        self._arm_traj_pub = self.create_publisher(
            JointTrajectory,
            '/cr10_group_controller/joint_trajectory', 5)
        self.get_logger().info(
            f'tactile_explorer pronto — action: {action_name}')

        # Publica status a 10 Hz para a GUI saber o estado mesmo entre
        # transições.
        self.create_timer(0.10, self._publish_status, callback_group=cb)

    # ──────────────────────────────────────────────────────────────────
    # Callbacks de subscrição
    # ──────────────────────────────────────────────────────────────────
    # Limite acima do qual o valor do sensor é considerado lixo (Gazebo
    # publica valores inválidos nos primeiros frames de simulação).
    _FT_MAX_PLAUSIBLE_N = 500.0

    def _cb_wrench(self, msg: WrenchStamped):
        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z
        if not (math.isfinite(fx) and math.isfinite(fy) and math.isfinite(fz)
                and abs(fx) < self._FT_MAX_PLAUSIBLE_N
                and abs(fy) < self._FT_MAX_PLAUSIBLE_N
                and abs(fz) < self._FT_MAX_PLAUSIBLE_N):
            return
        tx = msg.wrench.torque.x
        ty = msg.wrench.torque.y
        tz = msg.wrench.torque.z
        with self._ft_lock:
            self._ft_force = np.array([fx, fy, fz], dtype=np.float64)
            self._ft_torque = np.array([tx, ty, tz], dtype=np.float64)
            self._ft_stamp_s = (
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def _cb_joints(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        with self._q_lock:
            for i, j in enumerate(_ARM_JOINTS):
                if j in idx:
                    self._current_q[i] = float(msg.position[idx[j]])

    def _cb_start(self, msg: String):
        if self._busy.is_set():
            self.get_logger().warn(
                f'Recebido /palpation/start, mas explorer já está em '
                f'{self._phase}. Ignorando.')
            return
        try:
            payload = json.loads(msg.data) if msg.data else {}
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'JSON inválido em /palpation/start: {exc}')
            return
        with self._params_lock:
            self._target_force_n = float(
                payload.get('force_n', self._target_force_n))
            self._slide_speed_mms = float(
                payload.get('speed_mms', self._slide_speed_mms))
            self._slide_distance_mm = float(
                payload.get('distance_mm', self._slide_distance_mm))
            self._target_distance_cm = float(
                payload.get('target_distance_cm', self._target_distance_cm))
            # Ganhos PID — atualizados ao vivo pela GUI a cada partida.
            self._kp = float(payload.get('kp', self._kp))
            self._ki = float(payload.get('ki', self._ki))
            self._kd = float(payload.get('kd', self._kd))
            # Direção do sliding ('+X' / '-X' / '+Y' / '-Y' em XY mundo).
            slide_dir = str(payload.get('slide_dir', '+Y')).upper().strip()
            _DIR_MAP = {
                '+X': (1.0, 0.0), '-X': (-1.0, 0.0),
                '+Y': (0.0, 1.0), '-Y': (0.0, -1.0),
            }
            if slide_dir in _DIR_MAP:
                self._slide_dir_vec = np.array(_DIR_MAP[slide_dir])
            else:
                self.get_logger().warn(
                    f'slide_dir inválido "{slide_dir}" — usando +Y.')
                self._slide_dir_vec = np.array([0.0, 1.0])
            # Home customizada (graus URDF) — converte para radianos na
            # ordem ARM_JOINTS. Se o campo não vier, mantemos o último.
            home_deg = payload.get('home_deg')
            if isinstance(home_deg, dict):
                try:
                    self._user_home_q = np.array([
                        math.radians(float(home_deg[j]))
                        for j in _ARM_JOINTS
                    ], dtype=np.float64)
                except (KeyError, TypeError, ValueError) as exc:
                    self.get_logger().warn(
                        f'home_deg malformado: {exc} — usando seed default')
        threading.Thread(target=self._run_protocol, daemon=True).start()

    # ──────────────────────────────────────────────────────────────────
    # Publicação de status (consumida pela GUI)
    # ──────────────────────────────────────────────────────────────────
    def _publish_status(self):
        with self._ft_lock:
            f = self._ft_force.copy()
            # Para palpação HORIZONTAL, o eixo normal é Z (vertical).
            f_normal = float(abs(f[2]))
            f_mag = float(np.linalg.norm(f))
        with self._params_lock:
            tgt = float(self._target_force_n)
        msg = String()
        msg.data = json.dumps({
            'phase': self._phase,
            'target_force_n': tgt,
            'measured_force_normal_n': f_normal,
            'measured_force_mag_n': f_mag,
            'fx': float(f[0]), 'fy': float(f[1]), 'fz': float(f[2]),
            'speed_mms': self._slide_speed_mms,
            'distance_mm': self._slide_distance_mm,
            'kp': self._kp, 'ki': self._ki, 'kd': self._kd,
        })
        self._status_pub.publish(msg)

    def _set_phase(self, phase: str):
        self._phase = phase
        self.get_logger().info(f'[FSM] → {phase}')
        self._publish_status()

    # ──────────────────────────────────────────────────────────────────
    # Cinemática / waypoints
    # ──────────────────────────────────────────────────────────────────
    def _q_now(self) -> np.ndarray:
        with self._q_lock:
            return self._current_q.copy()

    def _tcp_now_world(self) -> tuple[np.ndarray, np.ndarray]:
        """Devolve (pos_world, R_world) do TCP. forward_kinematics
        retorna a transformação no frame da base_link do CR10; somamos
        `arm_base_z` para subir para world."""
        base_z = float(self.get_parameter('arm_base_z').value)
        T = forward_kinematics(self._q_now())
        p = T[:3, 3].copy()
        p[2] += base_z
        R = T[:3, :3].copy()
        return p, R

    def _dispatch_traj(self, q_to: np.ndarray, duration_s: float) -> bool:
        """Envia goal de 2 pontos (start → q_to) e espera concluir."""
        q_from = self._q_now()
        return self._send_waypoints(
            [q_from, np.asarray(q_to, dtype=float)],
            [0.01, max(0.05, float(duration_s))],
        ) in ('finished',)

    def _send_waypoints(self, qs: list[np.ndarray], times: list[float],
                          *, force_threshold_n: float | None = None,
                          extra_timeout_s: float = 5.0) -> str:
        """Envia uma trajetória multi-waypoint para o controller do braço.

        Se `force_threshold_n` for fornecido, monitora |F_z| em paralelo
        e cancela o goal assim que a leitura ultrapassa o limiar — sem
        depender do timeout interno para encerrar o CONTACT.

        Retorna:
            'finished'  trajetória completou
            'force'     limiar de força disparou (cancelamento)
            'timeout'   deadline expirou
            'failed'    server indisponível / goal recusado
        """
        if not self._arm_ac.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Action server do braço indisponível.')
            return 'failed'
        if len(qs) != len(times) or not qs:
            self.get_logger().error('_send_waypoints: qs/times inválidos.')
            return 'failed'

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(_ARM_JOINTS)
        for q_wp, t_wp in zip(qs, times):
            p = JointTrajectoryPoint()
            p.positions = [float(v) for v in q_wp]
            t = max(0.0, float(t_wp))
            p.time_from_start = Duration(
                sec=int(t), nanosec=int((t - int(t)) * 1e9))
            goal.trajectory.points.append(p)

        send_future = self._arm_ac.send_goal_async(goal)
        accept_deadline = time.time() + 2.0
        while not send_future.done() and time.time() < accept_deadline:
            time.sleep(0.02)
        if not send_future.done():
            return 'failed'
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('Action server recusou o goal.')
            return 'failed'

        result_future = handle.get_result_async()
        total_time = float(times[-1])
        deadline = time.time() + total_time + extra_timeout_s

        while not result_future.done() and time.time() < deadline:
            if force_threshold_n is not None:
                with self._ft_lock:
                    f_normal = float(abs(self._ft_force[2]))
                if f_normal >= force_threshold_n:
                    self.get_logger().info(
                        f'Contato detectado ({f_normal:.3f} N) — '
                        'cancelando o goal de CONTACT.')
                    try:
                        handle.cancel_goal_async()
                    except Exception:
                        pass
                    # Espera curta pelo encerramento do goal.
                    cancel_deadline = time.time() + 1.5
                    while (not result_future.done()
                            and time.time() < cancel_deadline):
                        time.sleep(0.02)
                    return 'force'
            time.sleep(0.02)

        if result_future.done():
            return 'finished'
        return 'timeout'

    # ──────────────────────────────────────────────────────────────────
    # Mão — publicação direta no controller (sem ECI real)
    # ──────────────────────────────────────────────────────────────────
    def _send_hand_pose(self, primary_rad: dict[str, float],
                         duration_s: float = 1.8) -> None:
        """Publica uma pose na mão COVVI expandindo as 26 juntas mimic."""
        names = list(_HAND_PRIMARY)
        positions = [float(primary_rad.get(j, 0.0)) for j in _HAND_PRIMARY]
        for mimic_name, driver, mult in _MIMIC_LIST:
            names.append(mimic_name)
            positions.append(float(primary_rad.get(driver, 0.0)) * mult)
        msg = JointTrajectory()
        msg.joint_names = names
        pt = JointTrajectoryPoint()
        pt.positions = positions
        dur = max(0.1, float(duration_s))
        pt.time_from_start = Duration(
            sec=int(dur), nanosec=int((dur - int(dur)) * 1e9))
        msg.points.append(pt)
        self._hand_pub.publish(msg)

    # ──────────────────────────────────────────────────────────────────
    # Fases da máquina de estados — PALPAÇÃO HORIZONTAL
    # ──────────────────────────────────────────────────────────────────
    def _phase_goto_home(self) -> bool:
        """HOME — leva o braço para a Home CUSTOMIZADA pelo usuário (vinda
        no payload de /palpation/start) e fecha a mão na pose pointing
        (Index estendido).

        Esta é a pose de partida da palpação. A IK só será chamada na
        fase CONTACT (alvo = TCP_home − target_distance_cm em −Z).
        Como a Home vem como ângulos articulares EXPLÍCITOS, não há IK
        nesta fase — o dispatch é direto.
        """
        self._set_phase('HOME')

        # 1. Pré-shape da mão (fire-and-forget; converge enquanto o
        #    braço se desloca).
        self._send_hand_pose(_HAND_POINTING_RAD, duration_s=2.0)

        # 2. Decide a pose-alvo: Home do usuário se disponível,
        #    caso contrário, seed POINTING default.
        if self._user_home_q is not None:
            q_home = self._user_home_q.copy()
        else:
            self.get_logger().warn(
                'Home customizada ausente — usando _POINTING_SEED_Q.')
            q_home = _POINTING_SEED_Q.copy()

        if not self._dispatch_traj(q_home, duration_s=3.0):
            return False

        # 3. Cache local para o CONTACT (IK arranca daqui).
        with self._q_lock:
            self._current_q = q_home.copy()
        time.sleep(0.5)
        return True

    def _plan_jacobian_cart(self, dir_world: np.ndarray, length_m: float,
                              *, v_const_ms: float | None = None,
                              v_max_ms: float | None = None,
                              v_min_ms: float | None = None,
                              n_waypoints: int | None = None
                              ) -> tuple[list[np.ndarray], list[float]] | None:
        """Planeja uma trajetória CARTESIANA RETILÍNEA no frame da base
        (= mundo, base_link sem rotação) na direção `dir_world` por
        `length_m` metros, preservando a orientação do TCP (Δω=0).

        A cada sub-step aplicamos o twist `[dx,dy,dz,0,0,0]` e integramos
        Δq via Jacobian-DLS — todas as juntas se movem em conjunto para
        respeitar a direção pedida. Isso é o que permite, por exemplo,
        deslizar em pura translação +X ou +Y (impossível só com joint1,
        que traçaria um arco).

        Perfil de velocidade:
          v_const_ms          → velocidade constante (típico SLIDING).
          v_max_ms+v_min_ms   → (1−u)² decay  (típico CONTACT/RETRACT).

        Retorna (qs, times) com `qs[0] = q_now` e `times[0] = 0`.
        """
        d = np.asarray(dir_world, dtype=float).flatten()
        if d.size != 3:
            self.get_logger().error('_plan_jacobian_cart: dir_world inválido.')
            return None
        n_d = float(np.linalg.norm(d))
        if n_d < 1e-9 or length_m <= 0.0:
            self.get_logger().error(
                f'_plan_jacobian_cart: direção/comprimento inválidos '
                f'(|d|={n_d}, L={length_m}).')
            return None
        d = d / n_d
        constant = v_const_ms is not None
        if not constant and (v_max_ms is None or v_min_ms is None):
            self.get_logger().error(
                '_plan_jacobian_cart: forneça v_const_ms OU (v_max_ms, v_min_ms).')
            return None

        q = self._q_now().copy()
        # ~0.5 wp/mm (mín. 30). Mais wps = trajetória mais linear.
        if n_waypoints is None:
            n_waypoints = max(30, int(length_m * 500))
        step_m = length_m / n_waypoints

        lam = 0.01
        I6 = np.eye(6)
        twist = np.zeros(6)
        twist[:3] = d * step_m

        qs: list[np.ndarray] = [q.copy()]
        times: list[float] = [0.0]
        t = 0.0
        for i in range(n_waypoints):
            J = jacobian(q)
            try:
                dq = J.T @ np.linalg.solve(J @ J.T + lam*lam*I6, twist)
            except np.linalg.LinAlgError:
                self.get_logger().error(
                    'Jacobiano singular durante o planejamento — abortando.')
                return None
            q = np.clip(q + dq, JOINT_MIN, JOINT_MAX)
            u = (i + 1) / n_waypoints
            if constant:
                v_i = max(float(v_const_ms), 1e-4)
            else:
                v_i = max(float(v_min_ms),
                          float(v_min_ms)
                          + (float(v_max_ms) - float(v_min_ms))
                          * (1.0 - u) ** 2)
            t += step_m / v_i
            qs.append(q.copy())
            times.append(t)
        return qs, times

    def _phase_contact(self) -> bool:
        """CONTACT — desce `target_distance_cm` cm em −Z a partir da Home.

        Planeja a trajetória INTEIRA via Jacobian-DLS uma única vez, envia
        um único goal multi-waypoint ao controller, e monitora |F_z|
        durante a execução. Quando a força-alvo é atingida o goal é
        cancelado; caso contrário a trajetória completa até o final.

        Não há mais o timeout fixo de 8 s: o deadline acompanha o tempo
        total planejado da trajetória + margem de segurança.
        """
        self._set_phase('CONTACT')
        with self._params_lock:
            target_force = float(self._target_force_n)
            descent_m = float(self._target_distance_cm) * 0.01
        v_max_ms = float(self.get_parameter('approach_v_max_mms').value) * 1e-3
        v_min_ms = float(self.get_parameter('approach_v_min_mms').value) * 1e-3
        v_min_ms = max(0.001, v_min_ms)
        v_max_ms = max(v_min_ms, v_max_ms)

        p_home, _ = self._tcp_now_world()
        self.get_logger().info(
            f'CONTACT: planejando descida de {descent_m*100:.1f} cm '
            f'do TCP_z={p_home[2]:.3f} (alvo de força {target_force} N, '
            f'perfil {v_max_ms*1000:.0f}→{v_min_ms*1000:.0f} mm/s).')

        plan = self._plan_jacobian_cart(
            np.array([0.0, 0.0, -1.0]), descent_m,
            v_max_ms=v_max_ms, v_min_ms=v_min_ms)
        if plan is None:
            return False
        qs, times = plan
        self.get_logger().info(
            f'CONTACT: trajetória planejada — {len(qs)} waypoints, '
            f'duração nominal {times[-1]:.1f} s.')

        outcome = self._send_waypoints(
            qs, times,
            force_threshold_n=target_force,
            extra_timeout_s=5.0)
        if outcome == 'force':
            return True
        if outcome == 'finished':
            self.get_logger().warn(
                f'Descida completa ({descent_m*100:.1f} cm) sem leitura '
                f'de força ≥ {target_force} N — modo CALIBRAÇÃO: o HOLD '
                'detectará F≈0 e o experimento continuará para o SLIDING.')
            return True
        self.get_logger().error(f'CONTACT terminou com estado {outcome}.')
        return False

    def _stream_q(self, q: np.ndarray, dt_s: float) -> None:
        """Publica 1 ponto no tópico do joint_trajectory_controller.

        `dt_s` é o `time_from_start` do ponto — usar dt_loop + margem
        (~0.10 s) para que o controller tenha tempo de interpolar e não
        descarte o setpoint por estar no passado.
        """
        msg = JointTrajectory()
        msg.joint_names = list(_ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        sec = int(dt_s)
        pt.time_from_start = Duration(
            sec=sec, nanosec=int((dt_s - sec) * 1e9))
        msg.points.append(pt)
        self._arm_traj_pub.publish(msg)

    def _phase_hold(self) -> bool:
        """HOLD — PID de força mantém |Fz|=target_force_n.

        Loop @33 Hz (período `_PID_DT_S`). Para cada tick:

            err     = F* − |Fz|
            ∫err   += err·dt        (anti-windup ± `_PID_I_MAX_Ns`,
                                       só integra após o primeiro contato)
            d(err) = (err − prev)/dt
            v_cmd  = Kp·err + Ki·∫err + Kd·d(err)
            v_cmd  ∈ [−`_PID_V_MAX_MS`, +`_PID_V_MAX_MS`]      (saturação)
            Δz     = −v_cmd·dt      (err>0 → pressionar mais → −Z)
            twist  = [0, 0, Δz, 0, 0, 0]
            Δq     = Jᵀ(JJᵀ + λ²I)⁻¹·twist                   (DLS, λ=0.01)
            q_new  = clip(q + Δq, JMIN, JMAX)
            stream(q_new, dt+0.10)

        Modo CALIBRAÇÃO — se |Fz| nunca passar de `_FORCE_CONTACT_FLOOR_N`
        durante toda a fase, o PID fica congelado (não pressiona contra o
        nada), a fase espera os `hold_seconds` e o experimento continua
        normalmente para o SLIDING. Esse comportamento é intencional:
        permite testar o pipeline sem superfície de palpação posicionada.
        """
        self._set_phase('HOLD')
        hold_s = float(self.get_parameter('hold_seconds').value)
        with self._params_lock:
            target_f = float(self._target_force_n)
            kp, ki, kd = self._kp, self._ki, self._kd

        dt = _PID_DT_S
        integral = 0.0
        prev_err = 0.0
        ever_in_contact = False
        lam = 0.01
        I6 = np.eye(6)

        self.get_logger().info(
            f'HOLD-PID: alvo {target_f:.2f} N, ganhos '
            f'Kp={kp:.4g}, Ki={ki:.4g}, Kd={kd:.4g}, '
            f'duração {hold_s:.1f} s.')

        t_end = time.time() + hold_s
        while time.time() < t_end:
            t0 = time.time()
            with self._ft_lock:
                fz = float(abs(self._ft_force[2]))
            if fz > _FORCE_CONTACT_FLOOR_N:
                ever_in_contact = True

            err = target_f - fz
            if ever_in_contact:
                integral = float(np.clip(
                    integral + err * dt, -_PID_I_MAX_Ns, _PID_I_MAX_Ns))
            else:
                # Sem contato — não acumula viés. PID parado, robô estável.
                integral = 0.0
            deriv = (err - prev_err) / dt
            prev_err = err

            if ever_in_contact:
                v_cmd = kp * err + ki * integral + kd * deriv
                v_cmd = float(np.clip(v_cmd, -_PID_V_MAX_MS, _PID_V_MAX_MS))
                dz = -v_cmd * dt
                twist = np.array([0., 0., dz, 0., 0., 0.])
                q = self._q_now()
                J = jacobian(q)
                try:
                    dq = J.T @ np.linalg.solve(
                        J @ J.T + lam * lam * I6, twist)
                except np.linalg.LinAlgError:
                    self.get_logger().warn(
                        'HOLD-PID: Jacobiano singular — tick descartado.')
                    dq = np.zeros(6)
                q_new = np.clip(q + dq, JOINT_MIN, JOINT_MAX)
                self._stream_q(q_new, dt + 0.10)

            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

        if not ever_in_contact:
            self.get_logger().info(
                f'HOLD-PID: |Fz| nunca excedeu {_FORCE_CONTACT_FLOOR_N} N '
                '— modo CALIBRAÇÃO, experimento continua sem aplicar '
                'controle de força. Posicione a superfície e refaça.')
        return True

    def _phase_sliding(self) -> bool:
        """SLIDING — arrasto Cartesiano RETILÍNEO em XY na direção
        escolhida pela GUI (`slide_dir` ∈ {+X, −X, +Y, −Y}), mantendo Z
        e orientação do TCP constantes.

        A trajetória é planejada via Jacobian-DLS: a cada sub-step (mm)
        aplicamos o twist `[dx, dy, 0, 0, 0, 0]` e integramos Δq.
        TODAS as juntas se coordenam para produzir a translação em linha
        reta — joint1 sozinho só conseguiria traçar um arco, então não
        é mais usado isoladamente.

        Velocidade tangencial constante = `slide_speed_mms`.
        """
        self._set_phase('SLIDING')
        with self._params_lock:
            dist_m = self._slide_distance_mm * 1e-3
            speed_ms = max(0.001, self._slide_speed_mms * 1e-3)
            dir_xy = self._slide_dir_vec.copy()

        # Direção 3D no mundo: XY do seletor, Z=0 (preserva altura).
        dir_world = np.array([float(dir_xy[0]), float(dir_xy[1]), 0.0])

        self.get_logger().info(
            f'SLIDING: planejando arrasto Cartesiano '
            f'dir=({dir_world[0]:+.0f},{dir_world[1]:+.0f},0) '
            f'por {dist_m*1000:.0f} mm @ {speed_ms*1000:.1f} mm/s.')
        plan = self._plan_jacobian_cart(
            dir_world, dist_m, v_const_ms=speed_ms)
        if plan is None:
            return False
        qs, times = plan
        self.get_logger().info(
            f'SLIDING: trajetória — {len(qs)} waypoints, '
            f'dur {times[-1]:.2f} s.')
        outcome = self._send_waypoints(
            qs, times, extra_timeout_s=3.0)
        return outcome == 'finished'

    def _phase_retract(self) -> bool:
        """RETRACT — sobe `retract_mm` em +Z com o mesmo perfil suave do
        CONTACT (Jacobian-DLS + fast→slow), mas sem feedback de força."""
        self._set_phase('RETRACT')
        retract_m = float(self.get_parameter('retract_mm').value) * 1e-3
        v_max_ms = float(self.get_parameter('approach_v_max_mms').value) * 1e-3
        v_min_ms = float(self.get_parameter('approach_v_min_mms').value) * 1e-3
        v_min_ms = max(0.001, v_min_ms)
        v_max_ms = max(v_min_ms, v_max_ms)

        plan = self._plan_jacobian_cart(
            np.array([0.0, 0.0, +1.0]), retract_m,
            v_max_ms=v_max_ms, v_min_ms=v_min_ms)
        if plan is None:
            return False
        qs, times = plan
        outcome = self._send_waypoints(qs, times, extra_timeout_s=3.0)
        return outcome == 'finished'

    # ──────────────────────────────────────────────────────────────────
    # Orquestração do protocolo completo (thread daemon)
    # ──────────────────────────────────────────────────────────────────
    def _run_protocol(self):
        self._busy.set()
        try:
            # FASE 0: leva o braço à HOME CUSTOMIZADA (vinda no payload)
            # e fecha a mão na pose pointing (Index estendido).
            if not self._phase_goto_home():
                self._set_phase('ABORTED'); return
            if not self._phase_contact():
                self._set_phase('ABORTED'); return
            if not self._phase_hold():
                self._set_phase('ABORTED'); return
            if not self._phase_sliding():
                self._set_phase('ABORTED'); return
            self._phase_retract()
            self._set_phase('DONE')
            time.sleep(0.5)
            self._set_phase('IDLE')
        finally:
            self._busy.clear()


def main(args=None):
    rclpy.init(args=args)
    node = TactileExplorer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
