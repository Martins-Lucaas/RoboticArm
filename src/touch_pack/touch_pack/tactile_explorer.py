"""
tactile_explorer.py — Backend ROS 2 da célula de palpação tátil.

Coreografia (Gupta et al., 2021) sobre uma SUPERFÍCIE HORIZONTAL.

    IDLE  →  HOME  →  CONTACT  →  HOLD  →  SLIDING  →  RETRACT  →  IDLE

Arquitetura de controle:
  Todas as fases de movimento usam STREAMING DIRETO de setpoints a 33 Hz
  (publicação em /cr10_group_controller/joint_trajectory, 1 ponto por
  mensagem). Não há action server nem trajetórias pré-planejadas:
  cada passo é calculado e enviado individualmente no loop de controle.

  Vantagens:
    - Sem fila de movimentos acumulada no controlador.
    - Nenhum movimento residual de uma fase carrega para a próxima
      (_settle() publica a posição atual repetidamente antes de toda
      transição, zerando qualquer lookahead pendente).
    - Velocidade explicitamente limitada pelo tamanho do passo
      (step_m = v_ms × dt), independente do SpeedFactor do controlador.

Fases:
  HOME      Interpolação linear no espaço de juntas a ≤ 0.3 rad/s.
  CONTACT   Streaming Jacobiano em −Z com perfil fast→slow e check de Fz.
  HOLD      PID de Fz (unchanged — já era streaming).
  SLIDING   Streaming Jacobiano em XY (direção configurável) velocidade cte.
  RETRACT   Streaming Jacobiano em +Z com mesmo perfil do CONTACT.

Interface ROS:
  sub /palpation/start    std_msgs/String   JSON params
  sub /palpation/stop     std_msgs/String
  sub /ft_sensor/wrench   geometry_msgs/WrenchStamped
  sub /joint_states       sensor_msgs/JointState
  pub /palpation/status   std_msgs/String   JSON {phase, ...}
  pub /cr10_group_controller/joint_trajectory  (streaming direto)

Parâmetros ROS:
  hold_seconds         5.0    duração da fase HOLD
  retract_mm           80.0   recuo em RETRACT
  approach_v_max_mms   50.0   velocidade inicial da descida (mm/s)
  approach_v_min_mms    5.0   velocidade final da descida (mm/s)
"""
from __future__ import annotations

import json
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy,
)

_QOS_COMMAND = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)
_QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)

from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from .kinematics import (
    forward_kinematics, jacobian,
    JOINT_MIN, JOINT_MAX, MIMIC_LIST as _MIMIC_LIST,
)


_ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

_POINTING_SEED_Q = np.array([
    0.0,
    math.radians(-20.0),
    math.radians(90.0),
    0.0,
    math.radians(90.0),
    0.0,
])

_HAND_POINTING_RAD = {
    'Thumb':  math.radians(30.0),
    'Index':  0.0,
    'Middle': math.radians(80.0),
    'Ring':   math.radians(80.0),
    'Little': math.radians(80.0),
    'Rotate': 0.0,
}

_HAND_PRIMARY = ['Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Rotate']

# Limiar de contato para o HOLD-PID.
_FORCE_CONTACT_FLOOR_N = 0.05

# Saturações de segurança do PID de força no HOLD.
_PID_V_MAX_MS = 0.005   # 5 mm/s
_PID_I_MAX_Ns = 5.0
_PID_DT_S     = 0.030   # período do loop (33 Hz)

# ── Parâmetros do loop de streaming ──────────────────────────────────────────
_CTRL_DT    = 0.030   # período de cada passo (33 Hz)
_CTRL_LOOK  = 0.10    # time_from_start enviado ao controller (s)
_JAC_LAM    = 0.01    # regularização DLS
_ORI_GAIN   = 0.5     # ganho de correção de orientação
_Z_CORR_GAIN = 0.5   # ganho de correção de posição Z durante sliding (P-controller por tick)
_HOME_MAX_RAD_S = 0.30  # velocidade máxima do HOME (≈ 17°/s por junta)
_SETTLE_TICKS   = 6     # ticks de espera entre fases (6 × 30 ms = 180 ms)


class TactileExplorer(Node):

    def __init__(self):
        super().__init__('tactile_explorer')

        self.declare_parameter('hold_seconds',        5.0)
        self.declare_parameter('retract_mm',          80.0)
        self.declare_parameter('arm_base_z',          0.78)
        self.declare_parameter('approach_v_max_mms',  50.0)
        self.declare_parameter('approach_v_min_mms',   5.0)
        self.declare_parameter('kp', 0.001)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)

        self._phase: str = 'IDLE'
        self._busy = threading.Event()
        self._params_lock = threading.Lock()
        self._target_force_n: float = 1.0
        self._slide_speed_mms: float = 10.0
        self._slide_distance_mm: float = 90.0
        self._target_distance_cm: float = 5.0
        self._slide_dir_vec: np.ndarray = np.array([0.0, 1.0])
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
        self._stop_requested = threading.Event()

        cb = ReentrantCallbackGroup()

        self.create_subscription(String, '/palpation/start',
                                  self._cb_start, _QOS_COMMAND, callback_group=cb)
        self.create_subscription(String, '/palpation/stop',
                                  self._cb_stop, 10, callback_group=cb)
        self.create_subscription(WrenchStamped, '/ft_sensor/wrench',
                                  self._cb_wrench, _QOS_SENSOR, callback_group=cb)
        self.create_subscription(JointState, '/joint_states',
                                  self._cb_joints, 50, callback_group=cb)

        self._status_pub = self.create_publisher(String, '/palpation/status', 10)

        # Publisher direto no tópico do controller — sem action server.
        # depth=1: sem fila; cada nova mensagem substitui a anterior para
        # evitar rajada de setpoints antigos após jitter do SO.
        self._arm_traj_pub = self.create_publisher(
            JointTrajectory,
            '/cr10_group_controller/joint_trajectory', 1)
        self._hand_pub = self.create_publisher(
            JointTrajectory,
            '/hand_position_controller/joint_trajectory', 5)

        self.get_logger().info('tactile_explorer pronto — streaming 33 Hz')
        self.create_timer(0.10, self._publish_status, callback_group=cb)

    # ──────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────
    _FT_MAX_PLAUSIBLE_N = 500.0

    def _cb_wrench(self, msg: WrenchStamped):
        fx, fy, fz = msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z
        if not (math.isfinite(fx) and math.isfinite(fy) and math.isfinite(fz)
                and abs(fx) < self._FT_MAX_PLAUSIBLE_N
                and abs(fy) < self._FT_MAX_PLAUSIBLE_N
                and abs(fz) < self._FT_MAX_PLAUSIBLE_N):
            return
        with self._ft_lock:
            self._ft_force = np.array([fx, fy, fz], dtype=np.float64)
            self._ft_torque = np.array(
                [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z],
                dtype=np.float64)
            self._ft_stamp_s = (
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def _cb_joints(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        with self._q_lock:
            for i, j in enumerate(_ARM_JOINTS):
                if j in idx:
                    self._current_q[i] = float(msg.position[idx[j]])

    def _cb_stop(self, msg: String) -> None:
        if self._busy.is_set():
            self._stop_requested.set()
            self.get_logger().warn('[STOP] Parada solicitada.')

    def _cb_start(self, msg: String):
        if self._busy.is_set():
            self.get_logger().warn(
                f'Recebido /palpation/start mas explorer está em '
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
            approach = payload.get('approach_speed_mms')
            if approach is not None:
                try:
                    v_max = max(1.0, float(approach))
                    v_min = max(0.5, v_max * 0.2)
                    self.set_parameters([
                        rclpy.parameter.Parameter(
                            'approach_v_max_mms',
                            rclpy.parameter.Parameter.Type.DOUBLE, v_max),
                        rclpy.parameter.Parameter(
                            'approach_v_min_mms',
                            rclpy.parameter.Parameter.Type.DOUBLE, v_min),
                    ])
                except (TypeError, ValueError) as exc:
                    self.get_logger().warn(f'approach_speed_mms inválido: {exc}')
            self._kp = float(payload.get('kp', self._kp))
            self._ki = float(payload.get('ki', self._ki))
            self._kd = float(payload.get('kd', self._kd))
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
    # Status
    # ──────────────────────────────────────────────────────────────────
    def _publish_status(self):
        with self._ft_lock:
            f = self._ft_force.copy()
        with self._params_lock:
            tgt = float(self._target_force_n)
        msg = String()
        msg.data = json.dumps({
            'phase': self._phase,
            'target_force_n': tgt,
            'measured_force_normal_n': float(abs(f[2])),
            'measured_force_mag_n': float(np.linalg.norm(f)),
            'fx': float(f[0]), 'fy': float(f[1]), 'fz': float(f[2]),
            'speed_mms': self._slide_speed_mms,
            'distance_mm': self._slide_distance_mm,
            'hold_seconds': float(self.get_parameter('hold_seconds').value),
            'kp': self._kp, 'ki': self._ki, 'kd': self._kd,
        })
        self._status_pub.publish(msg)

    def _set_phase(self, phase: str):
        self._phase = phase
        self.get_logger().info(f'[FSM] → {phase}')
        self._publish_status()

    # ──────────────────────────────────────────────────────────────────
    # Primitiva de streaming — 1 ponto por mensagem, substitui o goal
    # atual no controller (sem queue). Chamada a cada _CTRL_DT segundos.
    # ──────────────────────────────────────────────────────────────────
    def _stream_q(self, q: np.ndarray, dt_s: float) -> None:
        """Publica 1 setpoint. time_from_start = dt_s (lookahead do ctrl)."""
        msg = JointTrajectory()
        msg.joint_names = list(_ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        sec = int(dt_s)
        pt.time_from_start = Duration(sec=sec,
                                       nanosec=int((dt_s - sec) * 1e9))
        msg.points.append(pt)
        self._arm_traj_pub.publish(msg)

    def _q_now(self) -> np.ndarray:
        with self._q_lock:
            return self._current_q.copy()

    # ──────────────────────────────────────────────────────────────────
    # _settle: publica posição atual por N ticks para zerar lookahead
    # e movimento residual antes de cada transição de fase.
    # ──────────────────────────────────────────────────────────────────
    def _settle(self, ticks: int = _SETTLE_TICKS) -> None:
        q = self._q_now()
        for _ in range(ticks):
            self._stream_q(q, _CTRL_LOOK + _CTRL_DT)
            time.sleep(_CTRL_DT)

    # ──────────────────────────────────────────────────────────────────
    # Movimento no espaço de juntas: interpola linearmente q_from → q_to
    # a velocidade máxima de _HOME_MAX_RAD_S rad/s por junta.
    # ──────────────────────────────────────────────────────────────────
    def _joint_stream_to(self, q_target: np.ndarray) -> bool:
        q_from = self._q_now()
        delta = np.asarray(q_target, float) - q_from
        max_d = float(np.max(np.abs(delta)))
        if max_d < 0.001:
            return True
        n_steps = max(1, int(math.ceil(max_d / (_HOME_MAX_RAD_S * _CTRL_DT))))
        for i in range(1, n_steps + 1):
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                return False
            alpha = i / n_steps
            q = np.clip(q_from + alpha * delta, JOINT_MIN, JOINT_MAX)
            self._stream_q(q, _CTRL_LOOK + _CTRL_DT)
            time.sleep(_CTRL_DT)
        return True

    # ──────────────────────────────────────────────────────────────────
    # Movimento Cartesiano retilíneo por streaming Jacobiano a 33 Hz.
    #
    # Cada iteração:
    #   1. Verifica stop / força
    #   2. Calcula step_m a partir do perfil de velocidade
    #   3. Aplica twist Jacobiano DLS (translação + correção de orientação)
    #   4. Publica 1 setpoint via _stream_q
    #   5. Dorme _CTRL_DT
    #
    # Sem pré-planejamento: o próximo passo é calculado depois do anterior.
    # Não há fila — cada mensagem substitui a anterior no controller.
    #
    # Retorna: 'done' | 'force' | 'stop' | 'error'
    # ──────────────────────────────────────────────────────────────────
    def _cartesian_stream(self, direction: np.ndarray, total_m: float, *,
                           v_const_ms: float | None = None,
                           v_max_ms: float | None = None,
                           v_min_ms: float | None = None,
                           lock_ori: bool = False,
                           lock_z: bool = False,
                           force_threshold_n: float | None = None) -> str:
        d = np.asarray(direction, dtype=float).flatten()
        nd = float(np.linalg.norm(d))
        if nd < 1e-9 or total_m <= 0.0:
            self.get_logger().error('_cartesian_stream: direção/distância inválida.')
            return 'error'
        d /= nd

        constant = v_const_ms is not None
        if not constant and (v_max_ms is None or v_min_ms is None):
            self.get_logger().error(
                '_cartesian_stream: forneça v_const_ms OU (v_max_ms, v_min_ms).')
            return 'error'

        I6 = np.eye(6)

        # Captura orientação e/ou altura Z iniciais para travamento.
        # Ambos derivam do mesmo FK — calculado uma única vez no início.
        R0: np.ndarray | None = None
        z0: float | None = None
        if lock_ori or lock_z:
            T0 = forward_kinematics(self._q_now())
            if lock_ori:
                R0 = T0[:3, :3].copy()
            if lock_z:
                z0 = float(T0[2, 3])

        covered = 0.0

        while covered < total_m:
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                return 'stop'

            # Verificação de força.
            if force_threshold_n is not None:
                with self._ft_lock:
                    fz = float(abs(self._ft_force[2]))
                if fz >= force_threshold_n:
                    return 'force'

            # Lê estado real das juntas a cada tick (closed-loop Jacobiano):
            # evita acumulação de erro se o controlador não rastrear
            # perfeitamente (limites de junta, lag do hardware real).
            q = self._q_now().copy()

            # Perfil de velocidade.
            u = covered / total_m
            if constant:
                v = float(v_const_ms)
            else:
                v = float(v_min_ms) + (float(v_max_ms) - float(v_min_ms)) * (1.0 - u) ** 2
            v = max(1e-4, v)
            step = v * _CTRL_DT

            # Torção: translação na direção pedida + correções de orientação e Z.
            J = jacobian(q)
            twist = np.zeros(6)
            twist[:3] = d * step
            if R0 is not None or z0 is not None:
                T_cur = forward_kinematics(q)
                if R0 is not None:
                    R_err = R0 @ T_cur[:3, :3].T
                    err_vec = 0.5 * np.array([
                        R_err[2, 1] - R_err[1, 2],
                        R_err[0, 2] - R_err[2, 0],
                        R_err[1, 0] - R_err[0, 1],
                    ])
                    twist[3:] = _ORI_GAIN * err_vec
                if z0 is not None:
                    # Correção proporcional de altura: impede deriva Z acumulada
                    # no sliding (o DLS aproxima o Jacobiano e pode introduzir
                    # erro residual em Z a cada tick).
                    twist[2] += _Z_CORR_GAIN * (z0 - float(T_cur[2, 3]))

            try:
                dq = J.T @ np.linalg.solve(J @ J.T + _JAC_LAM**2 * I6, twist)
            except np.linalg.LinAlgError:
                self.get_logger().warn('Jacobiano singular — passo descartado.')
                time.sleep(_CTRL_DT)
                continue

            q_cmd = np.clip(q + dq, JOINT_MIN, JOINT_MAX)
            self._stream_q(q_cmd, _CTRL_LOOK + _CTRL_DT)
            covered += step
            time.sleep(_CTRL_DT)

        return 'done'

    # ──────────────────────────────────────────────────────────────────
    # Mão COVVI
    # ──────────────────────────────────────────────────────────────────
    def _send_hand_pose(self, primary_rad: dict[str, float],
                         duration_s: float = 1.8) -> None:
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
    # Fases
    # ──────────────────────────────────────────────────────────────────
    def _phase_goto_home(self) -> bool:
        """HOME — interpolação linear no espaço de juntas a ≤ 0.3 rad/s.
        Sem action server, sem pré-planejamento externo."""
        self._set_phase('HOME')
        self._send_hand_pose(_HAND_POINTING_RAD, duration_s=2.0)

        q_home = (self._user_home_q.copy()
                  if self._user_home_q is not None
                  else _POINTING_SEED_Q.copy())

        # Settle antes de mover — garante que não há lookahead residual.
        self._settle()

        if not self._joint_stream_to(q_home):
            return False

        # Settle final para estabilizar antes do CONTACT.
        self._settle(ticks=_SETTLE_TICKS * 3)

        # Verificação de orientação: o TCP deve estar apontando para baixo
        # (componente -Z da terceira coluna de R deve ser ≤ −0.7).
        # Se a home customizada tiver o TCP em orientação incorreta, a fase
        # CONTACT desceria na direção errada (lock_ori manteria a orientação ruim).
        q_actual = self._q_now()
        R_tcp = forward_kinematics(q_actual)[:3, :3]
        tcp_z_world = R_tcp[:, 2]   # terceira coluna = eixo Z do TCP no frame mundo
        if tcp_z_world[2] > -0.5:
            self.get_logger().warn(
                f'HOME: TCP não está apontando para baixo '
                f'(tcp_z_world[2]={tcp_z_world[2]:.2f}, esperado < −0.5). '
                'Palpação continua mas a descida pode ser incorreta.')
        else:
            self.get_logger().info(
                f'HOME: orientação OK — tcp_z_world[2]={tcp_z_world[2]:.2f}')

        # NÃO sobrescrever _current_q: _cb_joints já mantém o valor correto
        # a partir de /joint_states. Forçar q_home aqui descartaria o estado
        # real e poderia causar salto no primeiro passo do CONTACT.
        return True

    def _phase_contact(self) -> bool:
        """CONTACT — streaming em −Z com perfil fast→slow e monitor de Fz.
        Cada setpoint é calculado inline (sem trajetória pré-computada).
        A fase termina quando |Fz| ≥ target_force ou a distância é percorrida."""
        self._set_phase('CONTACT')
        self._settle()

        with self._params_lock:
            target_force = float(self._target_force_n)
            descent_m = float(self._target_distance_cm) * 0.01
        v_max = float(self.get_parameter('approach_v_max_mms').value) * 1e-3
        v_min = float(self.get_parameter('approach_v_min_mms').value) * 1e-3
        v_min = max(0.001, v_min)
        v_max = max(v_min, v_max)

        self.get_logger().info(
            f'CONTACT: descida {descent_m*100:.1f} cm, '
            f'perfil {v_max*1000:.0f}→{v_min*1000:.0f} mm/s, '
            f'alvo Fz={target_force:.2f} N')

        outcome = self._cartesian_stream(
            np.array([0.0, 0.0, -1.0]), descent_m,
            v_max_ms=v_max, v_min_ms=v_min,
            lock_ori=True,
            force_threshold_n=target_force)

        # Settle imediato ao detectar contato ou fim da descida.
        self._settle()

        if outcome == 'force':
            self.get_logger().info('CONTACT: força-alvo atingida.')
            return True
        if outcome == 'done':
            self.get_logger().warn(
                f'CONTACT: descida completa sem Fz ≥ {target_force} N '
                '— modo CALIBRAÇÃO.')
            return True
        return False   # 'stop' ou 'error'

    def _phase_hold(self) -> bool:
        """HOLD — PID de Fz a 33 Hz (streaming individual por tick)."""
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
            f'HOLD-PID: alvo {target_f:.2f} N, '
            f'Kp={kp:.4g} Ki={ki:.4g} Kd={kd:.4g}, '
            f'duração {hold_s:.1f} s')

        t_end = time.time() + hold_s
        while time.time() < t_end:
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                self.get_logger().warn('[STOP] HOLD interrompido.')
                return False
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
                    dq = np.zeros(6)
                q_new = np.clip(q + dq, JOINT_MIN, JOINT_MAX)
                # HOLD: usa time_from_start=dt (sem lookahead extra) para que
                # as correções de força tomem efeito em 30 ms, não 130 ms.
                # Um lookahead de 100 ms no PID causa atraso de 4 ticks →
                # integrador acumula antes do efeito → overshoot e ruídos.
                self._stream_q(q_new, dt)

            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

        if not ever_in_contact:
            self.get_logger().info(
                f'HOLD-PID: |Fz| nunca > {_FORCE_CONTACT_FLOOR_N} N '
                '— modo CALIBRAÇÃO.')
        return True

    def _phase_sliding(self) -> bool:
        """SLIDING — streaming Cartesiano retilíneo em XY a velocidade cte."""
        self._set_phase('SLIDING')
        self._settle()

        with self._params_lock:
            dist_m  = self._slide_distance_mm * 1e-3
            speed_ms = max(0.001, self._slide_speed_mms * 1e-3)
            dir_xy  = self._slide_dir_vec.copy()

        dir_world = np.array([float(dir_xy[0]), float(dir_xy[1]), 0.0])

        self.get_logger().info(
            f'SLIDING: {dist_m*1000:.0f} mm em '
            f'({dir_world[0]:+.0f},{dir_world[1]:+.0f},0) '
            f'@ {speed_ms*1000:.1f} mm/s')

        outcome = self._cartesian_stream(
            dir_world, dist_m, v_const_ms=speed_ms, lock_ori=True, lock_z=True)

        self._settle()
        return outcome in ('done', 'force')

    def _phase_retract(self) -> bool:
        """RETRACT — streaming em +Z com perfil fast→slow (sem força)."""
        self._set_phase('RETRACT')
        self._settle()

        retract_m = float(self.get_parameter('retract_mm').value) * 1e-3
        v_max = float(self.get_parameter('approach_v_max_mms').value) * 1e-3
        v_min = float(self.get_parameter('approach_v_min_mms').value) * 1e-3
        v_min = max(0.001, v_min)
        v_max = max(v_min, v_max)

        outcome = self._cartesian_stream(
            np.array([0.0, 0.0, +1.0]), retract_m,
            v_max_ms=v_max, v_min_ms=v_min,
            lock_ori=True)

        self._settle(ticks=_SETTLE_TICKS * 2)
        return outcome in ('done', 'stop')

    # ──────────────────────────────────────────────────────────────────
    # Orquestração
    # ──────────────────────────────────────────────────────────────────
    def _run_protocol(self):
        self._busy.set()
        try:
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
