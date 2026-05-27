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
    T_TOUCH_TOOL_ATTACH,
)


_ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

_POINTING_SEED_Q = np.array([
    0.0,
    0.0,
    math.radians(-90.0),
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

# Velocidade máxima de referência (rad/s) por junta — equivale ao limite
# físico do CR10 (≈ 180°/s). O speed_factor_pct da GUI escala este valor:
# 10 % → 0.314 rad/s ≈ 18°/s (seguro para palpação).
_MAX_JOINT_VEL_RAD_S = math.pi  # 180°/s


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
        self._speed_factor_pct: float = 10.0   # % do slider da GUI (padrão 10 %)
        self._ft_lock = threading.Lock()
        self._ft_force = np.zeros(3, dtype=np.float64)
        self._ft_torque = np.zeros(3, dtype=np.float64)
        self._ft_stamp_s: float = 0.0
        self._q_lock = threading.Lock()
        self._current_q = _POINTING_SEED_Q.copy()
        self._stop_requested = threading.Event()
        self._protocol_thread: threading.Thread | None = None

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
            sf = payload.get('speed_factor_pct')
            if sf is not None:
                try:
                    self._speed_factor_pct = float(
                        max(1.0, min(100.0, float(sf))))
                except (TypeError, ValueError):
                    pass
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
        self._protocol_thread = threading.Thread(
            target=self._run_protocol, daemon=True)
        self._protocol_thread.start()

    # ──────────────────────────────────────────────────────────────────
    # Status
    # ──────────────────────────────────────────────────────────────────
    def _publish_status(self):
        with self._ft_lock:
            f = self._ft_force.copy()
        with self._params_lock:
            tgt       = float(self._target_force_n)
            speed_mms = self._slide_speed_mms
            dist_mm   = self._slide_distance_mm
        msg = String()
        msg.data = json.dumps({
            'phase': self._phase,
            'target_force_n': tgt,
            'measured_force_normal_n': float(abs(f[2])),
            'measured_force_mag_n': float(np.linalg.norm(f)),
            'fx': float(f[0]), 'fy': float(f[1]), 'fz': float(f[2]),
            'speed_mms': speed_mms,
            'distance_mm': dist_mm,
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
    def _stream_q(self, q: np.ndarray, dt_s: float,
                  velocities: np.ndarray | None = None) -> None:
        """Publica 1 setpoint. time_from_start = dt_s (lookahead do ctrl).

        Quando `velocities` é fornecido (rad/s por junta), o JTC usa splines
        cúbicos contínuos em velocidade — elimina a descontinuidade que
        acontecia ao encadear mensagens de 1 ponto sem hints de velocidade.
        """
        msg = JointTrajectory()
        msg.joint_names = list(_ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        if velocities is not None:
            pt.velocities = [float(v) for v in velocities]
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
        zero_vel = np.zeros(6)
        for _ in range(ticks):
            self._stream_q(q, _CTRL_LOOK + _CTRL_DT, velocities=zero_vel)
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
        # Velocidade constante derivada do speed_factor_pct da GUI.
        # A interpolação linear implica dq/passo = delta/n_steps →
        # velocidade por junta = dq/_CTRL_DT. Clampamos pelo limite do slider.
        v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S
        vel = np.clip(delta / n_steps / _CTRL_DT, -v_lim, v_lim)
        for i in range(1, n_steps + 1):
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                return False
            alpha = i / n_steps
            q = np.clip(q_from + alpha * delta, JOINT_MIN, JOINT_MAX)
            # Último passo: velocidade zero para parada suave antes do CONTACT.
            step_vel = vel if i < n_steps else np.zeros(6)
            self._stream_q(q, _CTRL_DT, velocities=step_vel)
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
                           lock_perp: bool = False,
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

        # FK inicial — sempre calculada, independente de lock_*.
        # p_start é a âncora para medir o progresso real do TCP via FK,
        # substituindo a integração de passos comandados (que acumula erro
        # por aproximação do Jacobiano e clipping de limites articulares).
        T0 = forward_kinematics(self._q_now(), T_end=T_TOUCH_TOOL_ATTACH)
        p_start = T0[:3, 3].copy()

        R0: np.ndarray | None = None
        z0: float | None = None
        perp_dir: np.ndarray | None = None
        p0_perp: float | None = None
        if lock_ori:
            R0 = T0[:3, :3].copy()
        if lock_z:
            z0 = float(T0[2, 3])
        if lock_perp:
            # Perpendicular a d no plano XY. Para d = [0,0,±1] a norma é zero
            # e a correção é suprimida automaticamente.
            perp = np.array([-d[1], d[0], 0.0])
            pnorm = float(np.linalg.norm(perp))
            if pnorm > 1e-9:
                perp_dir = perp / pnorm
                p0_perp = float(p_start @ perp_dir)

        # Safety: timeout baseado em 10× o tempo nominal + margem de 30 s.
        v_est = float(v_const_ms) if constant else float(v_max_ms)
        v_est = max(1e-4, v_est)
        _timeout_s = max(30.0, (total_m / v_est) * 10.0)
        _t0 = time.time()
        # Detecção de direção errada: > 5 mm na direção negativa por > 3 s.
        _neg_ticks = 0
        _NEG_MAX = int(3.0 / _CTRL_DT)
        # Log diagnóstico a cada 1 s.
        _log_every = max(1, int(1.0 / _CTRL_DT))
        _tick = 0

        self.get_logger().info(
            f'_cartesian_stream: d={d.round(3)} total={total_m*1e3:.1f}mm '
            f'v={v_est*1e3:.1f}mm/s p_start={p_start.round(4)} '
            f'TCP_Z={T0[:3,2].round(3)}')

        # Progresso real do TCP na direção d (metros, medido via FK a cada tick).
        progress = 0.0

        while progress < total_m:
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                return 'stop'

            # Timeout global — evita loop eterno se o robô não se move.
            if time.time() - _t0 > _timeout_s:
                self.get_logger().error(
                    f'_cartesian_stream: timeout {_timeout_s:.0f}s '
                    f'(progress={progress*1e3:.1f}mm/{total_m*1e3:.1f}mm). Abortando.')
                return 'error'

            if force_threshold_n is not None:
                with self._ft_lock:
                    fz = float(abs(self._ft_force[2]))
                if fz >= force_threshold_n:
                    return 'force'

            q = self._q_now().copy()

            # FK do tick atual — única chamada por iteração.
            # Serve tanto para medir o progresso real quanto para as correções
            # de orientação, Z e perpendicular.
            T_cur = forward_kinematics(q, T_end=T_TOUCH_TOOL_ATTACH)
            progress = float(np.dot(T_cur[:3, 3] - p_start, d))

            # Detecção de direção errada: se TCP persistentemente se afasta
            # de p_start na direção oposta a d, o Jacobiano provavelmente está
            # sendo calculado numa configuração errada. Aborta para não bloquear.
            if progress < -0.005:
                _neg_ticks += 1
                if _neg_ticks > _NEG_MAX:
                    self.get_logger().error(
                        f'_cartesian_stream: TCP na direção errada '
                        f'(progress={progress*1e3:.1f}mm por >{3.0:.0f}s). '
                        f'TCP_cur={T_cur[:3,3].round(4)} p_start={p_start.round(4)}. '
                        'Abortando.')
                    return 'error'
            else:
                _neg_ticks = 0

            # Log periódico para diagnóstico.
            if _tick % _log_every == 0:
                self.get_logger().debug(
                    f'  t={_tick*_CTRL_DT:.1f}s progress={progress*1e3:.2f}mm '
                    f'TCP={T_cur[:3,3].round(4)}')
            _tick += 1

            # Perfil de velocidade usa o progresso FK (não passos acumulados).
            u = max(0.0, min(1.0, progress / total_m))
            if constant:
                v = float(v_const_ms)
            else:
                v = float(v_min_ms) + (float(v_max_ms) - float(v_min_ms)) * (1.0 - u) ** 2
            v = max(1e-4, v)
            step = v * _CTRL_DT

            J = jacobian(q, T_end=T_TOUCH_TOOL_ATTACH)
            twist = np.zeros(6)
            twist[:3] = d * step

            if R0 is not None:
                R_err = R0 @ T_cur[:3, :3].T
                err_vec = 0.5 * np.array([
                    R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1],
                ])
                twist[3:] = _ORI_GAIN * err_vec
            if z0 is not None:
                twist[2] += _Z_CORR_GAIN * (z0 - float(T_cur[2, 3]))
            if perp_dir is not None and p0_perp is not None:
                perp_err = p0_perp - float(T_cur[:3, 3] @ perp_dir)
                twist[:3] += _Z_CORR_GAIN * perp_err * perp_dir

            try:
                dq = J.T @ np.linalg.solve(J @ J.T + _JAC_LAM**2 * I6, twist)
            except np.linalg.LinAlgError:
                self.get_logger().warn('Jacobiano singular — passo descartado.')
                time.sleep(_CTRL_DT)
                continue

            q_cmd = np.clip(q + dq, JOINT_MIN, JOINT_MAX)
            dq_actual = q_cmd - q
            v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S
            vel_cmd = np.clip(dq_actual / _CTRL_DT, -v_lim, v_lim)
            # time_from_start = _CTRL_DT (não _CTRL_LOOK + _CTRL_DT):
            # com T=30ms e vel_hint=dq/30ms a spline cúbica é monotônica —
            # vel_hint e horizonte são consistentes. Com T=130ms e vel=dq/30ms
            # (4.3× maior que dq/T) a spline overshoot na direção oposta por
            # ~73ms antes de corrigir, e como o stream substitui a cada 30ms
            # o TCP nunca sai da zona errada → direção invertida observada.
            self._stream_q(q_cmd, _CTRL_DT, velocities=vel_cmd)
            time.sleep(_CTRL_DT)

        return 'done'

    # ──────────────────────────────────────────────────────────────────
    # Mão COVVI
    # ──────────────────────────────────────────────────────────────────
    def _send_hand_pose(self, primary_rad: dict[str, float],
                         duration_s: float | None = None) -> None:
        if duration_s is None:
            # Escala inversa ao speed_factor_pct: 10 % → 2.0 s, 100 % → 0.2 s
            duration_s = max(0.3, 2.0 * (10.0 / max(1.0, self._speed_factor_pct)))
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
        self._send_hand_pose(_HAND_POINTING_RAD)   # duração escala com speed_factor_pct

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
        R_tcp = forward_kinematics(q_actual, T_end=T_TOUCH_TOOL_ATTACH)[:3, :3]
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
        """CONTACT — streaming em −Z a velocidade constante (slider approach) com monitor de Fz.
        Termina quando |Fz| ≥ target_force ou a distância máxima é percorrida."""
        self._set_phase('CONTACT')
        self._settle()

        with self._params_lock:
            target_force = float(self._target_force_n)
            descent_m = float(self._target_distance_cm) * 0.01
        v_approach = max(0.001,
                         float(self.get_parameter('approach_v_max_mms').value) * 1e-3)

        # Direção de approach = eixo Z do TCP na pose atual (direção dos dedos).
        # Garante consistência mesmo que a home customizada tenha orientação
        # diferente de [0,0,-1]. Se por algum motivo a norma for zero, cai para
        # world -Z (que é o caso padrão com home correta apontando para baixo).
        T_pre = forward_kinematics(self._q_now(), T_end=T_TOUCH_TOOL_ATTACH)
        approach_dir = T_pre[:3, 2].copy()
        if float(np.linalg.norm(approach_dir)) < 0.1:
            approach_dir = np.array([0.0, 0.0, -1.0])

        self.get_logger().info(
            f'CONTACT: descida até {descent_m*100:.1f} cm, '
            f'velocidade constante {v_approach*1000:.0f} mm/s, '
            f'alvo Fz={target_force:.2f} N, '
            f'direção approach={approach_dir.round(3)}')

        outcome = self._cartesian_stream(
            approach_dir, descent_m,
            v_const_ms=v_approach,
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
                J = jacobian(q, T_end=T_TOUCH_TOOL_ATTACH)
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
                v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S
                vel_hold = np.clip((q_new - q) / dt, -v_lim, v_lim)
                self._stream_q(q_new, dt, velocities=vel_hold)

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
            dir_world, dist_m, v_const_ms=speed_ms,
            lock_ori=True, lock_z=True, lock_perp=True)

        self._settle()
        return outcome in ('done', 'force')

    def _phase_retract(self) -> bool:
        """RETRACT — streaming em +Z a velocidade constante (slider approach)."""
        self._set_phase('RETRACT')
        self._settle()

        retract_m = float(self.get_parameter('retract_mm').value) * 1e-3
        v_approach = max(0.001,
                         float(self.get_parameter('approach_v_max_mms').value) * 1e-3)

        # Retrocede na direção oposta ao approach (negativo do eixo Z do TCP).
        T_pre = forward_kinematics(self._q_now(), T_end=T_TOUCH_TOOL_ATTACH)
        retract_dir = -T_pre[:3, 2].copy()
        if float(np.linalg.norm(retract_dir)) < 0.1:
            retract_dir = np.array([0.0, 0.0, +1.0])

        outcome = self._cartesian_stream(
            retract_dir, retract_m,
            v_const_ms=v_approach,
            lock_ori=True)

        self._settle(ticks=_SETTLE_TICKS * 2)
        return outcome == 'done'

    # ──────────────────────────────────────────────────────────────────
    # Orquestração
    # ──────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self._stop_requested.set()
        if self._protocol_thread is not None:
            self._protocol_thread.join(timeout=2.0)
        super().destroy_node()

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
            if not self._phase_retract():
                self._set_phase('ABORTED'); return
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
