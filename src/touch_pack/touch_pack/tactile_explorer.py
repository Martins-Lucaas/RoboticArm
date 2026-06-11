"""
tactile_explorer.py — Backend ROS 2 da célula de palpação tátil.

Coreografia (Gupta et al., 2021) sobre uma SUPERFÍCIE HORIZONTAL.

    IDLE  →  HOME  →  DESCENDING  →  HOLD  →  SLIDING  →  RETRACT  →  HOME  →  IDLE

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
  HOME         Interpolação linear no espaço de juntas a ≤ 0.3 rad/s.
  DESCENDING   Streaming Jacobiano ao longo do approach até a compressão
               atingir o setpoint (profundidade da GUI = curso máximo).
  HOLD         PID de força leva a compressão ao setpoint e ESPERA a
               estabilização (janela contínua dentro da tolerância)
               antes de liberar o SLIDING.
  SLIDING      Streaming Jacobiano lateral + PID de força no approach.

Controle de força (DESCENDING/HOLD/SLIDING):
  Setpoint selecionável na GUI (force_n, máx. 10 N). A medição é
  CANCELADA se a compressão exceder 15 N (_FORCE_ABORT_LIMIT_N).

Interface ROS:
  sub /palpation/start    touch_pack_msgs/PalpationStart
  sub /palpation/stop     std_msgs/String
  sub /palpation/pause    std_msgs/Bool     true=pausa (segura posição), false=retoma
  sub /load_cell/force_net std_msgs/Float32
  sub /joint_states       sensor_msgs/JointState
  pub /palpation/status   touch_pack_msgs/PalpationStatus
  pub /cr10_group_controller/joint_trajectory  (streaming direto)

Parâmetros ROS:
  retract_mm           80.0   recuo em RETRACT
  approach_v_max_mms   50.0   velocidade inicial da descida (mm/s)
  approach_v_min_mms    5.0   velocidade final da descida (mm/s)
"""
from __future__ import annotations

import math
import sys
import threading
import time

import numpy as np
if tuple(int(x) for x in np.__version__.split(".")[:2]) >= (2, 0):
    sys.exit(
        f"[ERRO] NumPy {np.__version__} detectado — ABI incompatível com "
        "ROS 2 Humble.\n"
        "Corrija: pip install 'numpy<2'\n"
        "Confirme com: python3 -c \"import numpy; print(numpy.__version__)\""
    )
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

from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from touch_pack_msgs.msg import PalpationStart, PalpationStatus

from .kinematics import (
    forward_kinematics, jacobian,
    JOINT_MIN, JOINT_MAX, MIMIC_LIST as _MIMIC_LIST,
    T_TOUCH_TOOL_ATTACH,
)
from .constants import (
    ARM_JOINTS as _ARM_JOINTS,
    HAND_JOINTS as _HAND_PRIMARY,
    HAND_POINTING_RAD as _HAND_POINTING_RAD,
    POINTING_SEED_DEG as _POINTING_SEED_DEG,
    FORCE_ABORT_LIMIT_N as _FORCE_ABORT_LIMIT_N,
    FORCE_SETPOINT_MAX_N as _FORCE_SETPOINT_MAX_N,
)

_POINTING_SEED_Q = np.array(
    [math.radians(_POINTING_SEED_DEG[j]) for j in _ARM_JOINTS])

# ── Célula de carga calibrada (/load_cell/force_net) ─────────────────────────
# Convenção de sinal: compressão = POSITIVO, tração = NEGATIVO.
# O force_receiver_node publica força calibrada (N). A GUI aplica tara
# (tare_v - v_now) / slope → compressão positiva, tração negativa.
_CONTACT_DETECT_N     = 0.2    # N: limiar — usado no SLIDING para detecção de perda de contato
_SLIDING_SAFETY_M   = 0.30   # m: distância máxima de segurança no SLIDING

# ── PID de força (DESCENDING / HOLD / SLIDING) ───────────────────────────────
_PID_V_MAX_MS = 0.005    # m/s: correção máxima do PID (5 mm/s)
_PID_I_MAX_Ns = 5.0      # N·s: anti-windup do integrador
_FORCE_LOST_TICKS    = int(2.0 / 0.030)  # ticks sem contato antes de parar SLIDING

# ── Estabilização do setpoint no HOLD ────────────────────────────────────────
# O HOLD só libera o SLIDING quando a compressão fica DENTRO da tolerância
# em torno do setpoint por _HOLD_STABLE_S contínuos (tol = máx(_HOLD_TOL_N,
# _HOLD_TOL_PCT × setpoint)). Sair da banda reinicia a janela. Se não
# estabilizar em _HOLD_TIMEOUT_S, prossegue com aviso — o PID do SLIDING
# continua corrigindo a força durante o deslizamento.
_HOLD_TOL_N     = 0.15   # N: tolerância absoluta mínima (≈ ruído da célula)
_HOLD_TOL_PCT   = 0.05   # fração do setpoint (5 %)
_HOLD_STABLE_S  = 1.0    # s contínuos dentro da tolerância
_HOLD_TIMEOUT_S = 12.0   # s: teto de espera pela estabilização

# ── Staleness da célula de carga ─────────────────────────────────────────────
# Idade máxima da última leitura de /load_cell/force_net para que o controle
# por força seja confiável. Se a ESP32/receiver cair no meio de uma fase
# controlada por força, _fz_corrected() devolveria um valor CONGELADO e o
# PID continuaria corrigindo às cegas (abaixo do setpoint → afundaria a
# ferramenta na mesa). DESCENDING/HOLD/SLIDING abortam com outcome 'stale'.
_FORCE_STALE_S = 0.5


# ── Parâmetros do loop de streaming ──────────────────────────────────────────
_CTRL_DT    = 0.030   # período de cada passo (33 Hz)
_CTRL_LOOK  = 0.10    # time_from_start do _settle (s)
_CTRL_WIN   = 10      # waypoints por batch de streaming (10 × 30 ms = 300 ms)
_SLIDE_WIN  = 3       # janela do SLIDING com PID (3 × 30 ms = 90 ms lookahead)
_JAC_LAM    = 0.01    # regularização DLS
_ORI_GAIN   = 0.5     # ganho de correção de orientação
_Z_CORR_GAIN = 0.5   # ganho de correção perpendicular durante sliding
_HOME_MAX_RAD_S = 0.05  # velocidade máxima do HOME (≈ 3°/s por junta);
                        # ajustável via parâmetro ROS home_speed_rad_s
_SETTLE_TICKS   = 6     # ticks de espera entre fases (6 × 30 ms = 180 ms)

# Velocidade máxima de referência (rad/s) por junta — equivale ao limite
# físico do CR10 (≈ 180°/s). O speed_factor_pct da GUI escala este valor:
# 10 % → 0.314 rad/s ≈ 18°/s (seguro para palpação).
_MAX_JOINT_VEL_RAD_S = math.pi  # 180°/s


class _ForcePID:
    """PID de força → velocidade de correção ao longo do approach (m/s).

    Convenção: erro = setpoint − compressão medida. Saída positiva
    aprofunda (mais compressão); negativa alivia. O integrador só
    acumula após o primeiro contato (evita windup durante aproximação
    sem carga) e é saturado em ±_PID_I_MAX_Ns. A saída é limitada a
    ±_PID_V_MAX_MS.
    """

    def __init__(self, kp: float, ki: float, kd: float, dt: float):
        self.kp, self.ki, self.kd, self.dt = kp, ki, kd, dt
        self.ever_in_contact = False
        self._integral = 0.0
        self._prev_err = 0.0

    def step(self, err: float, in_contact: bool) -> float:
        if in_contact:
            self.ever_in_contact = True
        if self.ever_in_contact:
            self._integral = float(np.clip(
                self._integral + err * self.dt,
                -_PID_I_MAX_Ns, _PID_I_MAX_Ns))
        deriv = (err - self._prev_err) / self.dt
        self._prev_err = err
        if not self.ever_in_contact:
            return 0.0
        v = self.kp * err + self.ki * self._integral + self.kd * deriv
        return float(np.clip(v, -_PID_V_MAX_MS, _PID_V_MAX_MS))


class TactileExplorer(Node):

    def __init__(self):
        super().__init__('tactile_explorer')

        self.declare_parameter('retract_mm',          80.0)
        self.declare_parameter('arm_base_z',          0.78)
        self.declare_parameter('approach_v_max_mms',  50.0)
        self.declare_parameter('approach_v_min_mms',   5.0)
        self.declare_parameter('descent_speed_mms', 5.0)
        self.declare_parameter('home_speed_rad_s', _HOME_MAX_RAD_S)
        # Ganhos do PID de força. PI por padrão (kd=0): a derivada amplifica
        # o ruído da célula de carga; o próprio contato já amortece o loop.
        self.declare_parameter('kp', 0.001)    # (m/s)/N
        self.declare_parameter('ki', 0.0005)   # (m/s)/(N·s)
        self.declare_parameter('kd', 0.0)      # (m/s)/(N/s)

        self._phase: str = 'IDLE'
        self._busy = threading.Event()
        self._params_lock = threading.Lock()
        self._target_depth_mm: float = 5.0
        self._target_force_n:  float = 2.0   # setpoint do PID (≤ 10 N)
        self._kp = float(self.get_parameter('kp').value)
        self._ki = float(self.get_parameter('ki').value)
        self._kd = float(self.get_parameter('kd').value)
        self._target_slide_mm: float = 50.0
        self._slide_speed_mms: float = 10.0
        self._slide_dir_vec: np.ndarray = np.array([0.0, 1.0])
        self._approach_dir: np.ndarray | None = None
        self._user_home_q: np.ndarray | None = None
        self._speed_factor_pct: float = 10.0   # % do slider da GUI (padrão 10 %)
        # Repetições automáticas do experimento (campo 'repeats' da GUI).
        # _cycle/_cycles_total alimentam o status para a GUI mostrar "i/N".
        self._repeats: int = 1
        self._cycle: int = 0
        self._cycles_total: int = 1
        # Overrides de estabilização do HOLD vindos do PalpationStart
        # (0.0 no msg = "usar default" → None aqui).
        self._hold_tol_n: float | None = None
        self._hold_stable_s: float | None = None
        self._hold_timeout_s: float | None = None
        self._lc_lock = threading.Lock()
        self._lc_force_net: float = 0.0   # compressão positiva, tare-compensada
        self._lc_force_ts: float = 0.0    # time.monotonic() da última leitura
        self._q_lock = threading.Lock()
        self._current_q = _POINTING_SEED_Q.copy()
        self._stop_requested = threading.Event()
        self._pause_requested = threading.Event()
        self._protocol_thread: threading.Thread | None = None

        cb = ReentrantCallbackGroup()

        self.create_subscription(PalpationStart, '/palpation/start',
                                  self._cb_start, _QOS_COMMAND, callback_group=cb)
        self.create_subscription(String, '/palpation/stop',
                                  self._cb_stop, 10, callback_group=cb)
        self.create_subscription(Bool, '/palpation/pause',
                                  self._cb_pause, 10, callback_group=cb)
        self.create_subscription(Float32, '/load_cell/force_net',
                                  self._cb_lc_force_net, _QOS_SENSOR, callback_group=cb)
        self.create_subscription(JointState, '/joint_states',
                                  self._cb_joints, 50, callback_group=cb)

        self._status_pub = self.create_publisher(
            PalpationStatus, '/palpation/status', 10)

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
    _LC_MAX_PLAUSIBLE_N = 100.0

    def _cb_lc_force_net(self, msg: Float32) -> None:
        """Recebe /load_cell/force_net — compressão positiva, tare-compensada."""
        val = float(msg.data)
        if not math.isfinite(val) or abs(val) > self._LC_MAX_PLAUSIBLE_N:
            return
        with self._lc_lock:
            self._lc_force_net = val
            self._lc_force_ts = time.monotonic()

    def _cb_joints(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        with self._q_lock:
            for i, j in enumerate(_ARM_JOINTS):
                if j in idx:
                    self._current_q[i] = float(msg.position[idx[j]])

    def _cb_stop(self, msg: String) -> None:
        if self._busy.is_set():
            self._stop_requested.set()
            self._pause_requested.clear()   # stop vence pausa
            self.get_logger().warn('[STOP] Parada solicitada.')

    def _cb_pause(self, msg: Bool) -> None:
        """Pausa/retoma o experimento — as fases seguram a posição atual
        enquanto pausadas (ver _pause_gate)."""
        if bool(msg.data):
            if self._busy.is_set():
                self._pause_requested.set()
        else:
            self._pause_requested.clear()

    def _cb_start(self, msg: PalpationStart):
        if self._busy.is_set():
            self.get_logger().warn(
                f'Recebido /palpation/start mas explorer está em '
                f'{self._phase}. Ignorando.')
            return
        with self._params_lock:
            self._target_depth_mm = float(msg.depth_mm)
            # Setpoint do PID de força — saturado no máximo selecionável.
            self._target_force_n = float(np.clip(
                float(msg.force_n), 0.1, _FORCE_SETPOINT_MAX_N))
            self._kp = float(msg.kp)
            self._ki = float(msg.ki)
            self._kd = float(msg.kd)
            self._target_slide_mm = float(msg.slide_dist_mm)
            self._slide_speed_mms = float(msg.speed_mms)
            if msg.approach_speed_mms > 0.0:
                v_max = max(1.0, float(msg.approach_speed_mms))
                v_min = max(0.5, v_max * 0.2)
                self.set_parameters([
                    rclpy.parameter.Parameter(
                        'approach_v_max_mms',
                        rclpy.parameter.Parameter.Type.DOUBLE, v_max),
                    rclpy.parameter.Parameter(
                        'approach_v_min_mms',
                        rclpy.parameter.Parameter.Type.DOUBLE, v_min),
                ])
            if msg.speed_factor_pct > 0.0:
                self._speed_factor_pct = float(
                    max(1.0, min(100.0, float(msg.speed_factor_pct))))
            self._repeats = int(np.clip(int(msg.repeats) or 1, 1, 100))
            slide_dir = str(msg.slide_dir).upper().strip() or '+Y'
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
            self._user_home_q = np.array(
                [math.radians(float(v)) for v in msg.home_deg],
                dtype=np.float64)
            # Estabilização do HOLD — 0.0 no msg = usar default do explorer.
            self._hold_tol_n = (float(msg.hold_tol_n)
                                if msg.hold_tol_n > 0.0 else None)
            self._hold_stable_s = (float(msg.hold_stable_s)
                                   if msg.hold_stable_s > 0.0 else None)
            self._hold_timeout_s = (float(msg.hold_timeout_s)
                                    if msg.hold_timeout_s > 0.0 else None)
        self._pause_requested.clear()
        self._protocol_thread = threading.Thread(
            target=self._run_protocol, daemon=True)
        self._protocol_thread.start()

    # ──────────────────────────────────────────────────────────────────
    # Status
    # ──────────────────────────────────────────────────────────────────
    def _publish_status(self):
        with self._lc_lock:
            force_net = self._lc_force_net
        with self._params_lock:
            depth_mm  = float(self._target_depth_mm)
            speed_mms = float(self._slide_speed_mms)
            target_f  = float(self._target_force_n)
        msg = PalpationStatus()
        msg.phase = self._phase
        msg.cycle = int(self._cycle)
        msg.cycles_total = int(self._cycles_total)
        msg.target_depth_mm = depth_mm
        msg.target_force_n = target_f
        msg.force_net_n = float(force_net)
        msg.speed_mms = speed_mms
        msg.paused = self._pause_requested.is_set()
        self._status_pub.publish(msg)

    def _fz_corrected(self) -> float:
        """Força de contato tare-compensada (N). Positivo = compressão."""
        with self._lc_lock:
            return self._lc_force_net

    def _force_stale_abort(self, phase: str) -> bool:
        """True se a leitura de força está velha/ausente — a fase chamadora
        deve abortar com outcome 'stale'. Loga o motivo uma única vez."""
        with self._lc_lock:
            ts = self._lc_force_ts
        if ts > 0.0:
            age = time.monotonic() - ts
            if age <= _FORCE_STALE_S:
                return False
            detail = f'última leitura há {age:.1f} s (> {_FORCE_STALE_S:.1f} s)'
        else:
            detail = 'nenhuma leitura recebida em /load_cell/force_net'
        self.get_logger().error(
            f'SEGURANÇA [{phase}]: célula de carga sem dados frescos — '
            f'{detail}. Controle por força não confiável; abortando. '
            'Verifique a ESP32 e o force_receiver.')
        return True

    def _pause_gate(self) -> bool:
        """Bloqueia enquanto o experimento estiver pausado, segurando a
        posição atual (re-publica o setpoint corrente como o _settle).
        Retorna False se um STOP chegar durante a pausa."""
        if not self._pause_requested.is_set():
            return True
        self.get_logger().warn('[PAUSE] experimento pausado — segurando posição.')
        q_hold = self._q_now()
        zero_vel = np.zeros(6)
        while self._pause_requested.is_set():
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                self.get_logger().warn('[PAUSE] stop durante a pausa.')
                return False
            self._stream_q(q_hold, _CTRL_LOOK + _CTRL_DT, velocities=zero_vel)
            time.sleep(_CTRL_DT)
        self.get_logger().info('[PAUSE] experimento retomado.')
        return True

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

    def _home_v_rad_s(self) -> float:
        """Velocidade máxima por junta dos retornos HOME (rad/s), saturada."""
        try:
            v = float(self.get_parameter('home_speed_rad_s').value)
        except Exception:
            v = _HOME_MAX_RAD_S
        return float(min(max(v, 0.01), 0.30))

    # ──────────────────────────────────────────────────────────────────
    # Movimento no espaço de juntas: interpola linearmente q_from → q_to
    # a velocidade máxima de home_speed_rad_s rad/s por junta.
    # ──────────────────────────────────────────────────────────────────
    def _joint_stream_to(self, q_target: np.ndarray) -> bool:
        q_from = self._q_now()
        delta = np.asarray(q_target, float) - q_from
        max_d = float(np.max(np.abs(delta)))
        if max_d < 0.001:
            return True
        n_steps = max(1, int(math.ceil(max_d / (self._home_v_rad_s() * _CTRL_DT))))
        v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S
        vel_peak = np.clip(delta / n_steps / _CTRL_DT, -v_lim, v_lim)
        # Rampa trapezoidal: ~20 % de aceleração/desaceleração (máx 8 passos = 240 ms).
        # Evita o solavanco de arranque causado por velocidade constante desde t=0.
        ramp = min(max(1, n_steps // 5), 8)
        for i in range(1, n_steps + 1):
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                return False
            alpha = i / n_steps
            q = np.clip(q_from + alpha * delta, JOINT_MIN, JOINT_MAX)
            if i <= ramp:
                scale = i / ramp
            elif i >= n_steps - ramp + 1:
                scale = (n_steps - i + 1) / ramp
            else:
                scale = 1.0
            step_vel = vel_peak * scale if i < n_steps else np.zeros(6)
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
                           force_threshold_n: float | None = None,
                           win: int = _CTRL_WIN) -> str:
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
                if self._fz_corrected() >= force_threshold_n:
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

            # ── Batch de `win` waypoints (janela deslizante) ─────────────────
            # Cada mensagem contém `win` pontos com timestamps cumulativos.
            # O JTC interpola S-curve sobre toda a janela → coordenação suave
            # de múltiplas juntas sem reinício de planejamento entre passos.
            # A janela desliza 1 passo (_CTRL_DT) por tick; a cada 30 ms
            # o batch é atualizado com o estado real lido de /joint_states.
            msg = JointTrajectory()
            msg.joint_names = list(_ARM_JOINTS)
            q_iter = q.copy()
            T_iter = T_cur
            v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S
            singular = False
            for k in range(1, win + 1):
                tw = np.zeros(6)
                tw[:3] = d * step
                if R0 is not None:
                    R_err = R0 @ T_iter[:3, :3].T
                    tw[3:] = _ORI_GAIN * 0.5 * np.array([
                        R_err[2, 1] - R_err[1, 2],
                        R_err[0, 2] - R_err[2, 0],
                        R_err[1, 0] - R_err[0, 1],
                    ])
                if z0 is not None:
                    tw[2] += _Z_CORR_GAIN * (z0 - float(T_iter[2, 3]))
                if perp_dir is not None and p0_perp is not None:
                    perp_err = p0_perp - float(T_iter[:3, 3] @ perp_dir)
                    tw[:3] += _Z_CORR_GAIN * perp_err * perp_dir
                J_k = jacobian(q_iter, T_end=T_TOUCH_TOOL_ATTACH)
                try:
                    dq_k = J_k.T @ np.linalg.solve(
                        J_k @ J_k.T + _JAC_LAM**2 * I6, tw)
                except np.linalg.LinAlgError:
                    singular = True
                    break
                q_next = np.clip(q_iter + dq_k, JOINT_MIN, JOINT_MAX)
                vel_k = np.clip((q_next - q_iter) / _CTRL_DT, -v_lim, v_lim)
                pt = JointTrajectoryPoint()
                pt.positions = [float(x) for x in q_next]
                pt.velocities = [float(x) for x in vel_k]
                t_k = k * _CTRL_DT
                pt.time_from_start = Duration(
                    sec=int(t_k), nanosec=int((t_k - int(t_k)) * 1e9))
                msg.points.append(pt)
                q_iter = q_next
                if k < win:
                    T_iter = forward_kinematics(q_iter, T_end=T_TOUCH_TOOL_ATTACH)
            if singular:
                self.get_logger().warn('Jacobiano singular — passo descartado.')
                time.sleep(_CTRL_DT)
                continue
            if msg.points:
                self._arm_traj_pub.publish(msg)
            time.sleep(_CTRL_DT)

        return 'done'

    # ──────────────────────────────────────────────────────────────────
    # Trajetória Cartesiana em batch completo (SLIDING / RETRACT).
    #
    # Pré-computa todos os N waypoints via Jacobiano iterado e os envia
    # em UMA única JointTrajectory. O JTC planeja a S-curve sobre o
    # conjunto inteiro — sem reinício a cada tick, sem instabilidade de
    # coordenação multi-junta independente da distância percorrida.
    #
    # Não monitora força: use _cartesian_stream para fases reativas.
    # Retorna: 'done' | 'stop' | 'error'
    # ──────────────────────────────────────────────────────────────────
    def _cartesian_batch_to(self, direction: np.ndarray, total_m: float, *,
                              v_const_ms: float | None = None,
                              v_max_ms: float | None = None,
                              v_min_ms: float | None = None,
                              lock_ori: bool = False,
                              lock_z: bool = False,
                              lock_perp: bool = False) -> str:
        d = np.asarray(direction, dtype=float).flatten()
        nd = float(np.linalg.norm(d))
        if nd < 1e-9 or total_m <= 0.0:
            self.get_logger().error('_cartesian_batch_to: direção/distância inválida.')
            return 'error'
        d /= nd

        constant = v_const_ms is not None
        if not constant and (v_max_ms is None or v_min_ms is None):
            self.get_logger().error(
                '_cartesian_batch_to: forneça v_const_ms OU (v_max_ms, v_min_ms).')
            return 'error'

        v_ref = float(v_const_ms) if constant else float(v_max_ms)
        v_ref = max(1e-4, v_ref)
        N = max(1, int(math.ceil(total_m / (v_ref * _CTRL_DT))))

        q = self._q_now()
        T0 = forward_kinematics(q, T_end=T_TOUCH_TOOL_ATTACH)

        R0 = T0[:3, :3].copy() if lock_ori else None
        z0 = float(T0[2, 3]) if lock_z else None
        perp_dir: np.ndarray | None = None
        p0_perp: float | None = None
        if lock_perp:
            perp = np.array([-d[1], d[0], 0.0])
            pnorm = float(np.linalg.norm(perp))
            if pnorm > 1e-9:
                perp_dir = perp / pnorm
                p0_perp = float(T0[:3, 3] @ perp_dir)

        I6 = np.eye(6)
        v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S
        self.get_logger().info(
            f'_cartesian_batch_to: pré-computando {N} waypoints '
            f'({total_m*1e3:.1f}mm @ {v_ref*1e3:.1f}mm/s) ...')

        msg = JointTrajectory()
        msg.joint_names = list(_ARM_JOINTS)
        q_iter = q.copy()
        T_iter = T0

        for k in range(1, N + 1):
            u = (k - 1) / max(1, N - 1)
            if constant:
                v_k = float(v_const_ms)
            else:
                v_k = float(v_min_ms) + (float(v_max_ms) - float(v_min_ms)) * (1.0 - u) ** 2
            v_k = max(1e-4, v_k)
            step = v_k * _CTRL_DT

            tw = np.zeros(6)
            tw[:3] = d * step
            if R0 is not None:
                R_err = R0 @ T_iter[:3, :3].T
                tw[3:] = _ORI_GAIN * 0.5 * np.array([
                    R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1],
                ])
            if z0 is not None:
                tw[2] += _Z_CORR_GAIN * (z0 - float(T_iter[2, 3]))
            if perp_dir is not None and p0_perp is not None:
                perp_err = p0_perp - float(T_iter[:3, 3] @ perp_dir)
                tw[:3] += _Z_CORR_GAIN * perp_err * perp_dir

            J_k = jacobian(q_iter, T_end=T_TOUCH_TOOL_ATTACH)
            try:
                dq_k = J_k.T @ np.linalg.solve(J_k @ J_k.T + _JAC_LAM**2 * I6, tw)
            except np.linalg.LinAlgError:
                self.get_logger().warn(f'Batch: Jacobiano singular no passo {k} — truncando.')
                break

            q_next = np.clip(q_iter + dq_k, JOINT_MIN, JOINT_MAX)
            vel_k = np.clip((q_next - q_iter) / _CTRL_DT, -v_lim, v_lim)
            if k == N:
                vel_k = np.zeros(6)

            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in q_next]
            pt.velocities = [float(x) for x in vel_k]
            t_k = k * _CTRL_DT
            pt.time_from_start = Duration(
                sec=int(t_k), nanosec=int((t_k - int(t_k)) * 1e9))
            msg.points.append(pt)
            q_iter = q_next
            T_iter = forward_kinematics(q_iter, T_end=T_TOUCH_TOOL_ATTACH)

        if not msg.points:
            return 'error'

        self._arm_traj_pub.publish(msg)
        self.get_logger().info(
            f'_cartesian_batch_to: {len(msg.points)} pts publicados '
            f'(duração {len(msg.points)*_CTRL_DT:.1f}s)')

        t_end = time.monotonic() + len(msg.points) * _CTRL_DT + 0.5
        while time.monotonic() < t_end:
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                self._settle()
                return 'stop'
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
    # HOME: trajectória batch (uma mensagem multi-ponto → JTC planeia S-curve)
    # ──────────────────────────────────────────────────────────────────
    def _joint_batch_to(self, q_target: np.ndarray) -> bool:
        """Envia uma única JointTrajectory com todos os waypoints ao JTC.

        Em vez de streaming de N goals individuais a 33 Hz (que o JTC trata
        como N trajectórias independentes), envia-os todos numa mensagem só.
        O JTC usa interpolação cúbica sobre o conjunto completo → curva de
        velocidade suave sem solavancos de arranque.

        Fallback para _joint_stream_to se a trajectória tiver < 2 pontos.
        """
        q_from = self._q_now()
        delta = np.asarray(q_target, float) - q_from
        max_d = float(np.max(np.abs(delta)))
        if max_d < 0.001:
            return True
        n_steps = max(2, int(math.ceil(max_d / (self._home_v_rad_s() * _CTRL_DT))))
        v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S
        vel_peak = np.clip(delta / n_steps / _CTRL_DT, -v_lim, v_lim)
        ramp = min(max(1, n_steps // 5), 8)

        msg = JointTrajectory()
        msg.joint_names = list(_ARM_JOINTS)
        for i in range(1, n_steps + 1):
            alpha = i / n_steps
            q = np.clip(q_from + alpha * delta, JOINT_MIN, JOINT_MAX)
            if i <= ramp:
                scale = i / ramp
            elif i >= n_steps - ramp + 1:
                scale = (n_steps - i + 1) / ramp
            else:
                scale = 1.0
            step_vel = vel_peak * scale if i < n_steps else np.zeros(6)
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q]
            pt.velocities = [float(v) for v in step_vel]
            t_s = i * _CTRL_DT
            pt.time_from_start = Duration(sec=int(t_s),
                                          nanosec=int((t_s - int(t_s)) * 1e9))
            msg.points.append(pt)

        self._arm_traj_pub.publish(msg)

        # Aguardar a execução, monitorizando stop a cada tick.
        t_end = time.monotonic() + n_steps * _CTRL_DT + 0.3
        while time.monotonic() < t_end:
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                self._settle()
                return False
            time.sleep(_CTRL_DT)
        return True

    # ──────────────────────────────────────────────────────────────────
    # Fases
    # ──────────────────────────────────────────────────────────────────
    def _phase_goto_home(self) -> bool:
        """HOME — trajectória batch ao JTC (S-curve interna) a ≤ 0.3 rad/s."""
        self._set_phase('HOME')
        self._send_hand_pose(_HAND_POINTING_RAD)

        q_home = (self._user_home_q.copy()
                  if self._user_home_q is not None
                  else _POINTING_SEED_Q.copy())

        # Settle antes de mover — garante que não há lookahead residual.
        self._settle()

        if not self._joint_batch_to(q_home):
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

    def _phase_descending(self) -> str:
        """DESCENDING — desce ao longo do approach até a força atingir o setpoint.

        Controle por força: termina quando a compressão alcança o setpoint
        do PID (force_n da GUI, ≤ 10 N). A profundidade da GUI é o curso
        máximo de segurança.

        Retorna: 'ok' (setpoint atingido) | 'no_contact' (curso esgotado)
                 | 'force' (> 15 N) | 'stale' (célula sem dados frescos)
                 | 'stop' (usuário).
        """
        self._set_phase('DESCENDING')
        self._settle()

        # Calcula approach_dir a partir da pose atual do TCP (coluna Z do frame).
        T_pre = forward_kinematics(self._q_now(), T_end=T_TOUCH_TOOL_ATTACH)
        approach_dir = T_pre[:3, 2].copy()
        if float(np.linalg.norm(approach_dir)) < 0.1:
            approach_dir = np.array([0.0, 0.0, -1.0])
        self._approach_dir = approach_dir.copy()

        with self._params_lock:
            depth_m      = float(self._target_depth_mm) / 1000.0
            target_f     = float(self._target_force_n)
            approach_mms = float(self.get_parameter('approach_v_max_mms').value)
            # Velocidade de descida = approach_v_max × speed_factor (igual ao robô em 10 %)
            speed_ms = max(0.001,
                           approach_mms * (self._speed_factor_pct / 100.0) / 1000.0)

        if depth_m <= 0.0:
            self.get_logger().warn('DESCENDING: profundidade = 0 mm — pulando fase.')
            return 'ok'
        I6    = np.eye(6)
        dt    = _CTRL_DT
        v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S

        descended_m = 0.0
        self.get_logger().info(
            f'DESCENDING: alvo={target_f:.2f} N  '
            f'curso máx={depth_m * 1000:.1f} mm  '
            f'vel={speed_ms * 1000:.1f} mm/s  '
            f'(approach={approach_mms:.0f} mm/s × {self._speed_factor_pct:.0f}%)')

        while descended_m < depth_m:
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                self.get_logger().warn('[STOP] DESCENDING interrompido pelo usuário.')
                return 'stop'
            if not self._pause_gate():
                return 'stop'

            t0 = time.time()

            if self._force_stale_abort('DESCENDING'):
                return 'stale'
            fz = self._fz_corrected()  # + compressão, − tração
            if fz > _FORCE_ABORT_LIMIT_N:
                self.get_logger().error(
                    f'SEGURANÇA: compressão {fz:.1f} N > '
                    f'{_FORCE_ABORT_LIMIT_N:.0f} N — medição cancelada.')
                return 'force'
            if fz >= target_f:
                self.get_logger().info(
                    f'DESCENDING: setpoint atingido — fz={fz:.2f} N '
                    f'(alvo {target_f:.2f} N) após '
                    f'{descended_m * 1000:.1f} mm.')
                return 'ok'

            step_m = min(speed_ms * dt, depth_m - descended_m)
            tw = np.zeros(6)
            tw[:3] = approach_dir * step_m

            q = self._q_now()
            J = jacobian(q, T_end=T_TOUCH_TOOL_ATTACH)
            try:
                dq = J.T @ np.linalg.solve(J @ J.T + _JAC_LAM**2 * I6, tw)
            except np.linalg.LinAlgError:
                time.sleep(dt)
                continue

            q_new = np.clip(q + dq, JOINT_MIN, JOINT_MAX)
            vel   = np.clip((q_new - q) / dt, -v_lim, v_lim)
            self._stream_q(q_new, dt, velocities=vel)
            descended_m += step_m

            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

        self.get_logger().warn(
            f'DESCENDING: curso máximo de {descended_m * 1000:.1f} mm esgotado '
            f'sem atingir {target_f:.2f} N (fz={self._fz_corrected():.2f} N) — '
            'abortando com retorno lento à home.')
        return 'no_contact'

    def _phase_hold(self, stable_s: float = _HOLD_STABLE_S,
                    timeout_s: float = _HOLD_TIMEOUT_S) -> str:
        """HOLD — PID de força leva a compressão ao setpoint e ESPERA a
        estabilização antes de liberar o SLIDING.

        Critério de saída: |fz − alvo| ≤ tol por `stable_s` s CONTÍNUOS
        (tol = máx(_HOLD_TOL_N, _HOLD_TOL_PCT × alvo)); sair da banda
        reinicia a janela. `timeout_s` é o teto de espera — estourou,
        prossegue com aviso (o PID do SLIDING continua corrigindo).

        Corrige ao longo do approach_dir: compressão abaixo do alvo →
        aprofunda; acima → alivia. Interrompível pelo botão Parar.

        Retorna: 'ok' | 'force' (> 15 N) | 'stale' (célula sem dados)
                 | 'stop' (usuário).
        """
        self._set_phase('HOLD')
        with self._params_lock:
            target_f = float(self._target_force_n)
            kp, ki, kd = self._kp, self._ki, self._kd
            # Overrides do PalpationStart (avançados da GUI); None = default.
            tol_override = self._hold_tol_n
            if self._hold_stable_s is not None:
                stable_s = self._hold_stable_s
            if self._hold_timeout_s is not None:
                timeout_s = self._hold_timeout_s

        tol_n = (tol_override if tol_override is not None
                 else max(_HOLD_TOL_N, _HOLD_TOL_PCT * target_f))
        approach_dir = (self._approach_dir if self._approach_dir is not None
                        else np.array([0., 0., -1.]))
        pid = _ForcePID(kp, ki, kd, _CTRL_DT)
        I6 = np.eye(6)
        dt = _CTRL_DT
        v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S

        self.get_logger().info(
            f'HOLD-PID: alvo {target_f:.2f} ± {tol_n:.2f} N  '
            f'Kp={kp:.4g} Ki={ki:.4g} Kd={kd:.4g}  '
            f'estável por {stable_s:.1f} s (timeout {timeout_s:.0f} s)')

        t_start = time.time()
        t_stable0: float | None = None   # início da janela estável corrente
        timed_out = False
        while True:
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                self.get_logger().warn('[STOP] HOLD interrompido pelo usuário.')
                return 'stop'
            if not self._pause_gate():
                return 'stop'
            t0 = time.time()

            if self._force_stale_abort('HOLD'):
                return 'stale'
            fz = self._fz_corrected()
            if fz > _FORCE_ABORT_LIMIT_N:
                self.get_logger().error(
                    f'SEGURANÇA: compressão {fz:.1f} N > '
                    f'{_FORCE_ABORT_LIMIT_N:.0f} N — medição cancelada.')
                return 'force'

            # ── Critério de estabilização do setpoint ─────────────────
            if abs(target_f - fz) <= tol_n:
                if t_stable0 is None:
                    t_stable0 = t0
                    self.get_logger().info(
                        f'HOLD-PID: dentro da banda (fz={fz:.2f} N) — '
                        f'aguardando {stable_s:.1f} s estável.')
                elif t0 - t_stable0 >= stable_s:
                    break    # setpoint estável — libera o SLIDING
            else:
                if t_stable0 is not None:
                    self.get_logger().info(
                        f'HOLD-PID: saiu da banda (fz={fz:.2f} N) — '
                        'janela de estabilidade reiniciada.')
                t_stable0 = None
            if t0 - t_start >= timeout_s:
                timed_out = True
                break

            v_cmd = pid.step(target_f - fz,
                             in_contact=(fz > _CONTACT_DETECT_N))
            tw = np.zeros(6)
            tw[:3] = approach_dir * (v_cmd * dt)

            q = self._q_now()
            J = jacobian(q, T_end=T_TOUCH_TOOL_ATTACH)
            try:
                dq = J.T @ np.linalg.solve(J @ J.T + _JAC_LAM**2 * I6, tw)
            except np.linalg.LinAlgError:
                time.sleep(dt)
                continue

            q_new = np.clip(q + dq, JOINT_MIN, JOINT_MAX)
            vel   = np.clip((q_new - q) / dt, -v_lim, v_lim)
            # time_from_start = dt (sem lookahead extra): correções de força
            # tomam efeito em 30 ms — lookahead longo atrasa o loop e o
            # integrador acumula antes do efeito (overshoot).
            self._stream_q(q_new, dt, velocities=vel)

            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

        if timed_out:
            self.get_logger().warn(
                f'HOLD-PID: timeout ({timeout_s:.0f} s) sem estabilizar — '
                f'fz={self._fz_corrected():.2f} N '
                f'(alvo {target_f:.2f} ± {tol_n:.2f} N). Prosseguindo: '
                'o PID do SLIDING continua corrigindo.')
        else:
            self.get_logger().info(
                f'HOLD-PID: setpoint estável — fz={self._fz_corrected():.2f} N '
                f'(alvo {target_f:.2f} ± {tol_n:.2f} N por {stable_s:.1f} s) '
                f'em {time.time() - t_start:.1f} s.')
        return 'ok'

    def _phase_sliding(self) -> str:
        """SLIDING — movimento lateral com PID de força simultâneo.
        Retorna: 'ok' | 'force' | 'stale' | 'error' | 'stop'.

        Streaming rolling-window (_SLIDE_WIN pts) combinando:
          • passo lateral constante em dir_world (velocidade do usuário)
          • correção PID ao longo de approach_dir (força normal alvo);
            antes do primeiro contato, lock posicional de profundidade
            (fallback — sem ele o Z deriva ao deslizar sem força medida)
          • lock de orientação e posição perpendicular

        Retorna: 'ok' | 'force' (> 15 N) | 'stop' (usuário) | 'error'.
        """
        self._set_phase('SLIDING')
        self._settle()

        with self._params_lock:
            speed_ms   = max(0.001, self._slide_speed_mms * 1e-3)
            dir_xy     = self._slide_dir_vec.copy()
            slide_lim_m = min(float(self._target_slide_mm) / 1000.0,
                              _SLIDING_SAFETY_M)
            target_f   = float(self._target_force_n)
            kp, ki, kd = self._kp, self._ki, self._kd

        approach_dir = (self._approach_dir if self._approach_dir is not None
                        else np.array([0., 0., -1.]))

        dir_world = np.array([float(dir_xy[0]), float(dir_xy[1]), 0.0])
        dn = float(np.linalg.norm(dir_world))
        if dn < 1e-9:
            self.get_logger().error('SLIDING: direção inválida.')
            return 'error'
        dir_world /= dn

        T_start = forward_kinematics(self._q_now(), T_end=T_TOUCH_TOOL_ATTACH)
        R0     = T_start[:3, :3].copy()
        p_start = T_start[:3, 3].copy()

        perp = np.array([-dir_world[1], dir_world[0], 0.0])
        pnorm = float(np.linalg.norm(perp))
        perp_dir = perp / pnorm if pnorm > 1e-9 else None
        p0_perp  = float(p_start @ perp_dir) if perp_dir is not None else None

        # Componente de approach_dir perpendicular a dir_world.
        # Quando approach_dir tem componente paralela ao deslizamento
        # (ex.: ferramenta levemente inclinada na direção Y), a correção
        # de força disputa com o movimento lateral, variando a velocidade
        # do deslizamento. Removendo essa componente, o PID de força age
        # apenas no subespaço ortogonal ao movimento lateral.
        _lat_comp = float(np.dot(approach_dir, dir_world))
        _adp = approach_dir - _lat_comp * dir_world
        _adp_norm = float(np.linalg.norm(_adp))
        approach_dir_eff = (_adp / _adp_norm) if _adp_norm > 1e-6 else approach_dir

        I6 = np.eye(6)
        v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S
        dt = _CTRL_DT
        pid = _ForcePID(kp, ki, kd, dt)

        dist_planned_m   = 0.0   # distância planejada acumulada (não depende de FK)
        step_m = speed_ms * dt   # deslocamento por tick no plano

        self.get_logger().info(
            f'SLIDING: speed={speed_ms*1e3:.1f} mm/s  '
            f'dir=({dir_world[0]:+.0f},{dir_world[1]:+.0f},0)  '
            f'alvo={slide_lim_m*1e3:.0f} mm  '
            f'força alvo={target_f:.2f} N  '
            f'Kp={kp:.4g} Ki={ki:.4g} Kd={kd:.4g}  '
            f'approach_eff=({approach_dir_eff[0]:+.3f},'
            f'{approach_dir_eff[1]:+.3f},{approach_dir_eff[2]:+.3f})')

        outcome = 'ok'
        while True:
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                outcome = 'stop'
                break
            if not self._pause_gate():
                outcome = 'stop'
                break

            t0 = time.time()

            # Verificação 1: distância planejada acumulada (determinístico).
            # Não depende de atraso do /joint_states — para assim que os
            # waypoints suficientes foram enviados ao JTC.
            if dist_planned_m >= slide_lim_m:
                self.get_logger().info(
                    f'SLIDING: {slide_lim_m*1e3:.0f} mm planejados — parando.')
                break

            # Verificação 2: posição real via FK (segurança extra).
            q = self._q_now()
            T_cur = forward_kinematics(q, T_end=T_TOUCH_TOOL_ATTACH)
            progress = float(np.dot(T_cur[:3, 3] - p_start, dir_world))
            if progress >= slide_lim_m:
                self.get_logger().info(
                    f'SLIDING: {slide_lim_m*1e3:.0f} mm (FK) atingidos.')
                break

            # PID de força (+ compressão, − tração)
            if self._force_stale_abort('SLIDING'):
                outcome = 'stale'
                break
            fz_corr = self._fz_corrected()

            if fz_corr > _FORCE_ABORT_LIMIT_N:
                self.get_logger().error(
                    f'SEGURANÇA: compressão {fz_corr:.1f} N > '
                    f'{_FORCE_ABORT_LIMIT_N:.0f} N — medição cancelada.')
                outcome = 'force'
                break

            # Correção PID calculada 1× por tick (a força não muda dentro
            # da janela de _SLIDE_WIN waypoints); aplicada por waypoint.
            v_corr = pid.step(target_f - fz_corr,
                              in_contact=(fz_corr > _CONTACT_DETECT_N))
            corr_step = v_corr * dt

            # Detecção de força perdida desativada temporariamente.
            # force_lost_count não é usado.

            # ── Rolling-window de _SLIDE_WIN waypoints ──────────────────
            msg = JointTrajectory()
            msg.joint_names = list(_ARM_JOINTS)
            q_iter = q.copy()
            T_iter = T_cur
            singular = False

            for k in range(1, _SLIDE_WIN + 1):
                tw = np.zeros(6)
                # Passo lateral — limita o último passo para não ultrapassar alvo
                remaining = max(0.0, slide_lim_m - dist_planned_m - (k - 1) * step_m)
                lateral   = min(step_m, remaining)
                tw[:3] = dir_world * lateral
                # Lock de orientação
                R_err = R0 @ T_iter[:3, :3].T
                tw[3:] = _ORI_GAIN * 0.5 * np.array([
                    R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1],
                ])
                # Correção ⊥ ao deslizamento: PID de força após o primeiro
                # contato (v_corr > 0 aprofunda, < 0 alivia); antes dele,
                # lock posicional de profundidade — sem força medida o PID
                # fica inerte e o Z derivaria.
                if pid.ever_in_contact:
                    tw[:3] += corr_step * approach_dir_eff
                else:
                    depth_err = float(np.dot(
                        p_start - T_iter[:3, 3], approach_dir_eff))
                    tw[:3] += _Z_CORR_GAIN * depth_err * approach_dir_eff
                # Lock perpendicular — sem ganho na direção transversal ao sliding
                if perp_dir is not None and p0_perp is not None:
                    perp_err = p0_perp - float(T_iter[:3, 3] @ perp_dir)
                    tw[:3] += _Z_CORR_GAIN * perp_err * perp_dir

                J_k = jacobian(q_iter, T_end=T_TOUCH_TOOL_ATTACH)
                try:
                    dq_k = J_k.T @ np.linalg.solve(
                        J_k @ J_k.T + _JAC_LAM**2 * I6, tw)
                except np.linalg.LinAlgError:
                    singular = True
                    break

                q_next = np.clip(q_iter + dq_k, JOINT_MIN, JOINT_MAX)
                vel_k  = np.clip((q_next - q_iter) / dt, -v_lim, v_lim)
                if k == _SLIDE_WIN:
                    vel_k = np.zeros(6)

                pt = JointTrajectoryPoint()
                pt.positions  = [float(x) for x in q_next]
                pt.velocities = [float(x) for x in vel_k]
                t_k = k * dt
                pt.time_from_start = Duration(
                    sec=int(t_k), nanosec=int((t_k - int(t_k)) * 1e9))
                msg.points.append(pt)
                q_iter = q_next
                if k < _SLIDE_WIN:
                    T_iter = forward_kinematics(q_iter, T_end=T_TOUCH_TOOL_ATTACH)

            if singular:
                self.get_logger().warn('SLIDING: Jacobiano singular — passo descartado.')
            elif msg.points:
                self._arm_traj_pub.publish(msg)
                dist_planned_m += len(msg.points) * step_m

            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

        self._settle()
        return outcome

    def _phase_retract(self) -> bool:
        """RETRACT — streaming em +Z a velocidade constante (slider approach)."""
        self._set_phase('RETRACT')
        self._settle()

        retract_m = float(self.get_parameter('retract_mm').value) * 1e-3
        # Mesma escala da descida: approach_v_max × speed_factor (10 % → 5 mm/s).
        v_approach = max(0.001,
                         float(self.get_parameter('approach_v_max_mms').value)
                         * (self._speed_factor_pct / 100.0) * 1e-3)

        # Retrocede na direção oposta ao approach (negativo do eixo Z do TCP).
        T_pre = forward_kinematics(self._q_now(), T_end=T_TOUCH_TOOL_ATTACH)
        retract_dir = -T_pre[:3, 2].copy()
        if float(np.linalg.norm(retract_dir)) < 0.1:
            retract_dir = np.array([0.0, 0.0, +1.0])

        outcome = self._cartesian_batch_to(
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

    def _retreat_and_home(self, final_phase: str) -> None:
        """Afasta da superfície (RETRACT) e retorna à home lentamente.

        Usado apenas no término com SUCESSO. Velocidades:
        retract = approach × speed_factor; home = home_speed_rad_s por junta.
        """
        self._phase_retract()
        self._phase_goto_home()
        self._set_phase(final_phase)

    def _abort_to_home(self) -> None:
        """Falha do experimento (qualquer motivo): sem RETRACT — retorna
        direto à home lentamente (≤ home_speed_rad_s por junta) e marca
        ABORTED."""
        self._phase_goto_home()
        self._set_phase('ABORTED')

    def _run_protocol(self):
        self._busy.set()
        try:
            with self._params_lock:
                repeats = int(self._repeats)
            self._cycles_total = repeats

            for cycle in range(1, repeats + 1):
                self._cycle = cycle
                if repeats > 1:
                    self.get_logger().info(
                        f'[CICLO] experimento {cycle}/{repeats}')

                if not self._phase_goto_home():
                    self._set_phase('ABORTED'); return

                out = self._phase_descending()
                if out in ('force', 'no_contact', 'stale'):
                    self._abort_to_home(); return
                if out != 'ok':   # stop do usuário → para no lugar
                    self._set_phase('ABORTED'); return

                out = self._phase_hold()
                if out in ('force', 'stale'):
                    self._abort_to_home(); return
                if out != 'ok':
                    self._set_phase('ABORTED'); return

                out = self._phase_sliding()
                if out in ('force', 'error', 'stale'):
                    self._abort_to_home(); return
                if out != 'ok':
                    self._set_phase('ABORTED'); return

                if cycle < repeats:
                    # Entre ciclos: só recua da superfície — o próximo
                    # ciclo refaz o HOME (e a re-aproximação) sozinho.
                    if not self._phase_retract():
                        self._set_phase('ABORTED'); return
                    # Stop pedido durante o RETRACT → não inicia o próximo.
                    if self._stop_requested.is_set():
                        self._stop_requested.clear()
                        self._phase_goto_home()
                        self._set_phase('ABORTED'); return

            self._retreat_and_home('DONE')
            time.sleep(0.5)
            self._set_phase('IDLE')
        finally:
            self._cycle = 0
            self._cycles_total = 1
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
