"""
palpation_gui.py — Painel Tkinter (tema claro) da célula de palpação tátil.

Funcionalidades:
  • Spinbox + Slider sincronizados para Velocidade / Força / Distância.
  • Botão "▶ Iniciar Palpação" (publica /palpation/start).
  • Feedback em tempo real de /ft_sensor/wrench (semáforo OK/WARN/DANGER).
  • Indicador de fase (IDLE/CONTACT/HOLD/SLIDING/RETRACT/DONE/ABORTED).
  • Painel de conexão à MÃO COVVI real (IP + Conectar + ECI + PWR)
    — sobe o subprocesso `covvi_hand_driver server <IP>` e ativa o ECI.
  • Painel de conexão ao ROBÔ CR10 real (IP + Conectar + dropdown de modo
    SIM_ONLY / MIRROR / REAL_FROM_SIM) — abre as 3 sockets TCP do
    controlador e executa a sequência ClearError + EnableRobot.
  • Botão ⏹ E-STOP — chama StopRobot+DisableRobot e abre a mão.

Comunicação ROS:
  pub  /palpation/start    std_msgs/String   JSON {force_n, speed_mms, distance_mm}
  sub  /palpation/status   std_msgs/String   JSON {phase, measured_force_normal_n,...}
  sub  /ft_sensor/wrench   geometry_msgs/WrenchStamped
  cli  covvi_interfaces/SetCurrentGrip   (lazy)
  cli  covvi_interfaces/SetHandPowerOn   (lazy)
  cli  covvi_interfaces/SetHandPowerOff  (lazy)
"""
from __future__ import annotations

import collections
import json
import logging
import math
import os
import re
import signal
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy,
)

from std_msgs.msg import String, Float32
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# QoS para comando crítico (/palpation/start): RELIABLE + TRANSIENT_LOCAL
# faz com que o último start fique persistido — se o explorer subir
# depois da GUI publicar, ele ainda recebe o último comando.
QOS_COMMAND = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)
# QoS para stream de sensor (/ft_sensor/wrench): BEST_EFFORT + depth=1
# minimiza latência e nunca trava o publisher por reentrega — só o
# pacote mais recente importa para o PID de força.
QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)

# Driver TCP/IP do CR10 real (cabeada via 192.168.5.1 / LAN1).
try:
    from .real_driver import (
        CR10RealDriver, CR10RealDriverConfig, CR10RealDriverError,
    )
    from .kinematics import urdf_to_dobot as _urdf_to_dobot, MIMIC_LIST
    _REAL_DRIVER_OK = True
except Exception:  # pragma: no cover
    CR10RealDriver = None
    CR10RealDriverConfig = None
    CR10RealDriverError = Exception
    _urdf_to_dobot = None
    MIMIC_LIST = []
    _REAL_DRIVER_OK = False


log = logging.getLogger('touch_pack.palpation_gui')

# ──────────────────────────────────────────────────────────────────────
# Tema claro (consistente com a paleta do laboratório).
# ──────────────────────────────────────────────────────────────────────
BG          = '#f1f5f9'
PANEL       = '#ffffff'
HEADER      = '#1d4ed8'
HEADER_FG   = 'white'
TEXT        = '#0f172a'
TEXT_MUTED  = '#475569'
TEXT_DIM    = '#94a3b8'
PRIMARY     = '#2563eb'
PRIMARY_HV  = '#1d4ed8'
OK          = '#16a34a'
WARN        = '#d97706'
DANGER      = '#dc2626'
DANGER_HV   = '#b91c1c'
BORDER      = '#cbd5e1'
BTN_NEUTRAL = '#e2e8f0'

FONT_TITLE  = ('Segoe UI', 18, 'bold')
FONT_HEAD   = ('Segoe UI', 12, 'bold')
FONT_LBL    = ('Segoe UI', 11)
FONT_SMALL  = ('Segoe UI', 10)
FONT_BIG    = ('Segoe UI', 26, 'bold')
FONT_MONO   = ('JetBrains Mono', 11)
FONT_MONO_S = ('JetBrains Mono', 10)

# Faixas dos parâmetros — adequadas ao protocolo Gupta et al. 2021.
SPEED_MIN, SPEED_MAX, SPEED_DEFAULT = 1.0,  30.0,  10.0    # mm/s
FORCE_MIN, FORCE_MAX, FORCE_DEFAULT = 0.2,   5.0,   1.0    # N
DIST_MIN,  DIST_MAX,  DIST_DEFAULT  = 10.0, 200.0,  90.0   # mm
# Distância em CENTÍMETROS entre o dedo (TCP na Home customizada) e a
# superfície de palpação. Usada na fase CONTACT para definir até onde o
# braço desce em -Z (ou pára antes se a força normal alvo for atingida).
# Limite superior estendido para além de 20 cm — o alvo de palpação fica
# bem abaixo da palma na Home customizada e o usuário pode ajustar a
# altura da superfície/alvo.
TGT_DIST_CM_MIN, TGT_DIST_CM_MAX, TGT_DIST_CM_DEFAULT = 0.5, 60.0, 5.0  # cm
# Velocidade de aproximação (CONTACT/RETRACT) — perfil fast→slow no
# explorer usa este valor como max; min é derivado como 20 % do max.
APPROACH_MIN, APPROACH_MAX, APPROACH_DEFAULT = 5.0, 100.0, 50.0  # mm/s

SPEED_FACTOR_MIN, SPEED_FACTOR_MAX, SPEED_FACTOR_DEFAULT = 1, 100, 10  # %
# At SPEED_FACTOR_DEFAULT (10 %), Gazebo trajectory duration = 3.0 s.
# Scales inversely: 100 % → 0.3 s, 1 % → 30 s.
_VEL_BASE_S = 3.0   # duration at 10 %

# Ganhos PID padrão do controle de força durante o HOLD. v_cmd (m/s) sai
# do PID a partir do erro em N; sintonize na própria GUI durante a
# calibração — defaults conservadores por segurança.
KP_DEFAULT, KP_MIN, KP_MAX, KP_STEP = 0.0010, 0.0, 0.020,  0.0005   # (m/s)/N
KI_DEFAULT, KI_MIN, KI_MAX, KI_STEP = 0.0000, 0.0, 0.010,  0.0002   # (m/s)/(N·s)
KD_DEFAULT, KD_MIN, KD_MAX, KD_STEP = 0.0000, 0.0, 0.005,  0.0001   # m/N

# Período de publicação do bridge de força (real CR10 → /ft_sensor/wrench).
FORCE_BRIDGE_PERIOD_S = 0.020   # 50 Hz

# ──────────────────────────────────────────────────────────────────────
# Controle Manual — definições do braço CR10 e da mão COVVI
# ──────────────────────────────────────────────────────────────────────
import math as _math   # alias para evitar sombrear `math` global do escopo

# Juntas do braço (URDF). Faixa de slider em graus.
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
ARM_LIMITS_DEG = {
    'joint1': (-180, 180), 'joint2': (-180, 180), 'joint3': (-160, 160),
    'joint4': (-180, 180), 'joint5': (-180, 180), 'joint6': (-180, 180),
}
ARM_HOME_DEG = {'joint1': 0, 'joint2': 0, 'joint3': -90,
                 'joint4': 0, 'joint5': 90, 'joint6': 0}
# Arquivo persistente para a home customizada do usuário. Sobrescreve
# `ARM_HOME_DEG` em runtime quando existe.
HOME_POSE_FILE = os.path.expanduser('~/.config/touch_pack/home_pose.json')
# Arquivo persistente com IPs e último modo. Carregado no __init__ e
# reescrito sempre que o usuário conectar com sucesso ou trocar de modo.
ROBOT_CONFIG_FILE = os.path.expanduser('~/.config/touch_pack/robot.json')
LC_CALIB_FILE     = os.path.expanduser('~/.config/touch_pack/load_cell_calib.json')
POSES_FILE        = os.path.expanduser('~/.config/touch_pack/poses.json')
ROBOT_CONFIG_DEFAULTS = {
    'hand_ip':    '192.168.5.103',
    'robot_ip':   '192.168.5.2',
    'robot_mode': 'SIM_ONLY',
}

# Juntas primárias da mão COVVI. Faixa de slider em graus → rad.
HAND_JOINTS = ['Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Rotate']
HAND_LIMITS_DEG = {
    'Thumb':  (0, 90), 'Index':  (0, 90), 'Middle': (0, 90),
    'Ring':   (0, 90), 'Little': (0, 90), 'Rotate': (0, 60),
}
HAND_OPEN_DEG  = {j: 0 for j in HAND_JOINTS}
HAND_CLOSE_DEG = {'Thumb': 70, 'Index': 80, 'Middle': 80,
                  'Ring':  80, 'Little': 80, 'Rotate': 0}
# Pose POINTING (apontar com Index) — coerente com _HAND_POINTING_RAD do
# tactile_explorer; é a configuração da mão durante a palpação.
HAND_POINT_DEG = {'Thumb': 30, 'Index': 0, 'Middle': 80,
                  'Ring':  80, 'Little': 80, 'Rotate': 0}

# MIMIC_LIST centralizada em kinematics.py (importada acima junto com
# urdf_to_dobot). Se o import falhar, definimos lista vazia — a expansão
# de juntas mimic vira no-op em vez de derrubar a GUI inteira.


def _shade(hex_color: str, factor: float) -> str:
    """Clareia/escurece uma cor hex (factor positivo = mais claro)."""
    h = hex_color.lstrip('#')
    r, g, b = (int(h[i:i + 2], 16) for i in (0, 2, 4))
    if factor >= 0:
        r = int(r + (255 - r) * factor)
        g = int(g + (255 - g) * factor)
        b = int(b + (255 - b) * factor)
    else:
        f = 1.0 + factor
        r = int(r * f); g = int(g * f); b = int(b * f)
    return f'#{r:02x}{g:02x}{b:02x}'


def _hdr_btn(parent, icon: str, label: str, command, *,
              bg=BTN_NEUTRAL, fg=TEXT, font=FONT_LBL, padx=12, pady=5):
    """Botão estilizado da barra superior — ícone Unicode + label,
    com troca dinâmica de estado via `btn.set_state(icon, label, bg, fg)`."""
    state = {'bg': bg, 'fg': fg}
    text = f' {icon}  {label} ' if icon else f' {label} '
    btn = tk.Button(parent, text=text, command=command,
                    bg=bg, fg=fg,
                    activebackground=_shade(bg, -0.08),
                    activeforeground=fg,
                    relief='flat', bd=0, padx=padx, pady=pady,
                    font=font, cursor='hand2',
                    highlightthickness=0)
    btn.bind('<Enter>',
              lambda _e: btn.config(bg=_shade(state['bg'], -0.08)))
    btn.bind('<Leave>',
              lambda _e: btn.config(bg=state['bg']))

    def set_state(icon: str, label: str, bg: str, fg: str = 'white'):
        state['bg'] = bg; state['fg'] = fg
        new = f' {icon}  {label} ' if icon else f' {label} '
        btn.config(text=new, bg=bg, fg=fg,
                    activebackground=_shade(bg, -0.08),
                    activeforeground=fg)
    btn.set_state = set_state  # type: ignore[attr-defined]
    return btn


# ──────────────────────────────────────────────────────────────────────
# Nó ROS + GUI
# ──────────────────────────────────────────────────────────────────────
class PalpationGUI(Node):

    def __init__(self):
        super().__init__('palpation_gui')

        # ─── Comunicação ROS (palpation/wrench) ───────────────────────
        self._start_pub = self.create_publisher(
            String, '/palpation/start', QOS_COMMAND)
        self._stop_pub = self.create_publisher(
            String, '/palpation/stop', 10)
        self.create_subscription(
            String, '/palpation/status', self._cb_status, 10)
        self.create_subscription(
            WrenchStamped, '/ft_sensor/wrench', self._cb_wrench, QOS_SENSOR)
        # Bridge real-CR10 → /ft_sensor/wrench: a thread `_force_bridge_loop`
        # lê `read_tcp_force()` do driver (estimado por torques articulares
        # compensados pela dinâmica) e publica como WrenchStamped no mesmo
        # tópico que o explorer e o painel da GUI já consomem — ou seja,
        # a leitura da última junta do robô espelha automaticamente para
        # a tela e para o PID do HOLD.
        self._wrench_pub = self.create_publisher(
            WrenchStamped, '/ft_sensor/wrench', QOS_SENSOR)

        # ─── Publishers para comando direto (aba Controle Manual) ────
        # Os joint_trajectory_controllers expõem um tópico direto
        # `<controller>/joint_trajectory` (além da action).
        self._arm_pub = self.create_publisher(
            JointTrajectory,
            '/cr10_group_controller/joint_trajectory', 5)
        self._hand_pub = self.create_publisher(
            JointTrajectory,
            '/hand_position_controller/joint_trajectory', 5)
        self._suppressing = False   # evita loop ao atualizar sliders

        # ─── Estado partilhado (Tk ↔ ROS) ────────────────────────────
        self._lock = threading.Lock()
        self._latest_phase: str = 'IDLE'
        self._latest_force_normal: float = 0.0
        self._latest_force_mag: float = 0.0
        self._fx = self._fy = self._fz = 0.0
        self._last_wrench_ts: float = 0.0
        # Cronômetro de fase: marca quando a fase atual começou (wall-clock)
        # e a duração esperada (em segundos) — usada pela progress bar para
        # SLIDING (distance/speed) e HOLD (hold_seconds publicado pelo
        # explorer). Para fases sem duração conhecida (CONTACT/RETRACT) a
        # barra mostra modo indeterminado.
        self._phase_t_start: float = time.time()
        self._latest_speed_mms: float = SPEED_DEFAULT
        self._latest_distance_mm: float = DIST_DEFAULT
        self._latest_hold_seconds: float = 5.0

        # ─── Mão COVVI (lazy) ────────────────────────────────────────
        self._hand_proc: subprocess.Popen | None = None
        # Indica intenção do usuário: True entre clicar Conectar e
        # clicar Desconectar. Watchdog usa esse flag para distinguir
        # morte indesejada (re-spawn) de saída esperada (no-op).
        self._hand_should_be_alive: bool = False
        self._hand_watchdog_thread: threading.Thread | None = None
        self._hand_watchdog_stop = threading.Event()
        self._eci_enabled = False
        self._eci_prefix = self.declare_parameter(
            'eci_prefix', '/covvi/hand').value
        self._param_robot_ip   = self.declare_parameter('robot_ip',   '').value
        self._param_robot_mode = self.declare_parameter('robot_mode', '').value
        self._eci_srv = None
        self._eci_msg = None
        self._cli_eci_grip = None
        self._cli_eci_posn = None
        self._cli_hand_pwr_on = None
        self._cli_hand_pwr_off = None
        self._hand_powered = False
        self._eci_posn_after: str | None = None

        # ─── CR10 real (lazy) ────────────────────────────────────────
        self._real_driver = None    # CR10RealDriver | None
        self._real_lock = threading.Lock()
        self._robot_mode: str = 'SIM_ONLY'
        self._robot_connected: bool = False
        self._robot_connecting: bool = False
        # Heartbeat + reconexão automática do braço — detecta perda de
        # comunicação com o controlador CR10 e tenta reabrir os sockets
        # com backoff exponencial. Iniciados em `_finish_robot_connect`.
        self._robot_heartbeat_thread: threading.Thread | None = None
        self._robot_heartbeat_stop = threading.Event()
        self._robot_reconnect_thread: threading.Thread | None = None
        self._robot_reconnecting: bool = False

        # Mirror MovJ — em modo MIRROR, cada nova trajetória publicada em
        # /cr10_group_controller/joint_trajectory dispara um MovJ(joint={...})
        # para o braço real, usando o ÚLTIMO ponto da trajetória (o alvo).
        # Cobre tanto os sliders manuais quanto a palpação autônoma do
        # tactile_explorer, porque ambos publicam nesse mesmo tópico.
        # Debounce de 80 ms para coalescer publicações em rajada.
        self._mirror_timer: threading.Timer | None = None
        self._mirror_timer_lock = threading.Lock()
        self._mirror_last_target: np.ndarray | None = None
        self._force_bridge_thread: threading.Thread | None = None
        self._force_bridge_stop = threading.Event()
        # Poll loop a 33 Hz: lê /joint_states (posição simulada) e espelha
        # para o braço real via MovJ. Captura TANTO sliders manuais QUANTO
        # trajetórias via action server (tactile_explorer), ao contrário de
        # _cb_arm_trajectory que só vê publicações diretas no tópico.
        # NOTA: a thread é iniciada APÓS _stop_event ser criado (fim do __init__).
        self._latest_joint_rad: list[float] | None = None
        self._mirror_poll_thread: threading.Thread | None = None
        # Subscrição na trajetória comandada (não na pose medida do sim):
        # captura sliders manuais e palpação autônoma com a mesma latência,
        # sem competir com /joint_states (que lagga atrás do comando).
        self.create_subscription(
            JointTrajectory,
            '/cr10_group_controller/joint_trajectory',
            self._cb_arm_trajectory, 1)  # depth=1: só o setpoint mais recente
        # /joint_states: posição real (simulada) do braço — usado pelo
        # mirror poll loop para capturar palpação via action server.
        self.create_subscription(
            JointState, '/joint_states', self._cb_joint_states, 5)
        self.create_subscription(
            Float32, '/load_cell/voltage', self._cb_lc_voltage, QOS_SENSOR)

        # ─── Home pose customizável ──────────────────────────────────
        # Default (ARM_HOME_DEG) é sobrescrito se ~/.config/touch_pack/
        # home_pose.json existir. Atualizado pelo botão "💾 Salvar Home".
        self._arm_home_deg: dict[str, float] = dict(ARM_HOME_DEG)
        self._load_home_pose()

        # IPs e modo persistidos — carregar antes da UI para os defaults
        # dos campos refletirem o último valor usado.
        self._robot_cfg: dict[str, str] = dict(ROBOT_CONFIG_DEFAULTS)
        self._load_robot_config()
        # Parâmetros ROS sobrescrevem robot.json (permitem override via launch/CLI).
        if self._param_robot_ip:
            self._robot_cfg['robot_ip'] = self._param_robot_ip
        if self._param_robot_mode in ('SIM_ONLY', 'MIRROR', 'REAL_FROM_SIM'):
            self._robot_cfg['robot_mode'] = self._param_robot_mode

        # ─── Célula de carga (load cell UDP via force_receiver_node) ─
        self._lc_voltage: float          = 0.0
        self._lc_voltage_buf: collections.deque = collections.deque(maxlen=50)
        self._lc_last_ts: float          = 0.0
        self._lc_calibrated: bool        = False
        self._lc_calib_slope: float      = 0.4490
        self._lc_calib_intercept: float  = 0.0017
        self._lc_calib_n_pts: int        = 0
        self._lc_calib_points: list      = []   # (mass_kg, v_sensor) — wizard
        self._lc_zero_voltage: float | None = None  # V capturado sem força (âncora do zero)
        self._load_lc_calib()
        # Tare: tensão capturada com o sensor descarregado; subtrai o offset
        # residual que faz o zero não bater após a calibração.
        self._lc_tare_voltage: float = 0.0
        self._lc_tare_done: bool = False
        # Subprocesso do force_receiver_node (gerenciado pelo botão Conectar)
        self._force_rx_proc: subprocess.Popen | None = None
        self._force_rx_should_be_alive: bool = False

        # ─── Poses & Movimentos ──────────────────────────────────────
        self._poses: list[dict] = []        # [{id, name, q_deg:[6]}]
        self._movements: list[dict] = []    # [{id, name, pose_ids, speed_pct, dur_s}]
        self._next_pose_id: int = 1
        self._next_movement_id: int = 1
        self._drag_enabled: bool = False
        self._exec_stop = threading.Event()
        self._exec_thread: threading.Thread | None = None
        self._exec_movement_id: int | None = None
        # Refs de widgets (preenchidos em _build_poses_tab)
        self._poses_lbx: tk.Listbox | None = None
        self._movs_lbx: tk.Listbox | None = None
        self._mov_detail_outer: tk.Frame | None = None
        self._mov_detail_inner: tk.Frame | None = None
        self._drag_btn = None
        self._load_poses_data()

        # ─── Tkinter root ────────────────────────────────────────────
        self.root = tk.Tk()
        self.root.withdraw()
        self._build_ui()
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self.root.deiconify()

        # ROS spin em thread separada (Tk mainloop no thread principal).
        self._stop_event = threading.Event()
        self._spin_thread = threading.Thread(
            target=self._spin_ros, daemon=True)
        self._spin_thread.start()
        # Poll loop iniciado AQUI para garantir que _stop_event já existe.
        self._mirror_poll_thread = threading.Thread(
            target=self._mirror_poll_loop, daemon=True, name='mirror-poll')
        self._mirror_poll_thread.start()

        self.root.after(100, self._refresh_status_panel)

    # ──────────────────────────────────────────────────────────────────
    # UI construction
    # ──────────────────────────────────────────────────────────────────
    def _build_ui(self):
        self.root.title('Palpação Tátil — touch_pack')
        self.root.configure(bg=BG)
        # Janela pode encolher bastante; o corpo das abas usa scroll vertical
        # quando o conteúdo for maior que a área visível.
        self.root.minsize(720, 460)

        style = ttk.Style()
        style.theme_use('clam')
        style.configure('Tactile.Horizontal.TScale',
                         background=PANEL, troughcolor=BORDER)

        self._build_header()
        self._build_body()
        self._build_statusbar()

    # ── Header com título + painéis de conexão + E-STOP ─────────────
    def _build_header(self):
        hdr = tk.Frame(self.root, bg=HEADER, height=128)
        hdr.pack(fill='x', side='top')
        hdr.pack_propagate(False)

        # Linha 1: título à esquerda, E-STOP à direita ────────────────
        top = tk.Frame(hdr, bg=HEADER)
        top.pack(fill='x', padx=18, pady=(8, 0))

        title_box = tk.Frame(top, bg=HEADER)
        title_box.pack(side='left')
        tk.Label(title_box, text='Palpação Tátil', font=FONT_TITLE,
                 bg=HEADER, fg=HEADER_FG).pack(anchor='w')
        tk.Label(title_box,
                 text='CR10 + COVVI Index — Gupta et al. 2021',
                 font=FONT_SMALL, bg=HEADER, fg='#cbd5e1').pack(anchor='w')

        estop = _hdr_btn(top, '⏹', 'E-STOP', self._estop,
                          bg=DANGER, fg='white',
                          font=('Segoe UI', 11, 'bold'),
                          padx=20, pady=10)
        estop.bind('<Enter>',
                    lambda e, b=estop: b.config(bg=DANGER_HV), add='+')
        estop.bind('<Leave>',
                    lambda e, b=estop: b.config(bg=DANGER), add='+')
        estop.pack(side='right', padx=(12, 0))

        # Linha 2: painéis de conexão (mão à esquerda, robô à direita) ─
        mid = tk.Frame(hdr, bg=HEADER)
        mid.pack(fill='x', padx=18, pady=(8, 10))

        # ── COVVI HAND ────────────────────────────────────────────────
        conn = tk.Frame(mid, bg=HEADER)
        conn.pack(side='left')

        tk.Label(conn, text='MÃO COVVI', font=FONT_SMALL,
                 bg=HEADER, fg='#cbd5e1'
                 ).grid(row=0, column=0, columnspan=5, sticky='w',
                        pady=(0, 2))
        tk.Label(conn, text='IP:', font=FONT_LBL,
                 bg=HEADER, fg=HEADER_FG
                 ).grid(row=1, column=0, sticky='w', padx=(0, 6))
        self._hand_ip_var = tk.StringVar(value=self._robot_cfg['hand_ip'])
        tk.Entry(conn, textvariable=self._hand_ip_var,
                  width=16, font=FONT_MONO_S, bg='white', fg=TEXT,
                  relief='flat', bd=0, highlightthickness=1,
                  highlightbackground=BORDER, highlightcolor=PRIMARY,
                  justify='center'
                  ).grid(row=1, column=1, padx=(0, 8), ipady=4, sticky='w')
        self._hand_connect_btn = _hdr_btn(
            conn, '⚡', 'Conectar', self._connect_real_hand,
            bg=PRIMARY, fg='white', font=FONT_LBL, padx=14, pady=5)
        self._hand_connect_btn.grid(row=1, column=2, sticky='w', padx=(0, 8))
        self._eci_btn = _hdr_btn(
            conn, '◉', 'ECI OFF', self._toggle_eci,
            bg=BTN_NEUTRAL, fg=TEXT, font=FONT_SMALL, padx=12, pady=5)
        self._eci_btn.grid(row=1, column=3, sticky='w')
        self._pwr_btn = _hdr_btn(
            conn, '⏻', 'PWR OFF', self._toggle_hand_power,
            bg=BTN_NEUTRAL, fg=TEXT, font=FONT_SMALL, padx=12, pady=5)
        self._pwr_btn.grid(row=1, column=4, sticky='w', padx=(6, 0))

        # ── ROBÔ CR10 ─────────────────────────────────────────────────
        conn_rob = tk.Frame(mid, bg=HEADER)
        conn_rob.pack(side='left', padx=(28, 0))

        tk.Label(conn_rob, text='ROBÔ CR10', font=FONT_SMALL,
                 bg=HEADER, fg='#cbd5e1'
                 ).grid(row=0, column=0, columnspan=4, sticky='w',
                        pady=(0, 2))
        tk.Label(conn_rob, text='IP:', font=FONT_LBL,
                 bg=HEADER, fg=HEADER_FG
                 ).grid(row=1, column=0, sticky='w', padx=(0, 6))
        self._robot_ip_var = tk.StringVar(value=self._robot_cfg['robot_ip'])
        tk.Entry(conn_rob, textvariable=self._robot_ip_var,
                  width=14, font=FONT_MONO_S, bg='white', fg=TEXT,
                  relief='flat', bd=0, highlightthickness=1,
                  highlightbackground=BORDER, highlightcolor=PRIMARY,
                  justify='center'
                  ).grid(row=1, column=1, padx=(0, 8), ipady=4, sticky='w')
        self._robot_connect_btn = _hdr_btn(
            conn_rob, '⚡', 'Conectar', self._connect_real_robot,
            bg=PRIMARY, fg='white', font=FONT_LBL, padx=14, pady=5)
        self._robot_connect_btn.grid(row=1, column=2, sticky='w', padx=(0, 8))
        self._robot_mode_var = tk.StringVar(value=self._robot_cfg['robot_mode'])
        # `_robot_mode` (estado interno) deve seguir o valor carregado.
        self._robot_mode = self._robot_cfg['robot_mode']
        mode_menu = tk.OptionMenu(
            conn_rob, self._robot_mode_var,
            'SIM_ONLY', 'MIRROR',
            command=self._set_robot_mode)
        mode_menu.config(bg=BTN_NEUTRAL, fg=TEXT, font=FONT_SMALL,
                          relief='flat', highlightthickness=0,
                          activebackground=PRIMARY,
                          activeforeground='white',
                          padx=8, pady=2)
        mode_menu['menu'].config(bg=PANEL, fg=TEXT, font=FONT_SMALL,
                                   activebackground=PRIMARY,
                                   activeforeground='white')
        mode_menu.grid(row=1, column=3, sticky='w')

        # ── ESP32 / LOAD CELL ─────────────────────────────────────────────
        conn_esp = tk.Frame(mid, bg=HEADER)
        conn_esp.pack(side='left', padx=(28, 0))
        tk.Label(conn_esp, text='LOAD CELL (ESP32)', font=FONT_SMALL,
                 bg=HEADER, fg='#cbd5e1'
                 ).grid(row=0, column=0, columnspan=2, sticky='w', pady=(0, 2))
        self._esp32_dot_lbl = tk.Label(
            conn_esp, text='●', font=FONT_LBL, bg=HEADER, fg=TEXT_DIM)
        self._esp32_dot_lbl.grid(row=1, column=0, sticky='w')
        self._esp32_status_lbl = tk.Label(
            conn_esp, text='OFFLINE', font=FONT_LBL, bg=HEADER, fg=TEXT_DIM)
        self._esp32_status_lbl.grid(row=1, column=1, sticky='w', padx=(4, 0))

    # ── Corpo: Notebook com 2 abas ───────────────────────────────────
    def _build_body(self):
        # Estilo das abas no tema claro
        style = ttk.Style()
        style.configure('Tactile.TNotebook', background=BG, borderwidth=0)
        style.configure('Tactile.TNotebook.Tab',
                         background=BTN_NEUTRAL, foreground=TEXT,
                         padding=(18, 8), font=FONT_LBL, borderwidth=0)
        style.map('Tactile.TNotebook.Tab',
                   background=[('selected', PANEL)],
                   foreground=[('selected', PRIMARY)])

        nb = ttk.Notebook(self.root, style='Tactile.TNotebook')
        nb.pack(fill='both', expand=True, padx=18, pady=18)

        tab_palp  = tk.Frame(nb, bg=BG)
        tab_man   = tk.Frame(nb, bg=BG)
        tab_lc    = tk.Frame(nb, bg=BG)
        tab_poses = tk.Frame(nb, bg=BG)
        nb.add(tab_palp,  text='Palpação')
        nb.add(tab_man,   text='Controle Manual')
        nb.add(tab_lc,    text='Célula de Carga')
        nb.add(tab_poses, text='Poses & Movimentos')

        # ttk.Progressbar foi removida (causava segfault com Canvas embed).
        # _scrollable agora é seguro para todas as abas.
        self._build_palpation_tab(self._scrollable(tab_palp))
        self._build_manual_tab(self._scrollable(tab_man))
        self._build_loadcell_tab(tab_lc)   # sub-abas são scrolláveis internamente
        self._build_poses_tab(tab_poses)   # layout próprio — sem _scrollable externo

    def _scrollable(self, parent: tk.Frame) -> tk.Frame:
        """Envolve `parent` num Canvas com scrollbar vertical e retorna o
        Frame interno onde o caller deve montar o conteúdo. A largura do
        frame interno acompanha a largura do canvas (responsivo) e a
        scrollregion atualiza quando o conteúdo cresce/encolhe.
        """
        canvas = tk.Canvas(parent, bg=BG, highlightthickness=0,
                            borderwidth=0)
        vbar = ttk.Scrollbar(parent, orient='vertical',
                              command=canvas.yview)
        canvas.configure(yscrollcommand=vbar.set)
        canvas.pack(side='left', fill='both', expand=True)
        vbar.pack(side='right', fill='y')

        inner = tk.Frame(canvas, bg=BG)
        win = canvas.create_window((0, 0), window=inner, anchor='nw')

        def _on_inner(_e):
            canvas.configure(scrollregion=canvas.bbox('all'))
        inner.bind('<Configure>', _on_inner)

        def _on_canvas(e):
            canvas.itemconfigure(win, width=e.width)
        canvas.bind('<Configure>', _on_canvas)

        # Mousewheel só rola se o ponteiro estiver sobre este canvas — bind
        # local via <Enter>/<Leave> evita capturar scroll de outras abas.
        def _wheel(e):
            delta = 1 if e.num == 5 or e.delta < 0 else -1
            canvas.yview_scroll(delta, 'units')
        canvas.bind('<Enter>',
                     lambda _e: (canvas.bind_all('<MouseWheel>', _wheel),
                                  canvas.bind_all('<Button-4>', _wheel),
                                  canvas.bind_all('<Button-5>', _wheel)))
        canvas.bind('<Leave>',
                     lambda _e: (canvas.unbind_all('<MouseWheel>'),
                                  canvas.unbind_all('<Button-4>'),
                                  canvas.unbind_all('<Button-5>')))
        return inner

    def _build_palpation_tab(self, root: tk.Frame):
        body = tk.Frame(root, bg=BG)
        body.pack(fill='both', expand=True)

        col_left  = tk.Frame(body, bg=BG)
        col_right = tk.Frame(body, bg=BG)
        col_left.pack(side='left', fill='both', expand=True, padx=(0, 9))
        col_right.pack(side='left', fill='both', expand=True, padx=(9, 0))

        params_card = self._card(col_left, 'Parâmetros da Palpação')

        self.speed_var       = tk.DoubleVar(value=SPEED_DEFAULT)
        self.force_var       = tk.DoubleVar(value=FORCE_DEFAULT)
        self.dist_var        = tk.DoubleVar(value=DIST_DEFAULT)
        self.target_dist_var = tk.DoubleVar(value=TGT_DIST_CM_DEFAULT)
        # approach speed exposta via parâmetro ROS (não via slider Tk) —
        # adicionar mais um Spinbox+Scale na lista estava sendo associado
        # ao segfault do Tk.
        self.approach_var    = tk.DoubleVar(value=APPROACH_DEFAULT)
        # Ganhos PID enviados a cada /palpation/start; o explorer aplica
        # no HOLD para manter a força normal alvo. Padrão conservador.
        self.pid_kp_var      = tk.DoubleVar(value=KP_DEFAULT)
        self.pid_ki_var      = tk.DoubleVar(value=KI_DEFAULT)
        self.pid_kd_var      = tk.DoubleVar(value=KD_DEFAULT)
        # Direção XY (mundo) do sliding. O explorer escolhe o sinal de
        # Δθ_joint1 que melhor alinha o arco com esse vetor.
        self.slide_dir_var   = tk.StringVar(value='+Y')

        self._param_row(params_card, label='Velocidade de Deslizamento',
                         unit='mm/s', var=self.speed_var,
                         vmin=SPEED_MIN, vmax=SPEED_MAX, step=1.0,
                         hint='Referências do artigo: 5, 10, 15 mm/s')
        self._param_row(params_card, label='Força Normal Alvo',
                         unit='N', var=self.force_var,
                         vmin=FORCE_MIN, vmax=FORCE_MAX, step=0.1,
                         hint='Referência do artigo: 1.0 N')
        self._param_row(params_card, label='Distância de Deslizamento',
                         unit='mm', var=self.dist_var,
                         vmin=DIST_MIN, vmax=DIST_MAX, step=5.0,
                         hint='Referência do artigo: 90 mm')
        # Distância CM dedo→alvo — descida desde a Home customizada.
        self._param_row(params_card, label='Distância dedo→alvo',
                         unit='cm', var=self.target_dist_var,
                         vmin=TGT_DIST_CM_MIN, vmax=TGT_DIST_CM_MAX,
                         step=0.5,
                         hint='Distância vertical da Home até a superfície '
                              '(o contato pára antes se a força for atingida)')
        # approach speed: slider removido (era novo _param_row que
        # parecia disparar o segfault do Tk). Continua sendo enviado no
        # payload com o valor default; para alterar use:
        #   ros2 param set /tactile_explorer approach_v_max_mms 30.0

        # Seletor de direção do deslizamento (XY mundo). joint1 só pode
        # produzir um arco; aqui o usuário escolhe a ORIENTAÇÃO desejada
        # e o explorer pega o sinal de Δθ que melhor se alinha a ela.
        self._build_slide_dir_selector(params_card)

        # ── Calibração PID (Kp / Ki / Kd) ─────────────────────────────
        pid_card = self._card(col_left, 'Calibração PID — Controle de Força (HOLD)')
        self._param_row(pid_card, label='Kp',
                         unit='(m/s)/N', var=self.pid_kp_var,
                         vmin=KP_MIN, vmax=KP_MAX, step=KP_STEP,
                         hint='Proporcional — quanto reage à diferença '
                              'instantânea entre força medida e alvo.')
        self._param_row(pid_card, label='Ki',
                         unit='(m/s)/(N·s)', var=self.pid_ki_var,
                         vmin=KI_MIN, vmax=KI_MAX, step=KI_STEP,
                         hint='Integral — elimina offset estacionário '
                              '(ative aos poucos: causa overshoot).')
        self._param_row(pid_card, label='Kd',
                         unit='m/N', var=self.pid_kd_var,
                         vmin=KD_MIN, vmax=KD_MAX, step=KD_STEP,
                         hint='Derivativo — amortece oscilação. '
                              'Aplicado sobre o erro filtrado pelo loop.')

        # ── Coluna direita: botão de início (fixado no fundo) + feedback FT ──
        # O botão é empacotado primeiro com side='bottom' para ficar visível
        # independente do tamanho da janela; o fb_card preenche o restante.
        btn_wrap = tk.Frame(col_right, bg=BG)
        btn_wrap.pack(fill='x', side='bottom', pady=(14, 0))
        self.stop_palp_btn = tk.Button(
            btn_wrap, text='⏹  Parar Palpação',
            command=self._on_stop_palpation, bg=WARN, fg='white',
            activebackground=_shade(WARN, -0.1), activeforeground='white',
            font=FONT_HEAD, relief='flat', bd=0, padx=18, pady=10,
            cursor='hand2')
        self.stop_palp_btn.pack(fill='x', pady=(0, 6))
        self.start_btn = tk.Button(
            btn_wrap, text='▶  Iniciar Palpação',
            command=self._on_start, bg=PRIMARY, fg='white',
            activebackground=PRIMARY_HV, activeforeground='white',
            font=FONT_HEAD, relief='flat', bd=0, padx=18, pady=12,
            cursor='hand2')
        self.start_btn.pack(fill='x')

        fb_card = self._card(col_right,
                              'Sensor de Força — Mirror do Robô (TCP)')

        fnrow = tk.Frame(fb_card, bg=PANEL)
        fnrow.pack(fill='x', pady=(6, 4))
        tk.Label(fnrow, text='Força Normal na Última Junta', font=FONT_LBL,
                 bg=PANEL, fg=TEXT_MUTED).pack(anchor='w')
        self.force_value_lbl = tk.Label(
            fnrow, text='—   N', font=FONT_BIG, bg=PANEL, fg=TEXT_DIM)
        self.force_value_lbl.pack(anchor='w', pady=(2, 2))
        self.force_status_lbl = tk.Label(
            fnrow, text='aguardando /ft_sensor/wrench',
            font=FONT_LBL, bg=PANEL, fg=TEXT_DIM)
        self.force_status_lbl.pack(anchor='w')

        tk.Frame(fb_card, bg=BORDER, height=1).pack(fill='x', pady=8)
        errrow = tk.Frame(fb_card, bg=PANEL)
        errrow.pack(fill='x', pady=(2, 6))
        tk.Label(errrow, text='Erro vs. Alvo', font=FONT_LBL,
                 bg=PANEL, fg=TEXT_MUTED).pack(side='left')
        self.err_value_lbl = tk.Label(
            errrow, text='—  N', font=FONT_HEAD, bg=PANEL, fg=TEXT)
        self.err_value_lbl.pack(side='right')

        compbox = tk.Frame(fb_card, bg=PANEL)
        compbox.pack(fill='x', pady=(2, 6))
        # Palpação HORIZONTAL: Fz = força normal (vertical),
        # Fy = direção do sliding, Fx = tangencial perpendicular.
        self.fx_lbl = self._kv(compbox, 'Fx (tang.)', '0.00 N')
        self.fy_lbl = self._kv(compbox, 'Fy (slide)',  '0.00 N')
        self.fz_lbl = self._kv(compbox, 'Fz (normal)', '0.00 N')

        tk.Frame(fb_card, bg=BORDER, height=1).pack(fill='x', pady=8)
        prow = tk.Frame(fb_card, bg=PANEL)
        prow.pack(fill='x')
        tk.Label(prow, text='Fase do Experimento', font=FONT_LBL,
                 bg=PANEL, fg=TEXT_MUTED).pack(side='left')
        self.phase_lbl = tk.Label(
            prow, text='IDLE', font=FONT_HEAD, bg=PANEL, fg=TEXT)
        self.phase_lbl.pack(side='right')

        # Cronômetro só com label — sem ttk.Progressbar. Em alguns
        # ambientes (Ubuntu 22.04 + Tcl 8.6 sem fontes JetBrains/Segoe
        # instaladas) o Progressbar corrompia estado interno do Tk
        # provocando segfault na criação de widgets posteriores.
        timerow = tk.Frame(fb_card, bg=PANEL)
        timerow.pack(fill='x', pady=(8, 2))
        tk.Label(timerow, text='Progresso', font=FONT_LBL,
                 bg=PANEL, fg=TEXT_MUTED).pack(side='left')
        self.timer_lbl = tk.Label(
            timerow, text='—', font=FONT_HEAD, bg=PANEL, fg=TEXT_MUTED)
        self.timer_lbl.pack(side='right')

    # ── Aba "Controle Manual" ────────────────────────────────────────
    def _build_manual_tab(self, root: tk.Frame):
        """Constrói a aba de jog manual: tempo de movimento + 6 sliders do
        braço + 6 sliders da mão.

        As alterações nos sliders publicam direto em
            /cr10_group_controller/joint_trajectory   (braço, rad)
            /hand_position_controller/joint_trajectory (mão, rad + mimic)
        sem passar pela máquina de estados do tactile_explorer."""
        body = tk.Frame(root, bg=BG)
        body.pack(fill='both', expand=True)

        # ── Top: controle de velocidade (SpeedFactor %) ─────────────────
        speed_card = tk.Frame(body, bg=PANEL,
                               highlightthickness=1,
                               highlightbackground=BORDER,
                               highlightcolor=BORDER)
        speed_card.pack(fill='x', pady=(0, 10))
        tk.Label(speed_card, text='Velocidade de Movimento',
                 bg=PANEL, fg=TEXT, font=FONT_HEAD,
                 anchor='w').pack(fill='x', padx=14, pady=(10, 6))
        tk.Frame(speed_card, bg=BORDER, height=1).pack(fill='x')
        speed_inner = tk.Frame(speed_card, bg=PANEL)
        speed_inner.pack(fill='x', padx=14, pady=10)

        self.speed_factor_var = tk.DoubleVar(value=SPEED_FACTOR_DEFAULT)
        self._param_row(speed_inner, label='Velocidade', unit='%',
                        var=self.speed_factor_var,
                        vmin=SPEED_FACTOR_MIN, vmax=SPEED_FACTOR_MAX, step=1,
                        hint='Afeta jog manual (MovJ/PTP) e duração no Gazebo. '
                             'NÃO afeta streaming durante palpação (CONTACT/'
                             'HOLD/SLIDING/RETRACT usam ServoJ com velocidade '
                             'própria definida nos parâmetros acima).')
        self.speed_factor_var.trace_add(
            'write', lambda *_: self._apply_speed_factor_if_active())

        cols = tk.Frame(body, bg=BG)
        cols.pack(fill='both', expand=True)
        col_arm  = tk.Frame(cols, bg=BG)
        col_hand = tk.Frame(cols, bg=BG)
        col_arm.pack(side='left', fill='both', expand=True, padx=(0, 9))
        col_hand.pack(side='left', fill='both', expand=True, padx=(9, 0))

        # ── BRAÇO CR10 ────────────────────────────────────────────────
        card_arm = self._card(col_arm, 'Braço CR10 — juntas (graus)')
        self.arm_sliders: dict[str, tk.DoubleVar] = {}
        for j in ARM_JOINTS:
            lo, hi = ARM_LIMITS_DEG[j]
            var = tk.DoubleVar(value=self._arm_home_deg[j])
            self.arm_sliders[j] = var
            self._joint_row(card_arm, label=j, unit='°',
                              var=var, vmin=lo, vmax=hi, step=1.0,
                              on_change=self._publish_arm_from_sliders)

        btns_arm = tk.Frame(col_arm, bg=BG)
        btns_arm.pack(fill='x', pady=(10, 0))
        tk.Button(btns_arm, text='⌂  Home',
                   command=self._apply_arm_home,
                   bg=PRIMARY, fg='white',
                   activebackground=PRIMARY_HV, activeforeground='white',
                   font=FONT_LBL, relief='flat', bd=0, padx=14, pady=8,
                   cursor='hand2'
                   ).pack(side='left', fill='x', expand=True, padx=(0, 4))
        # 💾 = grava os ângulos atuais como nova Home (persiste em JSON).
        tk.Button(btns_arm, text='💾  Salvar Home',
                   command=self._save_home_pose,
                   bg=OK, fg='white',
                   activebackground=_shade(OK, -0.08),
                   activeforeground='white',
                   font=FONT_LBL, relief='flat', bd=0, padx=14, pady=8,
                   cursor='hand2'
                   ).pack(side='left', fill='x', expand=True, padx=(4, 0))

        btns_arm2 = tk.Frame(col_arm, bg=BG)
        btns_arm2.pack(fill='x', pady=(4, 0))
        tk.Button(btns_arm2, text='📍  Capturar do Robô',
                   command=self._capture_arm_from_robot,
                   bg=_shade(PRIMARY, 0.25), fg=PRIMARY,
                   activebackground=_shade(PRIMARY, 0.15),
                   activeforeground=PRIMARY,
                   font=FONT_LBL, relief='flat', bd=0, padx=14, pady=6,
                   cursor='hand2'
                   ).pack(side='left', fill='x', expand=True)

        # ── MÃO COVVI ─────────────────────────────────────────────────
        card_hand = self._card(col_hand, 'Mão COVVI — juntas primárias (graus)')
        self.hand_sliders: dict[str, tk.DoubleVar] = {}
        for j in HAND_JOINTS:
            lo, hi = HAND_LIMITS_DEG[j]
            var = tk.DoubleVar(value=0)
            self.hand_sliders[j] = var
            self._joint_row(card_hand, label=j, unit='°',
                              var=var, vmin=lo, vmax=hi, step=1.0,
                              on_change=self._publish_hand_from_sliders)

        btns_hand = tk.Frame(col_hand, bg=BG)
        btns_hand.pack(fill='x', pady=(10, 0))
        tk.Button(btns_hand, text='✋  Abrir',
                   command=lambda: self._apply_hand_preset(
                       HAND_OPEN_DEG, eci_grip_id=11),   # 11 = GLOVE
                   bg=BTN_NEUTRAL, fg=TEXT,
                   activebackground=_shade(BTN_NEUTRAL, -0.08),
                   activeforeground=TEXT,
                   font=FONT_LBL, relief='flat', bd=0, padx=12, pady=8,
                   cursor='hand2'
                   ).pack(side='left', fill='x', expand=True, padx=(0, 3))
        tk.Button(btns_hand, text='👉  Apontar',
                   command=lambda: self._apply_hand_preset(
                       HAND_POINT_DEG, eci_grip_id=7),    # 7 = FINGER (Index ext.)
                   bg=OK, fg='white',
                   activebackground=_shade(OK, -0.08),
                   activeforeground='white',
                   font=FONT_LBL, relief='flat', bd=0, padx=12, pady=8,
                   cursor='hand2'
                   ).pack(side='left', fill='x', expand=True, padx=3)
        tk.Button(btns_hand, text='✊  Fechar',
                   command=lambda: self._apply_hand_preset(
                       HAND_CLOSE_DEG, eci_grip_id=2),    # 2 = POWER
                   bg=PRIMARY, fg='white',
                   activebackground=PRIMARY_HV, activeforeground='white',
                   font=FONT_LBL, relief='flat', bd=0, padx=12, pady=8,
                   cursor='hand2'
                   ).pack(side='left', fill='x', expand=True, padx=(3, 0))

    def _joint_row(self, parent, *, label, unit, var,
                    vmin, vmax, step, on_change):
        """Linha compacta com label + spinbox + slider para uma junta.

        Conecta `var.trace` para que arrastar o slider OU digitar no
        spinbox dispare imediatamente o publish."""
        row = tk.Frame(parent, bg=PANEL); row.pack(fill='x', pady=(3, 1))
        top = tk.Frame(row, bg=PANEL); top.pack(fill='x')
        tk.Label(top, text=label, font=FONT_MONO_S, bg=PANEL, fg=TEXT,
                 width=10, anchor='w').pack(side='left')
        tk.Spinbox(top, from_=vmin, to=vmax, increment=step,
                    textvariable=var, width=7, font=FONT_MONO,
                    justify='right', relief='flat', bd=0,
                    highlightthickness=1, highlightbackground=BORDER,
                    highlightcolor=PRIMARY
                    ).pack(side='right', padx=(6, 0), ipady=2)
        tk.Label(top, text=unit, font=FONT_SMALL, bg=PANEL, fg=TEXT_MUTED
                 ).pack(side='right')
        ttk.Scale(row, from_=vmin, to=vmax, variable=var,
                   orient='horizontal',
                   style='Tactile.Horizontal.TScale'
                   ).pack(fill='x', pady=(1, 0))
        # `var.trace_add` dispara em qualquer mudança do valor.
        var.trace_add('write',
                       lambda *_a: (not self._suppressing) and on_change())

    # ── Clamp helpers ─────────────────────────────────────────────────
    def _clamp_var(self, var: tk.DoubleVar, vmin: float, vmax: float,
                    default: float | None = None) -> float | None:
        """Lê `var`, força-o ao intervalo [vmin, vmax] (re-escreve no var
        se necessário) e devolve o valor saneado. Retorna `default` (ou
        None) se a leitura falhar."""
        try:
            v = float(var.get())
        except (ValueError, tk.TclError):
            return default
        v_clamped = max(vmin, min(vmax, v))
        if v_clamped != v:
            var.set(v_clamped)
        return v_clamped

    def _move_duration_seconds(self) -> float:
        """Duração da trajetória Gazebo derivada do slider de velocidade.

        Inversamente proporcional à velocidade: 10 % → 3.0 s, 100 % → 0.3 s."""
        try:
            speed_pct = float(self.speed_factor_var.get())
            speed_pct = max(SPEED_FACTOR_MIN, min(SPEED_FACTOR_MAX, speed_pct))
        except (ValueError, tk.TclError):
            speed_pct = SPEED_FACTOR_DEFAULT
        return max(0.3, _VEL_BASE_S * (10.0 / speed_pct))

    def _apply_speed_factor_if_active(self) -> None:
        """Envia SpeedFactor(%) ao braço real sempre que o slider mudar."""
        if not self._robot_connected or self._real_driver is None:
            return
        try:
            v = int(max(SPEED_FACTOR_MIN,
                        min(SPEED_FACTOR_MAX, self.speed_factor_var.get())))
        except (ValueError, tk.TclError):
            return
        try:
            # _send_dash já serializa via _dash_lock interno — _real_lock não necessário.
            self._real_driver._send_dash(f'SpeedFactor({v})')
            self.get_logger().warning(
                f'[SPEED] SpeedFactor({v})%% enviado ao CR10 real')
        except CR10RealDriverError as exc:
            self.get_logger().warning(f'SpeedFactor falhou: {exc}')

    @staticmethod
    def _duration_msg(seconds: float) -> Duration:
        sec = int(seconds)
        nsec = int((seconds - sec) * 1e9)
        return Duration(sec=sec, nanosec=nsec)

    # ── Publicação direta nos controllers ─────────────────────────────
    def _publish_arm_from_sliders(self):
        if self._suppressing:
            return
        # Bloqueia publish de slider durante palpação ativa: o explorer está
        # fazendo streaming no mesmo tópico JTC a 33 Hz. Um publish do slider
        # substituiria o setpoint do explorer por uma posição arbitrária,
        # causando solavanco e desestabilizando o controle de força no HOLD.
        with self._lock:
            _phase = self._latest_phase
        if _phase not in ('IDLE', 'DONE', 'ABORTED'):
            return
        self._suppressing = True
        try:
            positions_deg: list[float] = []
            for j in ARM_JOINTS:
                lo, hi = ARM_LIMITS_DEG[j]
                v = self._clamp_var(self.arm_sliders[j], lo, hi)
                if v is None:
                    return
                positions_deg.append(v)
            duration_s = self._move_duration_seconds()
        finally:
            self._suppressing = False
        positions_rad = [_math.radians(d) for d in positions_deg]
        msg = JointTrajectory()
        # stamp=zero → controller starts the trajectory immediately,
        # regardless of whether the node uses sim-time or wall-time.
        msg.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions_rad]
        pt.time_from_start = self._duration_msg(duration_s)
        msg.points.append(pt)
        self._arm_pub.publish(msg)
        # MIRROR é tratado pela subscrição em /cr10_group_controller/joint_trajectory
        # — captura este publish e também o do tactile_explorer numa única rota.

    # ── Mirror MovJ (MIRROR mode — braço real segue os sliders) ──────────
    def _mirror_movj_debounced(self, positions_rad: list[float]) -> None:
        """Agenda MovJ ao braço real com debounce de 80 ms.

        Enquanto chegam publicações em rajada (slider arrastando, streaming
        do explorer), o timer é cancelado e reagendado — só dispara 80 ms
        após a última publicação, evitando flood de MovJ.
        """
        q_new = np.asarray(positions_rad, dtype=np.float64)
        with self._mirror_timer_lock:
            if self._mirror_timer is not None:
                self._mirror_timer.cancel()
            self._mirror_timer = threading.Timer(
                0.08, self._mirror_movj_send, args=[q_new.tolist()])
            self._mirror_timer.daemon = True
            self._mirror_timer.start()

    def _mirror_movj_send(self, positions_rad: list[float]) -> None:
        """Converte URDF→DOBOT, define SpeedFactor e envia MovJ ao braço real.

        SpeedFactor é sempre enviado antes do MovJ para garantir que a
        velocidade está correta — igual ao padrão da DobotAPI do fabricante.
        """
        try:
            q_dobot_rad = _urdf_to_dobot(
                np.array(positions_rad, dtype=np.float64))
            q_dobot_deg = [math.degrees(float(v)) for v in q_dobot_rad]
            try:
                speed_pct = int(max(SPEED_FACTOR_MIN,
                                    min(SPEED_FACTOR_MAX,
                                        self.speed_factor_var.get())))
            except (ValueError, tk.TclError):
                speed_pct = SPEED_FACTOR_DEFAULT
            with self._real_lock:
                drv = self._real_driver
                if (drv is None or not self._robot_connected
                        or self._robot_mode != 'MIRROR'):
                    return
                # Race guard: o timer de debounce pode disparar após a fase
                # mudar para HOME/CONTACT/etc. — MovJ durante ServoJ causa solavanco.
                with self._lock:
                    if self._latest_phase not in ('IDLE', 'DONE', 'ABORTED'):
                        return
                drv._send_dash(f'SpeedFactor({speed_pct})')
                drv.mov_j_joint_deg(q_dobot_deg)
            self._mirror_last_target = np.asarray(
                positions_rad, dtype=np.float64)
        except CR10RealDriverError as exc:
            self.get_logger().warning(f'Mirror MovJ falhou: {exc}')

    # ── Subscrição no tópico de trajetória comandada ─────────────────
    def _cb_arm_trajectory(self, msg: JointTrajectory) -> None:
        """Captura trajetórias publicadas em /cr10_group_controller/joint_trajectory.

        Em MIRROR + IDLE (jog manual): dispara MovJ debounced (SpeedFactor já
        incluído em _mirror_movj_send). Durante palpação ativa o poll loop
        via /joint_states + ServoJ cobre o mirroring.
        """
        if self._robot_mode != 'MIRROR':
            return
        # Drag teach ativo → motores liberados, não enviar comandos de posição.
        if self._drag_enabled:
            return
        # Execução de movimento via _execute_movement_worker → não interferir.
        if self._exec_movement_id is not None:
            return
        with self._lock:
            phase = self._latest_phase
        if phase not in ('IDLE', 'DONE', 'ABORTED'):
            return  # palpação ativa → ServoJ poll loop assume
        if not msg.points:
            return
        positions_rad = list(msg.points[-1].positions)
        if len(positions_rad) < 6:
            return
        self._mirror_movj_debounced(positions_rad[:6])

    def _cb_joint_states(self, msg: JointState) -> None:
        """Armazena posições URDF das juntas do braço — alimenta o mirror poll."""
        pos = dict(zip(msg.name, msg.position))
        try:
            self._latest_joint_rad = [float(pos[j]) for j in ARM_JOINTS]
        except KeyError:
            pass  # msg parcial (mão ou outra cadeia) — ignorar

    def _mirror_poll_loop(self) -> None:
        """Envia ServoJ ao braço real a 33 Hz APENAS durante palpação ativa.

        Durante jog manual (fase IDLE/DONE/ABORTED) o mirroring é feito por
        _cb_arm_trajectory → _mirror_movj_debounced com SpeedFactor + MovJ,
        idêntico ao padrão da DobotAPI do fabricante.

        ServoJ é usado apenas durante palpação contínua (CONTACT/HOLD/
        SLIDING/RETRACT) onde a latência baixa supera a vantagem do SpeedFactor.
        """
        _servoj_ready = False
        _diag_count = 0
        _PERIOD = 0.030   # 33 Hz
        _t_next = time.monotonic() + _PERIOD
        while not self._stop_event.is_set():
            # Drift-compensated sleep: corrige jitter acumulado do SO.
            # wait(0.030) pode demorar 31–40 ms no Linux com carga, causando
            # descontinuidades no ServoJ que levam a sons e solavancos no real.
            now = time.monotonic()
            sleep_s = max(0.0, _t_next - now)
            self._stop_event.wait(sleep_s)
            _t_next += _PERIOD
            # Evita recuperar múltiplos ticks atrasados de uma vez.
            if _t_next < time.monotonic():
                _t_next = time.monotonic() + _PERIOD
            if (self._robot_mode != 'MIRROR' or not self._robot_connected
                    or self._real_driver is None or _urdf_to_dobot is None):
                _servoj_ready = False
                continue
            # Drag teach ativo → lê posição real e espelha para o Gazebo.
            if self._drag_enabled:
                _servoj_ready = False
                drv = self._real_driver
                if drv is None or not self._robot_connected:
                    continue
                try:
                    q_urdf = drv.read_joints_urdf()
                    msg = JointTrajectory()
                    msg.joint_names = ARM_JOINTS
                    pt = JointTrajectoryPoint()
                    pt.positions = [float(v) for v in q_urdf]
                    pt.velocities = [0.0] * 6
                    # time_from_start = período do loop → Gazebo acompanha sem lag
                    pt.time_from_start = Duration(sec=0, nanosec=30_000_000)
                    msg.points.append(pt)
                    self._arm_pub.publish(msg)
                except Exception:
                    pass
                continue
            # Execução de movimento em andamento → worker controla o braço real.
            if self._exec_movement_id is not None:
                _servoj_ready = False
                continue
            # Jog manual: MovJ via _cb_arm_trajectory cuida do espelhamento.
            with self._lock:
                phase = self._latest_phase
            if phase in ('IDLE', 'DONE', 'ABORTED'):
                _servoj_ready = False
                continue
            positions = self._latest_joint_rad
            if positions is None:
                continue
            q_new = np.asarray(positions, dtype=np.float64)
            last = self._mirror_last_target
            if last is not None and np.max(np.abs(q_new - last)) < 0.0001:
                continue   # braço estacionário — sem ServoJ redundante
            # Captura referência local: evita corrida com connect/disconnect sem
            # segurar _real_lock no caminho quente (servo_j usa _dash_lock interno).
            drv = self._real_driver
            if drv is None or not self._robot_connected:
                continue
            try:
                try:
                    drv.servo_j_urdf(positions)
                    _servoj_ready = True
                except CR10RealDriverError:
                    # -50001: subsistema ServoJ não pronto (pós-PTP).
                    # prepare_servoj envia ClearError+Continue e aguarda 50 ms.
                    drv.prepare_servoj()
                    drv.servo_j_urdf(positions)
                    _servoj_ready = True
            except CR10RealDriverError as exc:
                self.get_logger().warning(f'ServoJ falhou: {exc}')
                _servoj_ready = False
                continue
            self._mirror_last_target = q_new
            _diag_count += 1
            if _diag_count >= 330:   # ~10 s (era 90 = 2.7 s — causava jitter periódico)
                _diag_count = 0
                # Diagnóstico fora do caminho crítico: apenas loga, não bloqueia ServoJ.
                try:
                    ang = drv.get_angle_deg()
                    self.get_logger().info(f'[MIRROR-POS] GetAngle real: {ang}')
                except Exception:
                    pass

    # ──────────────────────────────────────────────────────────────────
    # Bridge real CR10 → /ft_sensor/wrench
    # ──────────────────────────────────────────────────────────────────
    def _force_bridge_active(self) -> bool:
        return (self._force_bridge_thread is not None
                and self._force_bridge_thread.is_alive())

    def _start_force_bridge(self) -> None:
        """Sobe a thread que publica /ft_sensor/wrench a partir do TCP
        force estimado pelo controlador do CR10 real."""
        if self._force_bridge_active():
            return
        if self._real_driver is None or not self._robot_connected:
            return
        self._force_bridge_stop.clear()
        self._force_bridge_thread = threading.Thread(
            target=self._force_bridge_loop, daemon=True)
        self._force_bridge_thread.start()
        self._set_status(
            'Sensor de força: mirror do CR10 ativo (/ft_sensor/wrench).',
            OK)

    def _force_bridge_loop(self) -> None:
        period = FORCE_BRIDGE_PERIOD_S
        while not self._force_bridge_stop.is_set():
            t0 = time.time()
            drv = self._real_driver
            if drv is None or not self._robot_connected:
                return
            try:
                # Sem _real_lock: read_tcp_force usa porta 30004 (_feed_lock
                # interno) — independente do ServoJ em 29999 (_dash_lock).
                w = drv.read_tcp_force()
            except Exception as exc:
                self.get_logger().error(
                    f'Force bridge falhou: {exc}')
                # Backoff curto antes de tentar de novo.
                self._force_bridge_stop.wait(0.5)
                continue
            msg = WrenchStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'tcp_link'
            msg.wrench.force.x  = float(w[0])
            msg.wrench.force.y  = float(w[1])
            msg.wrench.force.z  = float(w[2])
            msg.wrench.torque.x = float(w[3])
            msg.wrench.torque.y = float(w[4])
            msg.wrench.torque.z = float(w[5])
            try:
                self._wrench_pub.publish(msg)
            except Exception:
                pass
            dt = time.time() - t0
            if dt < period:
                self._force_bridge_stop.wait(period - dt)

    def _stop_force_bridge(self) -> None:
        self._force_bridge_stop.set()
        thr = self._force_bridge_thread
        if thr is not None:
            thr.join(timeout=1.0)
        self._force_bridge_thread = None

    def _publish_hand_from_sliders(self):
        if self._suppressing:
            return
        self._suppressing = True
        try:
            primary_deg: dict[str, float] = {}
            primary_rad: dict[str, float] = {}
            for j in HAND_JOINTS:
                lo, hi = HAND_LIMITS_DEG[j]
                v = self._clamp_var(self.hand_sliders[j], lo, hi)
                if v is None:
                    return
                primary_deg[j] = float(v)
                primary_rad[j] = _math.radians(v)
            duration_s = self._move_duration_seconds()
        finally:
            self._suppressing = False
        names = list(HAND_JOINTS)
        positions = [primary_rad[j] for j in HAND_JOINTS]
        # Expande as 26 juntas mimic com as razões do URDF.
        for mimic_name, driver, mult in MIMIC_LIST:
            names.append(mimic_name)
            positions.append(primary_rad[driver] * mult)
        msg = JointTrajectory()
        # stamp=zero → controller starts immediately (sim-time-safe).
        msg.joint_names = names
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions]
        pt.time_from_start = self._duration_msg(duration_s)
        msg.points.append(pt)
        self._hand_pub.publish(msg)
        # Envia para a mão real via ECI (SetDigitPosn) se ativo
        if self._eci_enabled:
            self._schedule_eci_posn(primary_deg)

    def _schedule_eci_posn(self, deg_dict: dict) -> None:
        """Debounce de 60 ms para SetDigitPosn — evita flood de serviço."""
        if not self._eci_enabled or self._cli_eci_posn is None:
            return
        if self._eci_posn_after is not None:
            try:
                self.root.after_cancel(self._eci_posn_after)
            except Exception:
                pass
        self._eci_posn_after = self.root.after(
            60, lambda v=dict(deg_dict): self._send_eci_posn_now(v))

    def _send_eci_posn_now(self, deg_dict: dict) -> None:
        """Envia SetDigitPosn convertendo graus → escala ECI 0-200."""
        self._eci_posn_after = None
        if not self._eci_enabled or self._cli_eci_posn is None:
            return
        if not self._cli_eci_posn.service_is_ready():
            return

        def _to_eci(joint: str, deg: float) -> int:
            max_deg = 60.0 if joint == 'Rotate' else 90.0
            return max(0, min(200, int(deg / max_deg * 200)))

        req = self._eci_srv.SetDigitPosn.Request()
        req.speed = self._eci_msg.Speed()
        try:
            sf = float(self.speed_factor_var.get())
        except (ValueError, tk.TclError):
            sf = SPEED_FACTOR_DEFAULT
        req.speed.value = max(1, min(100, int(sf)))
        req.thumb  = _to_eci('Thumb',  deg_dict.get('Thumb',  0.0))
        req.index  = _to_eci('Index',  deg_dict.get('Index',  0.0))
        req.middle = _to_eci('Middle', deg_dict.get('Middle', 0.0))
        req.ring   = _to_eci('Ring',   deg_dict.get('Ring',   0.0))
        req.little = _to_eci('Little', deg_dict.get('Little', 0.0))
        req.rotate = _to_eci('Rotate', deg_dict.get('Rotate', 0.0))
        self._cli_eci_posn.call_async(req)

    def _apply_arm_home(self):
        """Move o braço para a Home customizada do usuário."""
        self._suppressing = True
        try:
            for j, deg in self._arm_home_deg.items():
                self.arm_sliders[j].set(deg)
        finally:
            self._suppressing = False
        self._publish_arm_from_sliders()

    # ── Home customizada — load / save em ~/.config/touch_pack/ ──────
    def _load_home_pose(self) -> None:
        """Carrega home salvo (sobrescreve `self._arm_home_deg`)."""
        try:
            if os.path.exists(HOME_POSE_FILE):
                with open(HOME_POSE_FILE) as fh:
                    data = json.load(fh)
                if isinstance(data, dict):
                    for j in ARM_JOINTS:
                        if j in data:
                            try:
                                lo, hi = ARM_LIMITS_DEG[j]
                                v = float(data[j])
                                self._arm_home_deg[j] = max(lo, min(hi, v))
                            except (TypeError, ValueError):
                                pass
                self.get_logger().info(
                    f'Home carregada de {HOME_POSE_FILE}')
        except Exception as exc:    # pragma: no cover
            self.get_logger().warn(f'Falha ao ler home pose: {exc}')

    def _save_home_pose(self) -> None:
        """Captura os ângulos dos sliders do braço como nova Home e
        persiste em ~/.config/touch_pack/home_pose.json.
        O botão `⌂ Home` passa a usar esses valores."""
        try:
            new_home = {
                j: float(self.arm_sliders[j].get()) for j in ARM_JOINTS
            }
        except (ValueError, tk.TclError):
            self._set_status('Sliders inválidos.', DANGER)
            return
        try:
            os.makedirs(os.path.dirname(HOME_POSE_FILE), exist_ok=True)
            with open(HOME_POSE_FILE, 'w') as fh:
                json.dump(new_home, fh, indent=2, sort_keys=True)
        except Exception as exc:    # pragma: no cover
            self._set_status(f'Falha ao salvar home: {exc}', DANGER)
            return
        self._arm_home_deg = new_home
        summary = ' / '.join(f'{j[-1]}={new_home[j]:+.0f}°'
                              for j in ARM_JOINTS)
        self._set_status(f'Home salva ({summary}).', OK)

    def _capture_arm_from_robot(self) -> None:
        """Lê a posição atual do robô real, atualiza os sliders e salva
        como Home. O Gazebo iniciará nessa configuração na próxima vez
        que o launch file for executado (lê o mesmo home_pose.json)."""
        if not self._robot_connected or self._real_driver is None:
            self._set_status(
                'Conecte o robô CR10 antes de capturar a posição.', WARN)
            return
        if not _REAL_DRIVER_OK or _urdf_to_dobot is None:
            self._set_status('Driver real não disponível.', DANGER)
            return
        try:
            from .kinematics import dobot_to_urdf as _dobot_to_urdf
        except Exception:
            self._set_status('kinematics não disponível.', DANGER)
            return
        try:
            # read_joints_rad usa porta 30004 (_feed_lock interno) — _real_lock não necessário.
            q_dobot_rad = self._real_driver.read_joints_rad()
        except CR10RealDriverError as exc:
            self._set_status(f'Falha ao ler juntas: {exc}', DANGER)
            return
        q_urdf_rad = _dobot_to_urdf(q_dobot_rad)
        new_home = {
            j: float(_math.degrees(q_urdf_rad[i]))
            for i, j in enumerate(ARM_JOINTS)
        }
        # Atualiza sliders (suprime o callback de publish).
        self._suppressing = True
        try:
            for j in ARM_JOINTS:
                lo, hi = ARM_LIMITS_DEG[j]
                clamped = max(lo, min(hi, new_home[j]))
                self.arm_sliders[j].set(clamped)
        finally:
            self._suppressing = False
        # Persiste em home_pose.json.
        try:
            os.makedirs(os.path.dirname(HOME_POSE_FILE), exist_ok=True)
            with open(HOME_POSE_FILE, 'w') as fh:
                json.dump(new_home, fh, indent=2, sort_keys=True)
        except Exception as exc:
            self._set_status(f'Falha ao salvar home capturada: {exc}', DANGER)
            return
        self._arm_home_deg = new_home
        summary = ' / '.join(f'{j[-1]}={new_home[j]:+.0f}°' for j in ARM_JOINTS)
        self._set_status(
            f'Home capturada do robô real e salva ({summary}). '
            'Reinicie o Gazebo para aplicar.', OK)

    # ── Persistência de IPs e modo (~/.config/touch_pack/robot.json) ──
    def _load_robot_config(self) -> None:
        """Carrega `_robot_cfg` (mescla defaults com JSON salvo). Silencioso
        se o arquivo não existir ou estiver corrompido — só preenche faltantes."""
        try:
            if not os.path.exists(ROBOT_CONFIG_FILE):
                return
            with open(ROBOT_CONFIG_FILE) as fh:
                data = json.load(fh)
            if not isinstance(data, dict):
                return
            for k, default in ROBOT_CONFIG_DEFAULTS.items():
                v = data.get(k)
                if isinstance(v, str) and v.strip():
                    self._robot_cfg[k] = v.strip()
            self.get_logger().info(
                f'Config robô carregada de {ROBOT_CONFIG_FILE}: '
                f'hand={self._robot_cfg["hand_ip"]} '
                f'robot={self._robot_cfg["robot_ip"]} '
                f'mode={self._robot_cfg["robot_mode"]}')
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f'Falha ao ler robot.json: {exc}')

    def _save_robot_config(self) -> None:
        """Persiste `_robot_cfg` em `ROBOT_CONFIG_FILE`. Atualiza os campos
        a partir dos StringVars antes de gravar."""
        try:
            ip_hand = (self._hand_ip_var.get() or '').strip()
            ip_robot = (self._robot_ip_var.get() or '').strip()
        except tk.TclError:
            return
        if ip_hand:
            self._robot_cfg['hand_ip'] = ip_hand
        if ip_robot:
            self._robot_cfg['robot_ip'] = ip_robot
        self._robot_cfg['robot_mode'] = self._robot_mode
        try:
            os.makedirs(os.path.dirname(ROBOT_CONFIG_FILE), exist_ok=True)
            with open(ROBOT_CONFIG_FILE, 'w') as fh:
                json.dump(self._robot_cfg, fh, indent=2, sort_keys=True)
        except OSError as exc:    # pragma: no cover
            self.get_logger().warn(f'Falha ao salvar robot.json: {exc}')

    def _send_eci_grip(self, grip_id: int, label: str = '') -> None:
        """Chama SetCurrentGrip via ECI de forma assíncrona.

        No-op se ECI não estiver ativo ou serviço indisponível.
        grip_id deve ser um valor de CurrentGripID (1–14 builtins).
        """
        if not self._eci_enabled or self._cli_eci_grip is None:
            return
        if not self._cli_eci_grip.service_is_ready():
            self._set_status('ECI SetCurrentGrip indisponível (aguarde).',
                              WARN)
            return
        try:
            grip = self._eci_msg.CurrentGripID()
            grip.value = grip_id
            req = self._eci_srv.SetCurrentGrip.Request()
            req.grip_id = grip
            self._cli_eci_grip.call_async(req)
            if label:
                self._set_status(f'ECI > {label} (id={grip_id})', OK)
        except Exception as exc:
            self.get_logger().error(f'SetCurrentGrip falhou: {exc}')

    def _apply_hand_preset(self, preset_deg: dict[str, float],
                            *, eci_grip_id: int | None = None):
        """Aplica um preset de mão (Abrir/Apontar/Fechar).

        Se `eci_grip_id` for fornecido e ECI estiver ativo, também chama
        SetCurrentGrip para mover a mão real.
        """
        self._suppressing = True
        try:
            for j in HAND_JOINTS:
                self.hand_sliders[j].set(preset_deg.get(j, 0))
        finally:
            self._suppressing = False
        self._publish_hand_from_sliders()
        if eci_grip_id is not None:
            self._send_eci_grip(eci_grip_id)

    # ──────────────────────────────────────────────────────────────────
    # Aba "Célula de Carga" — leitura + calibração
    # ──────────────────────────────────────────────────────────────────
    def _build_loadcell_tab(self, root: tk.Frame):
        sub_nb = ttk.Notebook(root, style='Tactile.TNotebook')
        sub_nb.pack(fill='both', expand=True)

        tab_leitura = tk.Frame(sub_nb, bg=BG)
        tab_calib   = tk.Frame(sub_nb, bg=BG)
        sub_nb.add(tab_leitura, text='Leitura')
        sub_nb.add(tab_calib,   text='Calibração')

        self._build_lc_leitura_tab(self._scrollable(tab_leitura))
        self._build_lc_calibration_tab(self._scrollable(tab_calib))
        self._restore_lc_calib_ui()
        self.root.after(100, self._refresh_lc_panel)

    def _build_lc_leitura_tab(self, root: tk.Frame):
        # ── Painel de conexão do nó UDP ──────────────────────────────────
        conn_panel = tk.Frame(root, bg=PANEL,
                              highlightthickness=1, highlightbackground=BORDER)
        conn_panel.pack(fill='x', pady=(0, 8))
        tk.Label(conn_panel, text='Receptor UDP (force_receiver_node)',
                 bg=PANEL, fg=TEXT, font=FONT_HEAD, anchor='w'
                 ).pack(fill='x', padx=14, pady=(10, 0))
        tk.Frame(conn_panel, bg=BORDER, height=1).pack(fill='x', pady=(6, 0))
        btn_row = tk.Frame(conn_panel, bg=PANEL)
        btn_row.pack(fill='x', padx=14, pady=8)
        self._force_rx_btn = tk.Button(
            btn_row, text='⚡  Conectar',
            command=self._toggle_force_receiver,
            bg=PRIMARY, fg='white',
            activebackground=PRIMARY_HV, activeforeground='white',
            font=FONT_LBL, relief='flat', bd=0, padx=14, pady=6,
            cursor='hand2')
        self._force_rx_btn.pack(side='left')
        self._force_rx_status_lbl = tk.Label(
            btn_row,
            text='Nó não iniciado — clique Conectar para abrir a porta UDP 8080',
            font=FONT_LBL, bg=PANEL, fg=TEXT_DIM)
        self._force_rx_status_lbl.pack(side='left', padx=(12, 0))

        # ── Card de leitura ───────────────────────────────────────────────
        card = self._card(root, 'Força Aplicada — Célula de Carga')

        # Força total calibrada (inclui preload estático da montagem)
        row_f = tk.Frame(card, bg=PANEL)
        row_f.pack(fill='x', pady=(8, 2))
        tk.Label(row_f, text='Força Total (calibração)', font=FONT_LBL,
                 bg=PANEL, fg=TEXT_MUTED).pack(anchor='w')
        self.lc_force_lbl = tk.Label(
            row_f, text='—   N', font=FONT_BIG, bg=PANEL, fg=TEXT_DIM)
        self.lc_force_lbl.pack(anchor='w', pady=(2, 0))

        self.lc_calib_status_lbl = tk.Label(
            card,
            text='Aguardando calibração — use a aba Calibração',
            font=FONT_LBL, bg=PANEL, fg=WARN)
        self.lc_calib_status_lbl.pack(anchor='w', pady=(0, 6))

        tk.Frame(card, bg=BORDER, height=1).pack(fill='x', pady=4)

        # Força normal perpendicular à mesa (após zeragem)
        row_n = tk.Frame(card, bg=PANEL)
        row_n.pack(fill='x', pady=(6, 2))
        tk.Label(row_n, text='Força Normal ⊥ mesa  (+compressão / −tração, ref. tare)',
                 font=FONT_LBL, bg=PANEL, fg=TEXT_MUTED).pack(anchor='w')
        self.lc_normal_force_lbl = tk.Label(
            row_n, text='—   N', font=FONT_BIG, bg=PANEL, fg=TEXT_DIM)
        self.lc_normal_force_lbl.pack(anchor='w', pady=(2, 0))

        self.lc_tare_status_lbl = tk.Label(
            card, text='Zeragem não realizada — clique Zerar Sensor antes de palpar',
            font=FONT_LBL, bg=PANEL, fg=WARN)
        self.lc_tare_status_lbl.pack(anchor='w', pady=(0, 6))

        tare_btn_row = tk.Frame(card, bg=PANEL)
        tare_btn_row.pack(fill='x', pady=(0, 6))
        tk.Button(
            tare_btn_row, text='🎯  Zerar Sensor (Tare)',
            command=self._lc_do_tare,
            bg=PRIMARY, fg='white',
            activebackground=PRIMARY_HV, activeforeground='white',
            font=FONT_LBL, relief='flat', bd=0, padx=14, pady=7,
            cursor='hand2',
        ).pack(side='left')
        tk.Label(
            tare_btn_row,
            text='Pressione com o sensor descarregado\n(robô sem tocar a superfície)',
            font=('Segoe UI', 9), bg=PANEL, fg=TEXT_DIM, justify='left',
        ).pack(side='left', padx=(10, 0))

        tk.Frame(card, bg=BORDER, height=1).pack(fill='x', pady=6)

        row_v = tk.Frame(card, bg=PANEL)
        row_v.pack(fill='x', pady=(2, 2))
        tk.Label(row_v, text='Tensão do Sensor', font=FONT_LBL,
                 bg=PANEL, fg=TEXT_MUTED).pack(side='left')
        self.lc_voltage_lbl = tk.Label(
            row_v, text='—  V', font=FONT_MONO, bg=PANEL, fg=TEXT_DIM)
        self.lc_voltage_lbl.pack(side='right')

    def _build_lc_calibration_tab(self, root: tk.Frame):
        # ── Status da calibração vigente ─────────────────────────────
        status_card = self._card(root, 'Calibração Vigente')
        self.lc_curr_calib_lbl = tk.Label(
            status_card, text='Nenhuma calibração salva',
            font=FONT_LBL, bg=PANEL, fg=WARN)
        self.lc_curr_calib_lbl.pack(anchor='w', pady=(0, 4))

        # Pontos usados na calibração (populados por _restore_lc_calib_ui)
        self.lc_saved_point_lbls: list[tk.Label] = []
        for i in range(5):
            lbl = tk.Label(
                status_card,
                text='',
                font=FONT_MONO_S, bg=PANEL, fg=OK, anchor='w')
            lbl.pack(fill='x', pady=1)
            self.lc_saved_point_lbls.append(lbl)

        # ── Passo 1: referência zero (sensor sem nada) ───────────────────
        zero_card = self._card(root, 'Passo 1 — Capturar Zero (sensor sem força)')
        tk.Label(
            zero_card,
            text='Retire tudo do sensor e clique Capturar Zero.\n'
                 'Este ponto ancora a reta em F = 0 N e elimina o offset.',
            font=FONT_LBL, bg=PANEL, fg=TEXT_MUTED, justify='left',
        ).pack(anchor='w', pady=(0, 8))
        zero_btn_row = tk.Frame(zero_card, bg=PANEL)
        zero_btn_row.pack(fill='x')
        tk.Button(
            zero_btn_row, text='⦾  Capturar Zero',
            command=self._lc_capture_zero,
            bg=PRIMARY, fg='white',
            activebackground=PRIMARY_HV, activeforeground='white',
            font=FONT_LBL, relief='flat', bd=0, padx=14, pady=7,
            cursor='hand2',
        ).pack(side='left')
        self.lc_zero_status_lbl = tk.Label(
            zero_btn_row,
            text='Não capturado',
            font=FONT_MONO, bg=PANEL, fg=WARN)
        self.lc_zero_status_lbl.pack(side='left', padx=(14, 0))

        # ── Passo 2: wizard de calibração por tração (5 pesos) ──────────
        wiz_card = self._card(root, 'Passo 2 — Calibração por Tração (5 Pesos Conhecidos)')

        self.lc_step_lbl = tk.Label(
            wiz_card,
            text='Passo 1 de 5 — pendure/apoie o primeiro peso e insira a massa',
            font=FONT_LBL, bg=PANEL, fg=PRIMARY)
        self.lc_step_lbl.pack(anchor='w', pady=(0, 4))
        tk.Label(
            wiz_card,
            text='Compressão é derivada automaticamente por simetria da célula de carga',
            font=('Segoe UI', 9), bg=PANEL, fg=TEXT_DIM,
        ).pack(anchor='w', pady=(0, 8))

        # Massa
        mass_row = tk.Frame(wiz_card, bg=PANEL)
        mass_row.pack(fill='x', pady=(0, 4))
        tk.Label(mass_row, text='Massa do peso (tração)', font=FONT_LBL,
                 bg=PANEL, fg=TEXT).pack(side='left')
        self.lc_mass_var = tk.DoubleVar(value=0.100)
        tk.Spinbox(
            mass_row, from_=0.001, to=10.0, increment=0.001,
            textvariable=self.lc_mass_var, width=8, font=FONT_MONO,
            justify='right', relief='flat', bd=0,
            highlightthickness=1, highlightbackground=BORDER,
            highlightcolor=PRIMARY,
        ).pack(side='right', padx=(6, 0), ipady=2)
        tk.Label(mass_row, text='kg', font=FONT_LBL,
                 bg=PANEL, fg=TEXT_MUTED).pack(side='right')

        # Tensão live
        volt_row = tk.Frame(wiz_card, bg=PANEL)
        volt_row.pack(fill='x', pady=(2, 8))
        tk.Label(volt_row, text='Tensão atual (média últimas leituras)',
                 font=FONT_LBL, bg=PANEL, fg=TEXT_MUTED).pack(side='left')
        self.lc_volt_live_lbl = tk.Label(
            volt_row, text='—  V', font=FONT_MONO, bg=PANEL, fg=TEXT)
        self.lc_volt_live_lbl.pack(side='right')

        # Botão capturar
        self.lc_capture_btn = tk.Button(
            wiz_card, text='▶  Capturar Leitura',
            command=self._lc_calib_capture,
            bg=PRIMARY, fg='white',
            activebackground=PRIMARY_HV, activeforeground='white',
            font=FONT_HEAD, relief='flat', bd=0, padx=18, pady=10,
            cursor='hand2')
        self.lc_capture_btn.pack(fill='x', pady=(0, 10))

        tk.Frame(wiz_card, bg=BORDER, height=1).pack(fill='x', pady=4)

        # Lista de pontos capturados
        pts_frame = tk.Frame(wiz_card, bg=PANEL)
        pts_frame.pack(fill='x', pady=(6, 6))
        tk.Label(pts_frame, text='Pontos capturados:', font=FONT_LBL,
                 bg=PANEL, fg=TEXT_MUTED).pack(anchor='w', pady=(0, 4))
        self.lc_point_lbls = []
        for i in range(5):
            lbl = tk.Label(
                pts_frame,
                text=f'{i + 1}.  — kg  →  — N  →  — V',
                font=FONT_MONO_S, bg=PANEL, fg=TEXT_DIM, anchor='w')
            lbl.pack(fill='x', pady=1)
            self.lc_point_lbls.append(lbl)

        tk.Frame(wiz_card, bg=BORDER, height=1).pack(fill='x', pady=6)

        # Botões Calcular + Reiniciar
        btn_row = tk.Frame(wiz_card, bg=PANEL)
        btn_row.pack(fill='x', pady=(0, 6))
        self.lc_compute_btn = tk.Button(
            btn_row, text='✓  Calcular Calibração',
            command=self._lc_calib_compute,
            bg=OK, fg='white',
            activebackground=_shade(OK, -0.08), activeforeground='white',
            font=FONT_LBL, relief='flat', bd=0, padx=14, pady=8,
            cursor='hand2', state='disabled')
        self.lc_compute_btn.pack(side='left', fill='x', expand=True, padx=(0, 4))
        tk.Button(
            btn_row, text='↺  Reiniciar',
            command=self._lc_calib_reset,
            bg=BTN_NEUTRAL, fg=TEXT,
            activebackground=_shade(BTN_NEUTRAL, -0.08), activeforeground=TEXT,
            font=FONT_LBL, relief='flat', bd=0, padx=14, pady=8,
            cursor='hand2',
        ).pack(side='left', fill='x', expand=True, padx=(4, 0))

        self.lc_result_lbl = tk.Label(
            wiz_card, text='', font=FONT_MONO_S, bg=PANEL, fg=TEXT_DIM, anchor='w')
        self.lc_result_lbl.pack(fill='x', pady=(4, 0))

    # ── Restaura UI com calibração salva em disco ─────────────────────
    def _restore_lc_calib_ui(self) -> None:
        """Popula o wizard com os pontos e o resultado da calibração salva.

        Chamado uma única vez logo após a UI ser construída. Se não houver
        calibração em disco, não faz nada — o wizard fica no estado inicial.
        """
        if not self._lc_calibrated:
            return

        slope     = self._lc_calib_slope
        intercept = self._lc_calib_intercept
        points    = self._lc_calib_points   # [(mass_kg, v_sensor), ...]
        n         = len(points)
        zero_v    = self._lc_zero_voltage

        # ── Painel "Calibração Vigente" ──────────────────────────────
        zero_note = f'  | zero={zero_v:.4f} V' if zero_v is not None else '  | sem zero'
        self.lc_curr_calib_lbl.config(
            text=f'slope={slope:.4f}  intercept={intercept:.4f}'
                 f'  ({n} pontos){zero_note}',
            fg=OK)

        # Pontos salvos exibidos em verde no card "Calibração Vigente"
        for i in range(5):
            if i < len(points):
                mass_kg, v_sensor = points[i]
                force_n = mass_kg * 9.80665
                self.lc_saved_point_lbls[i].config(
                    text=f'  {i + 1}.  {mass_kg:.3f} kg  →  {force_n:.3f} N'
                         f'  →  {v_sensor:.4f} V  ✓',
                    fg=OK)
            else:
                self.lc_saved_point_lbls[i].config(text='')

        # ── Zero capturado ────────────────────────────────────────────
        if zero_v is not None:
            self.lc_zero_status_lbl.config(
                text=f'V₀ = {zero_v:.4f} V  ✓  (salvo)', fg=OK)

        # ── Pontos do wizard ─────────────────────────────────────────
        for i, (mass_kg, v_sensor) in enumerate(points[:5]):
            force_n = mass_kg * 9.80665
            self.lc_point_lbls[i].config(
                text=f'{i + 1}.  {mass_kg:.3f} kg  →  {force_n:.3f} N'
                     f'  →  {v_sensor:.4f} V  ✓',
                fg=OK)

        # ── R² recalculado (inclui zero se disponível) ────────────────
        all_forces   = [m * 9.80665 for m, _ in points]
        all_voltages = [v           for _, v in points]
        if zero_v is not None:
            all_forces   = [0.0]   + all_forces
            all_voltages = [zero_v] + all_voltages
        result_txt = f'slope={slope:.4f}  intercept={intercept:.4f}'
        if len(all_forces) >= 2:
            v_fit  = [slope * f + intercept for f in all_forces]
            ss_res = sum((v - vf) ** 2 for v, vf in zip(all_voltages, v_fit))
            v_mean = sum(all_voltages) / len(all_voltages)
            ss_tot = sum((v - v_mean) ** 2 for v in all_voltages)
            r2     = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 1.0
            result_txt += f'  R²={r2:.4f}'
        result_txt += zero_note
        self.lc_result_lbl.config(text=result_txt, fg=OK)

        # ── Step label e botões ───────────────────────────────────────
        if n >= 5:
            self.lc_step_lbl.config(
                text=f'Calibração carregada do disco — {n} pontos'
                     '  (use ↺ Reiniciar para refazer)',
                fg=OK)
            self.lc_capture_btn.config(state='disabled')
            self.lc_compute_btn.config(state='normal')
        else:
            self.lc_step_lbl.config(
                text=f'Calibração parcial carregada ({n} de 5 pontos) — '
                     f'Passo {n + 1}: posicione o próximo peso',
                fg=WARN)
            self.lc_capture_btn.config(state='normal')
            if n >= 2:
                self.lc_compute_btn.config(state='normal')

    # ── Tare (zeragem) da célula de carga ────────────────────────────
    def _lc_do_tare(self) -> None:
        with self._lock:
            buf = list(self._lc_voltage_buf)
            has_calib = self._lc_calibrated

        if not has_calib:
            self._set_status('Calibre o sensor antes de zerar.', WARN)
            return
        if len(buf) < 5:
            self._set_status(
                'Aguardando leituras do sensor — verifique a conexão UDP.', WARN)
            return

        avg_v = sum(buf) / len(buf)
        with self._lock:
            self._lc_tare_voltage = avg_v
            self._lc_tare_done = True

        self._set_status(
            f'Sensor zerado — tensão de referência: {avg_v:.4f} V.', OK)

    # ── Callbacks ROS — célula de carga ──────────────────────────────
    def _cb_lc_voltage(self, msg: Float32) -> None:
        with self._lock:
            self._lc_voltage = float(msg.data)
            self._lc_voltage_buf.append(float(msg.data))
            self._lc_last_ts = time.time()

    # ── Calibração — load / save ──────────────────────────────────────
    def _load_lc_calib(self) -> None:
        try:
            if not os.path.exists(LC_CALIB_FILE):
                return
            with open(LC_CALIB_FILE) as fh:
                data = json.load(fh)
            slope     = float(data['slope'])
            intercept = float(data['intercept'])
            n_pts     = int(data.get('n_points', 0))
            zero_v_raw = data.get('zero_voltage')
            zero_v     = float(zero_v_raw) if zero_v_raw is not None else None
            points    = [
                (float(p['mass_kg']), float(p['v_sensor']))
                for p in data.get('points', [])
            ]
            with self._lock:
                self._lc_calib_slope     = slope
                self._lc_calib_intercept = intercept
                self._lc_calibrated      = True
                self._lc_calib_n_pts     = n_pts
            self._lc_calib_points  = points
            self._lc_zero_voltage  = zero_v
            self.get_logger().info(
                f'Calibração LC: slope={slope:.4f} intercept={intercept:.6f} '
                f'zero={zero_v}  ({n_pts} pts)')
        except Exception as exc:
            self.get_logger().warn(f'Falha ao ler calibração LC: {exc}')

    def _save_lc_calib(self, slope: float, intercept: float,
                       zero_voltage: float | None = None) -> None:
        data = {
            'slope':        slope,
            'intercept':    intercept,
            'zero_voltage': zero_voltage,
            'n_points':     len(self._lc_calib_points),
            'points': [
                {'mass_kg': m,
                 'force_n': round(m * 9.80665, 4),
                 'v_sensor': v}
                for m, v in self._lc_calib_points
            ],
        }
        try:
            os.makedirs(os.path.dirname(LC_CALIB_FILE), exist_ok=True)
            with open(LC_CALIB_FILE, 'w') as fh:
                json.dump(data, fh, indent=2)
            self.get_logger().info(f'Calibração LC salva em {LC_CALIB_FILE}')
        except OSError as exc:
            self._set_status(f'Falha ao salvar calibração: {exc}', DANGER)

    # ── Calibração — wizard ───────────────────────────────────────────
    def _lc_capture_zero(self) -> None:
        """Captura a tensão do sensor sem nenhuma força aplicada (F = 0 N).

        Este ponto é incluído automaticamente na regressão, ancorando a reta
        na origem real do sensor e eliminando o offset residual do intercepto.
        """
        with self._lock:
            buf = list(self._lc_voltage_buf)
        if len(buf) < 5:
            self._set_status(
                'Aguardando leituras do sensor — verifique a conexão UDP.', WARN)
            return
        v0 = sum(buf) / len(buf)
        self._lc_zero_voltage = v0
        self.lc_zero_status_lbl.config(
            text=f'V₀ = {v0:.4f} V  ✓', fg=OK)
        self._set_status(f'Zero capturado: {v0:.4f} V (F = 0 N)', OK)

    def _lc_calib_capture(self) -> None:
        with self._lock:
            buf = list(self._lc_voltage_buf)

        if len(buf) < 5:
            self._set_status(
                'Aguardando leituras da célula de carga (verifique a conexão UDP).',
                WARN)
            return

        avg_v = sum(buf) / len(buf)

        try:
            mass_kg = float(self.lc_mass_var.get())
        except (ValueError, tk.TclError):
            self._set_status('Massa inválida.', DANGER)
            return
        if mass_kg <= 0.0:
            self._set_status('Informe uma massa positiva (peso em tração).', DANGER)
            return

        self._lc_calib_points.append((mass_kg, avg_v))
        idx     = len(self._lc_calib_points)
        force_n = mass_kg * 9.80665

        self.lc_point_lbls[idx - 1].config(
            text=f'{idx}.  {mass_kg:.3f} kg  →  {force_n:.3f} N  →  {avg_v:.4f} V  ✓',
            fg=OK)

        if idx >= 5:
            self.lc_step_lbl.config(
                text='5 pesos capturados — clique em Calcular Calibração', fg=OK)
            self.lc_capture_btn.config(state='disabled')
            self.lc_compute_btn.config(state='normal')
        else:
            self.lc_step_lbl.config(
                text=f'Passo {idx + 1} de 5 — pendure/apoie o próximo peso',
                fg=PRIMARY)
        self._set_status(
            f'Ponto {idx}/5: {mass_kg:.3f} kg → {avg_v:.4f} V', OK)

    def _lc_calib_compute(self) -> None:
        if len(self._lc_calib_points) < 2:
            self._set_status('Mínimo de 2 pontos para calibrar.', DANGER)
            return

        forces   = [m * 9.80665 for m, _v in self._lc_calib_points]
        voltages = [v            for _m, v in self._lc_calib_points]

        # Inclui o ponto de zero capturado: ancora a reta em (F=0, V=V₀),
        # eliminando o offset residual que aparece quando o intercepto não
        # corresponde à tensão real do sensor sem carga.
        zero_v = self._lc_zero_voltage
        if zero_v is not None:
            forces   = [0.0] + forces
            voltages = [zero_v] + voltages

        # Ajuste linear: v = slope * F + intercept → F = (v - intercept) / slope
        coeffs    = np.polyfit(forces, voltages, 1)
        slope     = float(coeffs[0])
        intercept = float(coeffs[1])

        v_fit  = np.polyval(coeffs, forces)
        ss_res = float(np.sum((np.array(voltages) - v_fit) ** 2))
        ss_tot = float(np.var(voltages)) * len(voltages)
        r2     = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 1.0

        with self._lock:
            self._lc_calib_slope     = slope
            self._lc_calib_intercept = intercept
            self._lc_calibrated      = True
            self._lc_calib_n_pts     = len(self._lc_calib_points)

        self._save_lc_calib(slope, intercept, zero_v)

        zero_note = f'  | zero={zero_v:.4f} V' if zero_v is not None else '  | sem zero'
        result = f'slope={slope:.4f}  intercept={intercept:.4f}  R²={r2:.4f}{zero_note}'
        self.lc_result_lbl.config(text=result, fg=OK)
        self._set_status(f'Calibração concluída! {result}', OK)

        # Atualiza card "Calibração Vigente" com os pontos em verde
        self.lc_curr_calib_lbl.config(
            text=f'slope={slope:.4f}  intercept={intercept:.4f}'
                 f'  ({len(self._lc_calib_points)} pontos){zero_note}',
            fg=OK)
        for i in range(5):
            if i < len(self._lc_calib_points):
                mass_kg, v_sensor = self._lc_calib_points[i]
                force_n = mass_kg * 9.80665
                self.lc_saved_point_lbls[i].config(
                    text=f'  {i + 1}.  {mass_kg:.3f} kg  →  {force_n:.3f} N'
                         f'  →  {v_sensor:.4f} V  ✓',
                    fg=OK)
            else:
                self.lc_saved_point_lbls[i].config(text='')

    def _lc_calib_reset(self) -> None:
        self._lc_calib_points = []
        self._lc_zero_voltage = None
        with self._lock:
            self._lc_calibrated = False
            self._lc_tare_done = False
            self._lc_tare_voltage = 0.0
        self.lc_mass_var.set(0.100)
        self.lc_zero_status_lbl.config(text='Não capturado', fg=WARN)
        self.lc_step_lbl.config(
            text='Passo 1 de 5 — pendure/apoie o primeiro peso e insira a massa',
            fg=PRIMARY)
        self.lc_capture_btn.config(state='normal')
        self.lc_compute_btn.config(state='disabled')
        self.lc_result_lbl.config(text='', fg=TEXT_DIM)
        for i, lbl in enumerate(self.lc_point_lbls):
            lbl.config(text=f'{i + 1}.  — kg  →  — N  →  — V', fg=TEXT_DIM)
        for lbl in self.lc_saved_point_lbls:
            lbl.config(text='')
        self._set_status('Calibração reiniciada.', TEXT_DIM)

    # ── Force receiver — gerenciamento do subprocesso ─────────────────
    def _toggle_force_receiver(self) -> None:
        if (self._force_rx_proc is not None
                and self._force_rx_proc.poll() is None):
            self._disconnect_force_receiver()
        else:
            self._connect_force_receiver()

    def _connect_force_receiver(self) -> None:
        cmd = ['ros2', 'run', 'touch_pack', 'force_receiver']
        try:
            self._force_rx_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid)
        except FileNotFoundError:
            self._force_rx_status_lbl.config(
                text='ros2 não encontrado — faça source do workspace',
                fg=DANGER)
            self._force_rx_proc = None
            return

        def _pipe_log(proc=self._force_rx_proc):
            for raw in proc.stdout:
                log.info('[FORCE-RX] %s',
                         raw.decode('utf-8', errors='replace').rstrip())
        threading.Thread(target=_pipe_log, daemon=True,
                         name='force-rx-log').start()

        self._force_rx_should_be_alive = True
        self._force_rx_btn.config(
            text='⏳  Iniciando…', state='disabled',
            bg=BTN_NEUTRAL, fg=TEXT)
        self._force_rx_status_lbl.config(
            text='Iniciando nó UDP…', fg=WARN)
        self.root.after(1500, self._post_connect_force_receiver)

    def _post_connect_force_receiver(self) -> None:
        proc = self._force_rx_proc
        if proc is None or proc.poll() is not None:
            self._force_rx_btn.config(
                text='⚡  Conectar', state='normal', bg=PRIMARY, fg='white')
            self._force_rx_status_lbl.config(
                text='Falha ao iniciar — verifique o workspace e o source',
                fg=DANGER)
            self._force_rx_proc = None
            self._force_rx_should_be_alive = False
            return
        self._force_rx_btn.config(
            text='⏹  Desconectar', state='normal', bg=DANGER, fg='white')
        self._force_rx_status_lbl.config(
            text='Nó ativo — aguardando pacotes UDP do ESP32 na porta 8080',
            fg=OK)

    def _disconnect_force_receiver(self) -> None:
        self._force_rx_should_be_alive = False
        proc = self._force_rx_proc
        self._force_rx_proc = None
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except OSError:
                    pass
            except OSError:
                pass
        self._force_rx_btn.config(
            text='⚡  Conectar', state='normal', bg=PRIMARY, fg='white')
        self._force_rx_status_lbl.config(
            text='Nó desconectado', fg=TEXT_DIM)
        self._esp32_dot_lbl.config(fg=TEXT_DIM)
        self._esp32_status_lbl.config(text='OFFLINE', fg=TEXT_DIM)

    # ── Refresh do painel de carga (Tk thread, 10 Hz) ─────────────────
    def _refresh_lc_panel(self):
        with self._lock:
            voltage    = self._lc_voltage
            calibrated = self._lc_calibrated
            slope      = self._lc_calib_slope
            intercept  = self._lc_calib_intercept
            n_pts      = self._lc_calib_n_pts
            last_ts    = self._lc_last_ts
            tare_done  = self._lc_tare_done
            tare_v     = self._lc_tare_voltage

        # Watchdog do force_receiver_node: detecta morte inesperada
        if self._force_rx_should_be_alive:
            proc = self._force_rx_proc
            if proc is not None and proc.poll() is not None:
                self._force_rx_proc = None
                self._force_rx_should_be_alive = False
                self._force_rx_btn.config(
                    text='⚡  Conectar', state='normal',
                    bg=PRIMARY, fg='white')
                self._force_rx_status_lbl.config(
                    text='Nó encerrou inesperadamente — clique Conectar para reiniciar',
                    fg=DANGER)

        # ESP32 connection status (timeout 2 s sem pacote UDP)
        has_data = last_ts > 0.0
        node_up  = self._force_rx_should_be_alive
        esp32_ok = has_data and (time.time() - last_ts) < 2.0
        if not node_up:
            self._esp32_dot_lbl.config(fg=TEXT_DIM)
            self._esp32_status_lbl.config(text='OFFLINE', fg=TEXT_DIM)
        elif esp32_ok:
            self._esp32_dot_lbl.config(fg=OK)
            self._esp32_status_lbl.config(text='ONLINE', fg=OK)
        elif has_data:
            self._esp32_dot_lbl.config(fg=DANGER)
            self._esp32_status_lbl.config(text='TIMEOUT', fg=DANGER)
        else:
            self._esp32_dot_lbl.config(fg=WARN)
            self._esp32_status_lbl.config(text='AGUARDANDO', fg=WARN)

        # Tensão
        volt_txt   = f'{voltage:.4f}  V' if has_data else '—  V'
        volt_color = TEXT if has_data else TEXT_DIM
        self.lc_voltage_lbl.config(text=volt_txt, fg=volt_color)
        self.lc_volt_live_lbl.config(text=volt_txt, fg=volt_color)

        # Força total calibrada (inclui preload estático)
        if calibrated and has_data and abs(slope) > 1e-9:
            force_total = (voltage - intercept) / slope
            self.lc_force_lbl.config(
                text=f'{force_total:6.2f}  N',
                fg=OK if abs(force_total) < 100 else WARN)
            self.lc_calib_status_lbl.config(
                text=f'Calibrado — {n_pts} pontos | '
                     f'slope={slope:.4f}  intercept={intercept:.4f}',
                fg=OK)

            # Força de compressão ⊥ mesa = derivada da calibração por tração,
            # sinal invertido: robô pressionando → V cai abaixo do tare → resultado positivo.
            if tare_done:
                f_compress = (tare_v - voltage) / slope   # positivo = compressão, negativo = tração
                color = OK if abs(f_compress) < 100 else WARN
                self.lc_normal_force_lbl.config(
                    text=f'{f_compress:+6.2f}  N', fg=color)
                self.lc_tare_status_lbl.config(
                    text=f'Ref. tare: {tare_v:.4f} V  '
                         f'| calibração por tração, compressão por simetria',
                    fg=OK)
            else:
                self.lc_normal_force_lbl.config(text='—   N', fg=TEXT_DIM)
                self.lc_tare_status_lbl.config(
                    text='Zeragem não realizada — clique Zerar Sensor antes de palpar',
                    fg=WARN)
        elif calibrated and not has_data:
            self.lc_force_lbl.config(text='—   N', fg=TEXT_DIM)
            self.lc_normal_force_lbl.config(text='—   N', fg=TEXT_DIM)
            self.lc_calib_status_lbl.config(
                text='Calibrado — aguardando dados do sensor (verifique UDP)',
                fg=WARN)
        else:
            self.lc_force_lbl.config(text='—   N', fg=TEXT_DIM)
            self.lc_normal_force_lbl.config(text='—   N', fg=TEXT_DIM)
            self.lc_calib_status_lbl.config(
                text='Não calibrado — use a aba Calibração para calibrar o sensor',
                fg=WARN)

        if calibrated:
            self.lc_curr_calib_lbl.config(
                text=f'slope={slope:.4f}  intercept={intercept:.4f}  ({n_pts} pontos)',
                fg=OK)
        else:
            self.lc_curr_calib_lbl.config(
                text='Nenhuma calibração salva', fg=WARN)

        self.root.after(100, self._refresh_lc_panel)

    # ──────────────────────────────────────────────────────────────────
    # Aba "Poses & Movimentos"
    # ──────────────────────────────────────────────────────────────────
    def _build_poses_tab(self, root: tk.Frame) -> None:
        """Layout dois-colunas: esquerda=Poses (fixa 310px), direita=Movimentos."""
        left = tk.Frame(root, bg=BG, width=310)
        left.pack(side='left', fill='y', padx=(12, 6), pady=12)
        left.pack_propagate(False)

        right = tk.Frame(root, bg=BG)
        right.pack(side='left', fill='both', expand=True, padx=(6, 12), pady=12)

        # ── LEFT: Poses ──────────────────────────────────────────────
        tk.Label(left, text='Poses', bg=BG, fg=TEXT, font=FONT_HEAD).pack(anchor='w')
        tk.Frame(left, bg=BORDER, height=1).pack(fill='x', pady=(4, 8))

        btn_row = tk.Frame(left, bg=BG)
        btn_row.pack(fill='x', pady=(0, 8))

        self._drag_btn = tk.Button(
            btn_row, text='🖐 Drag OFF',
            command=self._toggle_drag,
            bg=BTN_NEUTRAL, fg=TEXT,
            activebackground=_shade(BTN_NEUTRAL, -0.08),
            font=FONT_SMALL, relief='flat', bd=0, padx=8, pady=4,
            cursor='hand2')
        self._drag_btn.pack(side='left', padx=(0, 4))

        tk.Button(
            btn_row, text='📷 Robot',
            command=self._capture_pose_robot,
            bg=BTN_NEUTRAL, fg=TEXT,
            activebackground=_shade(BTN_NEUTRAL, -0.08),
            font=FONT_SMALL, relief='flat', bd=0, padx=8, pady=4,
            cursor='hand2').pack(side='left', padx=(0, 4))

        tk.Button(
            btn_row, text='🎮 Sim',
            command=self._capture_pose_sim,
            bg=BTN_NEUTRAL, fg=TEXT,
            activebackground=_shade(BTN_NEUTRAL, -0.08),
            font=FONT_SMALL, relief='flat', bd=0, padx=8, pady=4,
            cursor='hand2').pack(side='left')

        lbx_frame = tk.Frame(left, bg=BG)
        lbx_frame.pack(fill='both', expand=True)

        p_scroll = ttk.Scrollbar(lbx_frame, orient='vertical')
        p_scroll.pack(side='right', fill='y')

        self._poses_lbx = tk.Listbox(
            lbx_frame, yscrollcommand=p_scroll.set,
            bg=PANEL, fg=TEXT, font=FONT_MONO_S,
            selectbackground=PRIMARY, selectforeground='white',
            relief='flat', bd=0, highlightthickness=1,
            highlightbackground=BORDER, activestyle='none')
        self._poses_lbx.pack(side='left', fill='both', expand=True)
        p_scroll.config(command=self._poses_lbx.yview)

        pose_act = tk.Frame(left, bg=BG)
        pose_act.pack(fill='x', pady=(8, 0))

        tk.Button(
            pose_act, text='✏ Renomear',
            command=self._rename_selected_pose,
            bg=BTN_NEUTRAL, fg=TEXT,
            activebackground=_shade(BTN_NEUTRAL, -0.08),
            font=FONT_SMALL, relief='flat', bd=0, padx=8, pady=4,
            cursor='hand2').pack(side='left', padx=(0, 4))

        tk.Button(
            pose_act, text='🗑 Deletar',
            command=self._delete_selected_pose,
            bg=DANGER, fg='white',
            activebackground=DANGER_HV,
            font=FONT_SMALL, relief='flat', bd=0, padx=8, pady=4,
            cursor='hand2').pack(side='left')

        # ── RIGHT: Movimentos ─────────────────────────────────────────
        mov_hdr = tk.Frame(right, bg=BG)
        mov_hdr.pack(fill='x')

        tk.Label(mov_hdr, text='Movimentos', bg=BG, fg=TEXT,
                 font=FONT_HEAD).pack(side='left', anchor='w')

        tk.Button(
            mov_hdr, text='+ Novo',
            command=self._new_movement,
            bg=PRIMARY, fg='white',
            activebackground=PRIMARY_HV,
            font=FONT_SMALL, relief='flat', bd=0, padx=10, pady=4,
            cursor='hand2').pack(side='right')

        tk.Frame(right, bg=BORDER, height=1).pack(fill='x', pady=(4, 8))

        mov_lbx_frame = tk.Frame(right, bg=BG, height=120)
        mov_lbx_frame.pack(fill='x')
        mov_lbx_frame.pack_propagate(False)

        m_scroll = ttk.Scrollbar(mov_lbx_frame, orient='vertical')
        m_scroll.pack(side='right', fill='y')

        self._movs_lbx = tk.Listbox(
            mov_lbx_frame, yscrollcommand=m_scroll.set,
            bg=PANEL, fg=TEXT, font=FONT_MONO_S,
            selectbackground=PRIMARY, selectforeground='white',
            relief='flat', bd=0, highlightthickness=1,
            highlightbackground=BORDER, activestyle='none')
        self._movs_lbx.pack(side='left', fill='both', expand=True)
        m_scroll.config(command=self._movs_lbx.yview)
        self._movs_lbx.bind('<<ListboxSelect>>', self._on_movement_select)

        self._mov_detail_outer = tk.Frame(right, bg=BG)
        self._mov_detail_outer.pack(fill='both', expand=True, pady=(8, 0))

        self._refresh_poses_list()
        self._refresh_movements_list()

    # ── Dados: load / save ────────────────────────────────────────────
    def _load_poses_data(self) -> None:
        try:
            with open(POSES_FILE) as f:
                data = json.load(f)
            self._poses = data.get('poses', [])
            self._movements = data.get('movements', [])
            self._next_pose_id = max((p['id'] for p in self._poses), default=0) + 1
            self._next_movement_id = max(
                (m['id'] for m in self._movements), default=0) + 1
        except (FileNotFoundError, json.JSONDecodeError, KeyError):
            self._poses = []
            self._movements = []
            self._next_pose_id = 1
            self._next_movement_id = 1

    def _save_poses_data(self) -> None:
        os.makedirs(os.path.dirname(POSES_FILE), exist_ok=True)
        with open(POSES_FILE, 'w') as f:
            json.dump({'poses': self._poses, 'movements': self._movements},
                      f, indent=2)

    # ── Lookup helpers ────────────────────────────────────────────────
    def _pose_by_id(self, pid: int) -> dict | None:
        for p in self._poses:
            if p['id'] == pid:
                return p
        return None

    def _movement_by_id(self, mid: int) -> dict | None:
        for m in self._movements:
            if m['id'] == mid:
                return m
        return None

    def _pose_label(self, p: dict) -> str:
        q = p['q_deg']
        parts = ' '.join(f'J{i + 1}={v:+.0f}°' for i, v in enumerate(q))
        return f"{p['name']}  [{parts}]"

    # ── Refresh widgets ───────────────────────────────────────────────
    def _refresh_poses_list(self) -> None:
        lbx = self._poses_lbx
        if lbx is None:
            return
        lbx.delete(0, 'end')
        for p in self._poses:
            lbx.insert('end', self._pose_label(p))

    def _refresh_movements_list(self, select_id: int | None = None) -> None:
        lbx = self._movs_lbx
        if lbx is None:
            return
        lbx.delete(0, 'end')
        for m in self._movements:
            lbx.insert('end', m['name'])
        if select_id is not None:
            for i, m in enumerate(self._movements):
                if m['id'] == select_id:
                    lbx.selection_set(i)
                    lbx.see(i)
                    break

    def _on_movement_select(self, _event=None) -> None:
        lbx = self._movs_lbx
        if lbx is None:
            return
        sel = lbx.curselection()
        if not sel:
            return
        self._refresh_movement_detail(self._movements[sel[0]])

    def _refresh_movement_detail(self, mov: dict) -> None:
        outer = self._mov_detail_outer
        if outer is None:
            return
        if self._mov_detail_inner is not None:
            self._mov_detail_inner.destroy()

        inner = tk.Frame(outer, bg=PANEL,
                         highlightthickness=1, highlightbackground=BORDER)
        inner.pack(fill='both', expand=True)
        self._mov_detail_inner = inner

        # Header
        hdr = tk.Frame(inner, bg=PANEL)
        hdr.pack(fill='x', padx=12, pady=(10, 4))
        tk.Label(hdr, text=mov['name'], bg=PANEL, fg=TEXT,
                 font=FONT_HEAD).pack(side='left')
        tk.Button(hdr, text='✏',
                  command=lambda: self._rename_movement(mov['id']),
                  bg=BTN_NEUTRAL, fg=TEXT,
                  font=FONT_SMALL, relief='flat', bd=0,
                  padx=6, pady=2, cursor='hand2').pack(side='left', padx=(8, 0))
        tk.Button(hdr, text='🗑 Deletar',
                  command=lambda: self._delete_movement(mov['id']),
                  bg=DANGER, fg='white', activebackground=DANGER_HV,
                  font=FONT_SMALL, relief='flat', bd=0,
                  padx=8, pady=2, cursor='hand2').pack(side='right')
        tk.Frame(inner, bg=BORDER, height=1).pack(fill='x')

        body = tk.Frame(inner, bg=PANEL)
        body.pack(fill='both', expand=True, padx=12, pady=8)

        # ── Sequência ─────────────────────────────────────────────────
        seq_col = tk.Frame(body, bg=PANEL)
        seq_col.pack(side='left', fill='both', expand=True, padx=(0, 12))

        tk.Label(seq_col, text='Sequência de Poses', bg=PANEL, fg=TEXT_MUTED,
                 font=FONT_SMALL).pack(anchor='w')

        seq_frame = tk.Frame(seq_col, bg=PANEL)
        seq_frame.pack(fill='both', expand=True, pady=(4, 0))

        seq_sb = ttk.Scrollbar(seq_frame, orient='vertical')
        seq_sb.pack(side='right', fill='y')

        seq_lbx = tk.Listbox(
            seq_frame, yscrollcommand=seq_sb.set,
            bg=BG, fg=TEXT, font=FONT_MONO_S,
            selectbackground=PRIMARY, selectforeground='white',
            relief='flat', bd=0, highlightthickness=0,
            activestyle='none', height=6)
        seq_lbx.pack(side='left', fill='both', expand=True)
        seq_sb.config(command=seq_lbx.yview)

        def _refresh_seq():
            seq_lbx.delete(0, 'end')
            for pid in mov['pose_ids']:
                p = self._pose_by_id(pid)
                seq_lbx.insert('end', p['name'] if p else f'[deletada:{pid}]')

        _refresh_seq()

        def _add_pose_to_seq():
            lbx = self._poses_lbx
            if lbx is None:
                return
            sel = lbx.curselection()
            if not sel:
                self._set_status('Selecione uma pose na lista à esquerda.', WARN)
                return
            mov['pose_ids'].append(self._poses[sel[0]]['id'])
            _refresh_seq()
            self._save_poses_data()

        def _remove_pose_from_seq():
            sel = seq_lbx.curselection()
            if not sel:
                return
            idx = sel[0]
            if 0 <= idx < len(mov['pose_ids']):
                del mov['pose_ids'][idx]
                _refresh_seq()
                self._save_poses_data()

        def _move_up():
            sel = seq_lbx.curselection()
            if not sel:
                return
            i = sel[0]
            if i > 0:
                mov['pose_ids'][i - 1], mov['pose_ids'][i] = \
                    mov['pose_ids'][i], mov['pose_ids'][i - 1]
                _refresh_seq()
                seq_lbx.selection_set(i - 1)
                self._save_poses_data()

        def _move_down():
            sel = seq_lbx.curselection()
            if not sel:
                return
            i = sel[0]
            if i < len(mov['pose_ids']) - 1:
                mov['pose_ids'][i], mov['pose_ids'][i + 1] = \
                    mov['pose_ids'][i + 1], mov['pose_ids'][i]
                _refresh_seq()
                seq_lbx.selection_set(i + 1)
                self._save_poses_data()

        seq_btns = tk.Frame(seq_col, bg=PANEL)
        seq_btns.pack(fill='x', pady=(6, 0))

        for txt, cmd in [('+ Adicionar', _add_pose_to_seq),
                          ('−', _remove_pose_from_seq),
                          ('↑', _move_up),
                          ('↓', _move_down)]:
            tk.Button(seq_btns, text=txt, command=cmd,
                      bg=BTN_NEUTRAL, fg=TEXT,
                      activebackground=_shade(BTN_NEUTRAL, -0.08),
                      font=FONT_SMALL, relief='flat', bd=0,
                      padx=8, pady=3, cursor='hand2').pack(side='left', padx=(0, 4))

        # ── Controles + Execução ──────────────────────────────────────
        ctrl_col = tk.Frame(body, bg=PANEL, width=190)
        ctrl_col.pack(side='left', fill='y')
        ctrl_col.pack_propagate(False)

        tk.Label(ctrl_col, text='Velocidade (%)', bg=PANEL, fg=TEXT_MUTED,
                 font=FONT_SMALL).pack(anchor='w')
        spd_var = tk.IntVar(value=mov.get('speed_pct', 10))

        def _on_spd(*_):
            try:
                v = max(1, min(100, int(spd_var.get())))
                mov['speed_pct'] = v
                self._save_poses_data()
            except (ValueError, tk.TclError):
                pass

        tk.Spinbox(ctrl_col, from_=1, to=100, textvariable=spd_var,
                   width=7, font=FONT_MONO_S, relief='flat', bd=1,
                   command=_on_spd).pack(anchor='w', pady=(0, 10))
        spd_var.trace_add('write', _on_spd)

        tk.Label(ctrl_col, text='Duração/passo (s)', bg=PANEL, fg=TEXT_MUTED,
                 font=FONT_SMALL).pack(anchor='w')
        dur_var = tk.DoubleVar(value=mov.get('dur_s', 2.0))

        def _on_dur(*_):
            try:
                v = max(0.1, float(dur_var.get()))
                mov['dur_s'] = round(v, 2)
                self._save_poses_data()
            except (ValueError, tk.TclError):
                pass

        tk.Spinbox(ctrl_col, from_=0.1, to=60.0, increment=0.5,
                   textvariable=dur_var, width=7, format='%.1f',
                   font=FONT_MONO_S, relief='flat', bd=1,
                   command=_on_dur).pack(anchor='w', pady=(0, 16))
        dur_var.trace_add('write', _on_dur)

        _mid = mov['id']
        tk.Button(ctrl_col, text='▶ Executar',
                  command=lambda: self._start_movement(_mid, loop=False),
                  bg=OK, fg='white', activebackground='#15803d',
                  font=FONT_SMALL, relief='flat', bd=0,
                  padx=8, pady=4, cursor='hand2').pack(fill='x', pady=(0, 4))
        tk.Button(ctrl_col, text='🔄 Loop',
                  command=lambda: self._start_movement(_mid, loop=True),
                  bg=WARN, fg='white', activebackground='#b45309',
                  font=FONT_SMALL, relief='flat', bd=0,
                  padx=8, pady=4, cursor='hand2').pack(fill='x', pady=(0, 4))

        tk.Button(ctrl_col, text='⏹ Parar',
                  command=self._stop_execution,
                  bg=DANGER, fg='white', activebackground=DANGER_HV,
                  font=FONT_SMALL, relief='flat', bd=0,
                  padx=8, pady=4, cursor='hand2').pack(fill='x')

    # ── Captura de poses ──────────────────────────────────────────────
    def _capture_pose_robot(self) -> None:
        drv = self._real_driver
        if drv is None or not self._robot_connected:
            self._set_status('Robô real não conectado — use 🎮 Sim.', WARN)
            return
        try:
            q_urdf = drv.read_joints_urdf()
            q_deg = [math.degrees(float(v)) for v in q_urdf]
            self._add_pose(q_deg, prefix='Robot')
        except Exception as exc:
            self._set_status(f'Erro ao capturar pose real: {exc}', DANGER)

    def _capture_pose_sim(self) -> None:
        positions = self._latest_joint_rad
        if positions is None:
            self._set_status('Sem leitura de /joint_states — inicie a simulação.', WARN)
            return
        q_deg = [math.degrees(float(v)) for v in positions]
        self._add_pose(q_deg, prefix='Sim')

    def _add_pose(self, q_deg: list, prefix: str = 'Pose') -> None:
        pid = self._next_pose_id
        self._next_pose_id += 1
        name = f'{prefix} {pid}'
        pose = {'id': pid, 'name': name,
                'q_deg': [round(float(v), 2) for v in q_deg[:6]]}
        self._poses.append(pose)
        self._save_poses_data()
        self._refresh_poses_list()
        self._set_status(f'Pose "{name}" capturada.', OK)

    # ── Ações nas poses ───────────────────────────────────────────────
    def _rename_selected_pose(self) -> None:
        lbx = self._poses_lbx
        if lbx is None:
            return
        sel = lbx.curselection()
        if not sel:
            self._set_status('Selecione uma pose para renomear.', WARN)
            return
        pose = self._poses[sel[0]]
        new_name = self._ask_name_dialog('Renomear Pose', pose['name'])
        if new_name:
            pose['name'] = new_name
            self._save_poses_data()
            self._refresh_poses_list()

    def _delete_selected_pose(self) -> None:
        lbx = self._poses_lbx
        if lbx is None:
            return
        sel = lbx.curselection()
        if not sel:
            self._set_status('Selecione uma pose para deletar.', WARN)
            return
        pose = self._poses[sel[0]]
        pid = pose['id']
        for m in self._movements:
            m['pose_ids'] = [x for x in m['pose_ids'] if x != pid]
        self._poses.pop(sel[0])
        self._save_poses_data()
        self._refresh_poses_list()
        self._set_status(f'Pose "{pose["name"]}" deletada.', OK)

    # ── Drag teach ────────────────────────────────────────────────────
    def _toggle_drag(self) -> None:
        drv = self._real_driver
        if drv is None or not self._robot_connected:
            self._set_status('Drag teach requer robô real conectado.', WARN)
            return
        new_state = not self._drag_enabled
        sw_ok = False  # DragTeachSwitch via TCP conseguiu
        try:
            drv.drag_teach(new_state)
            sw_ok = True
        except Exception as exc:
            if new_state:
                # TCP falhou (tipicamente: controlador em modo LOCAL / sem token).
                # Ativa apenas o tracking real→sim: o utilizador aciona o drag
                # fisicamente no botão do antebraço e a simulação acompanha.
                self.get_logger().warning(
                    f'DragTeachSwitch TCP falhou ({exc}). '
                    'Tracking real→sim ativo — active o drag físico no robô.')
            else:
                self._set_status(f'DragTeachSwitch falhou: {exc}', DANGER)
                return
        self._drag_enabled = new_state
        btn = self._drag_btn
        if btn is not None:
            if new_state:
                label = '🖐 Drag ON' if sw_ok else '🖐 Drag (físico)'
                btn.config(text=label, bg=WARN, fg='white',
                           activebackground='#b45309')
            else:
                btn.config(text='🖐 Drag OFF', bg=BTN_NEUTRAL, fg=TEXT,
                           activebackground=_shade(BTN_NEUTRAL, -0.08))
        if new_state:
            msg = 'Drag Teach ATIVO.' if sw_ok else \
                'Drag físico: ative o botão no robô — simulação a seguir o braço real.'
            self._set_status(msg, WARN)
        else:
            self._set_status('Drag Teach desativado.', OK)

    # ── Ações nos movimentos ──────────────────────────────────────────
    def _new_movement(self) -> None:
        name = self._ask_name_dialog(
            'Novo Movimento', f'Movimento {self._next_movement_id}')
        if name is None:
            return
        mid = self._next_movement_id
        self._next_movement_id += 1
        mov = {'id': mid, 'name': name, 'pose_ids': [],
               'speed_pct': 10, 'dur_s': 2.0}
        self._movements.append(mov)
        self._save_poses_data()
        self._refresh_movements_list(select_id=mid)
        self._refresh_movement_detail(mov)

    def _rename_movement(self, mov_id: int) -> None:
        mov = self._movement_by_id(mov_id)
        if mov is None:
            return
        new_name = self._ask_name_dialog('Renomear Movimento', mov['name'])
        if new_name:
            mov['name'] = new_name
            self._save_poses_data()
            self._refresh_movements_list(select_id=mov_id)
            self._refresh_movement_detail(mov)

    def _delete_movement(self, mov_id: int) -> None:
        mov = self._movement_by_id(mov_id)
        if mov is None:
            return
        name = mov['name']
        self._movements = [m for m in self._movements if m['id'] != mov_id]
        self._save_poses_data()
        self._refresh_movements_list()
        if self._mov_detail_inner is not None:
            self._mov_detail_inner.destroy()
            self._mov_detail_inner = None
        self._set_status(f'Movimento "{name}" deletado.', OK)

    # ── Execução de movimentos ────────────────────────────────────────
    def _start_movement(self, mov_id: int, loop: bool = False) -> None:
        if self._exec_thread is not None and self._exec_thread.is_alive():
            self._set_status('Execução em andamento — pare primeiro.', WARN)
            return
        mov = self._movement_by_id(mov_id)
        if mov is None:
            return
        if not mov['pose_ids']:
            self._set_status('Adicione poses à sequência antes de executar.', WARN)
            return
        self._exec_stop.clear()
        self._exec_movement_id = mov_id
        self._exec_thread = threading.Thread(
            target=self._execute_movement_worker,
            args=(dict(mov), loop),
            daemon=True, name='exec-movement')
        self._exec_thread.start()
        suffix = '  (loop)' if loop else ''
        self._set_status(f'Executando "{mov["name"]}"{suffix}...', OK)

    def _stop_execution(self) -> None:
        self._exec_stop.set()
        # Não limpa _exec_movement_id aqui — o finally do worker faz isso.
        # Isso evita que _mirror_poll_loop retome ServoJ antes do MovJ atual terminar.
        if (self._robot_mode == 'MIRROR' and self._robot_connected
                and self._real_driver is not None):
            try:
                self._real_driver.halt()
            except Exception:
                pass
        self._set_status('Execução interrompida.', WARN)

    def _execute_movement_worker(self, mov: dict, loop: bool) -> None:
        try:
            self._run_movement_once(mov)
            while loop and not self._exec_stop.is_set():
                self._run_movement_once(mov)
        except Exception as exc:
            log.warning('Execução de movimento falhou: %s', exc)
            self.root.after(
                0, lambda: self._set_status(f'Execução falhou: {exc}', DANGER))
        finally:
            self._exec_movement_id = None

    def _run_movement_once(self, mov: dict) -> None:
        """Executa uma passagem completa pelo movimento.

        Gazebo e robô real executam em paralelo: a trajetória multi-ponto é
        publicada de uma vez no Gazebo; o robô real recebe MovJ por passo,
        ambos cadenciados por `dur_s` segundos por pose.
        """
        dur_s = max(0.1, mov.get('dur_s', 2.0))
        speed_pct = max(1, min(100, mov.get('speed_pct', 10)))
        poses = [self._pose_by_id(pid) for pid in mov['pose_ids']]
        poses = [p for p in poses if p is not None]
        if not poses:
            return

        mode = self._robot_mode

        if mode in ('SIM_ONLY', 'MIRROR'):
            # Publica trajetória completa no Gazebo de uma vez.
            msg = JointTrajectory()
            msg.joint_names = ARM_JOINTS
            for i, pose in enumerate(poses):
                pt = JointTrajectoryPoint()
                pt.positions = [math.radians(float(v)) for v in pose['q_deg']]
                pt.velocities = [0.0] * 6
                total_s = (i + 1) * dur_s
                pt.time_from_start = Duration(
                    sec=int(total_s),
                    nanosec=int((total_s % 1.0) * 1_000_000_000))
                msg.points.append(pt)
            self._arm_pub.publish(msg)

        if mode == 'MIRROR':
            # Robô real: MovJ por pose, cadenciado por dur_s — paralelo ao Gazebo.
            drv = self._real_driver
            if drv is not None and self._robot_connected:
                try:
                    drv._send_dash(f'SpeedFactor({speed_pct})')
                except Exception:
                    pass
                for pose in poses:
                    if self._exec_stop.is_set():
                        break
                    t_step_start = time.monotonic()
                    try:
                        if _urdf_to_dobot is not None:
                            q_urdf = np.array(
                                [math.radians(float(v)) for v in pose['q_deg']])
                            q_dobot_deg = np.degrees(_urdf_to_dobot(q_urdf)).tolist()
                        else:
                            q_dobot_deg = list(pose['q_deg'])
                        drv.mov_j_joint_deg(q_dobot_deg)
                    except Exception as exc:
                        log.warning('MovJ falhou: %s', exc)
                        break
                    # Aguarda o restante de dur_s para este passo,
                    # verificando _exec_stop a cada 100 ms.
                    deadline = t_step_start + dur_s
                    while not self._exec_stop.is_set():
                        remaining = deadline - time.monotonic()
                        if remaining <= 0.0:
                            break
                        self._exec_stop.wait(min(0.1, remaining))
        elif mode == 'SIM_ONLY':
            # Aguarda a trajetória completa ou sinalização de stop.
            self._exec_stop.wait(len(poses) * dur_s)

    # ── Diálogo de nome ───────────────────────────────────────────────
    def _ask_name_dialog(self, title: str, initial: str = '') -> str | None:
        result: list[str | None] = [None]
        dlg = tk.Toplevel(self.root)
        dlg.title(title)
        dlg.configure(bg=BG)
        dlg.resizable(False, False)
        dlg.grab_set()

        tk.Label(dlg, text=title, bg=BG, fg=TEXT, font=FONT_HEAD
                 ).pack(padx=24, pady=(16, 8))
        var = tk.StringVar(value=initial)
        entry = tk.Entry(dlg, textvariable=var, font=FONT_LBL, width=32)
        entry.pack(padx=24, pady=(0, 8))
        entry.select_range(0, 'end')
        entry.focus_set()

        def _ok(_=None):
            val = var.get().strip()
            if val:
                result[0] = val
            dlg.destroy()

        def _cancel(_=None):
            dlg.destroy()

        row = tk.Frame(dlg, bg=BG)
        row.pack(pady=(0, 16))
        tk.Button(row, text='OK', command=_ok,
                  bg=PRIMARY, fg='white', font=FONT_LBL,
                  relief='flat', bd=0, padx=16, pady=4,
                  cursor='hand2').pack(side='left', padx=4)
        tk.Button(row, text='Cancelar', command=_cancel,
                  bg=BTN_NEUTRAL, fg=TEXT, font=FONT_LBL,
                  relief='flat', bd=0, padx=16, pady=4,
                  cursor='hand2').pack(side='left', padx=4)
        entry.bind('<Return>', _ok)
        entry.bind('<Escape>', _cancel)
        dlg.wait_window()
        return result[0]

    def _build_statusbar(self):
        self.status_var = tk.StringVar(value='pronto.')
        bar = tk.Frame(self.root, bg=PANEL, height=28)
        bar.pack(side='bottom', fill='x')
        self._status_lbl = tk.Label(bar, textvariable=self.status_var,
                                     bg=PANEL, fg=TEXT_MUTED,
                                     anchor='w', font=FONT_LBL)
        self._status_lbl.pack(side='left', padx=18)

    # ── helpers UI ────────────────────────────────────────────────────
    def _card(self, parent, title: str) -> tk.Frame:
        card = tk.Frame(parent, bg=PANEL,
                         highlightthickness=1,
                         highlightbackground=BORDER,
                         highlightcolor=BORDER)
        card.pack(fill='both', expand=True)
        tk.Label(card, text=title, bg=PANEL, fg=TEXT, font=FONT_HEAD,
                 anchor='w').pack(fill='x', padx=14, pady=(10, 6))
        tk.Frame(card, bg=BORDER, height=1).pack(fill='x')
        inner = tk.Frame(card, bg=PANEL)
        inner.pack(fill='both', expand=True, padx=14, pady=10)
        return inner

    def _kv(self, parent, key: str, val: str) -> tk.Label:
        row = tk.Frame(parent, bg=PANEL); row.pack(fill='x', pady=1)
        tk.Label(row, text=key, font=FONT_LBL, bg=PANEL, fg=TEXT_MUTED
                 ).pack(side='left')
        lbl = tk.Label(row, text=val, font=FONT_MONO, bg=PANEL, fg=TEXT)
        lbl.pack(side='right')
        return lbl

    def _build_slide_dir_selector(self, parent) -> None:
        """Segmented control (4 botões mutex) para a direção do sliding."""
        row = tk.Frame(parent, bg=PANEL); row.pack(fill='x', pady=(8, 2))
        top = tk.Frame(row, bg=PANEL); top.pack(fill='x')
        tk.Label(top, text='Direção do Deslizamento', font=FONT_LBL,
                 bg=PANEL, fg=TEXT, anchor='w').pack(side='left')
        btns = tk.Frame(row, bg=PANEL); btns.pack(fill='x', pady=(4, 2))
        self._slide_dir_btns: dict[str, tk.Button] = {}
        for d in ('+X', '-X', '+Y', '-Y'):
            b = tk.Button(btns, text=d,
                          command=lambda dd=d: self._on_slide_dir(dd),
                          bg=BTN_NEUTRAL, fg=TEXT, font=FONT_MONO,
                          activebackground=PRIMARY_HV,
                          activeforeground='white',
                          relief='flat', bd=0, padx=14, pady=6,
                          cursor='hand2')
            b.pack(side='left', fill='x', expand=True,
                   padx=(0 if d == '+X' else 4, 0))
            self._slide_dir_btns[d] = b
        tk.Label(row,
                 text='Arrasto Cartesiano retilíneo em XY (mundo); as '
                      'juntas se coordenam para preservar Z e orientação.',
                 font=('Segoe UI', 9), bg=PANEL, fg=TEXT_DIM,
                 anchor='w').pack(fill='x', pady=(2, 0))
        self._on_slide_dir(self.slide_dir_var.get())

    def _on_slide_dir(self, d: str) -> None:
        if d not in ('+X', '-X', '+Y', '-Y'):
            return
        self.slide_dir_var.set(d)
        for k, b in self._slide_dir_btns.items():
            if k == d:
                b.config(bg=PRIMARY, fg='white')
            else:
                b.config(bg=BTN_NEUTRAL, fg=TEXT)

    def _param_row(self, parent, *, label, unit, var,
                    vmin, vmax, step, hint=''):
        row = tk.Frame(parent, bg=PANEL); row.pack(fill='x', pady=(4, 2))
        top = tk.Frame(row, bg=PANEL); top.pack(fill='x')
        tk.Label(top, text=label, font=FONT_LBL, bg=PANEL, fg=TEXT,
                 anchor='w').pack(side='left')
        tk.Spinbox(top, from_=vmin, to=vmax, increment=step,
                    textvariable=var, width=8, font=FONT_MONO,
                    justify='right', relief='flat', bd=0,
                    highlightthickness=1, highlightbackground=BORDER,
                    highlightcolor=PRIMARY
                    ).pack(side='right', padx=(6, 0), ipady=2)
        tk.Label(top, text=unit, font=FONT_LBL, bg=PANEL, fg=TEXT_MUTED
                 ).pack(side='right')
        ttk.Scale(row, from_=vmin, to=vmax, variable=var,
                   orient='horizontal',
                   style='Tactile.Horizontal.TScale'
                   ).pack(fill='x', pady=(2, 0))
        if hint:
            tk.Label(row, text=hint, font=('Segoe UI', 9),
                     bg=PANEL, fg=TEXT_DIM, anchor='w'
                     ).pack(fill='x', pady=(0, 4))

    # ──────────────────────────────────────────────────────────────────
    # MÃO COVVI — conexão / ECI / PWR
    # ──────────────────────────────────────────────────────────────────
    def _connect_real_hand(self) -> None:
        """Sobe `ros2 run covvi_hand_driver server <IP>` em subprocesso."""
        if self._hand_proc is not None and self._hand_proc.poll() is None:
            self._disconnect_real_hand()
            return
        ip = (self._hand_ip_var.get() or '').strip()
        if not ip:
            self._set_status('Informe o IP da mão COVVI.', DANGER)
            return
        # Quebra o eci_prefix em namespace + node name, igual ao manual_control_node
        # do grasp_ml_pack (referência funcional). Com __ns:=/covvi e __name:=hand,
        # o driver expõe os serviços em /covvi/hand/SetCurrentGrip etc.
        parts = self._eci_prefix.strip('/').split('/')
        _ns   = '/' + parts[0]
        _name = parts[1] if len(parts) > 1 else 'server'
        cmd = ['ros2', 'run', 'covvi_hand_driver', 'server', ip,
               '--ros-args',
               '--remap', f'__ns:={_ns}',
               '--remap', f'__name:={_name}']
        log.warning('[DBG] _connect_real_hand: cmd=%s', cmd)
        try:
            self._hand_proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, preexec_fn=os.setsid)
            # Thread daemon lê stdout+stderr do driver e redireciona para o log
            def _pipe_hand_log(proc=self._hand_proc):
                for raw in proc.stdout:
                    line = raw.decode('utf-8', errors='replace').rstrip()
                    log.warning('[HAND-PROC] %s', line)
            threading.Thread(target=_pipe_hand_log, daemon=True,
                             name='hand-proc-log').start()
        except FileNotFoundError:
            self._set_status('ros2 não está no PATH (source o workspace).',
                              DANGER)
            self._hand_proc = None
            return
        self._hand_should_be_alive = True
        self._start_hand_watchdog()
        self._set_status(f'covvi_hand_driver server {ip} iniciando…', PRIMARY)
        self.root.after(2200, self._post_connect_real_hand)

    def _post_connect_real_hand(self) -> None:
        proc = self._hand_proc
        if proc is None or proc.poll() is not None:
            self._set_status(
                'Driver da mão falhou ao subir — verifique o IP / ECI box.',
                DANGER)
            self._hand_proc = None
            return
        self._hand_connect_btn.set_state('⚡', 'Desconectar', OK, 'white')
        # Conexão deu certo — persistir o IP para reusar no próximo boot.
        self._save_robot_config()
        # Ativa ECI automaticamente (como o manual_control_node do grasp_ml_pack)
        # _toggle_eci já agenda o auto-power-on em 800 ms
        if not self._eci_enabled:
            self._toggle_eci()
        self._set_status(
            f'Driver da mão ativo ({self._eci_prefix}) — power ON em breve…', OK)

    def _disconnect_real_hand(self) -> None:
        """Inicia desconexão limpa da mão COVVI.

        A UI é atualizada imediatamente; PowerOff + SIGTERM/wait correm em
        thread daemon para não congelar o Tkinter (wait do subprocesso pode
        levar até ~3 s quando o driver está ocupado).
        """
        self._hand_should_be_alive = False
        self._stop_hand_watchdog()

        eci_was_enabled = self._eci_enabled
        self._eci_enabled = False
        self._hand_powered = False
        self._eci_btn.set_state('◉', 'ECI OFF', BTN_NEUTRAL, TEXT)
        self._pwr_btn.set_state('⏻', 'PWR OFF', BTN_NEUTRAL, TEXT)
        self._hand_connect_btn.set_state('⏳', 'Desconectando…', BTN_NEUTRAL, TEXT)
        self._set_status('Desconectando mão COVVI…', TEXT_DIM)

        threading.Thread(
            target=self._disconnect_hand_worker,
            args=(eci_was_enabled,), daemon=True).start()

    def _disconnect_hand_worker(self, eci_was_enabled: bool) -> None:
        """Thread daemon: PowerOff síncrono → SIGINT/SIGTERM → wait → pausa — não bloqueia a GUI."""
        if eci_was_enabled:
            self._send_hand_poweroff_blocking(timeout_s=3.0)
        self._terminate_hand_subprocess()
        # A caixa ECI precisa de ~15 s para liberar o estado TCP (TIME_WAIT)
        # após a conexão ser quebrada — conectar antes causa
        # ExistingConnectionError. Exibimos contagem regressiva na status bar.
        ECI_RESET_S = 15
        for remaining in range(ECI_RESET_S, 0, -1):
            self.root.after(0, lambda r=remaining: self._set_status(
                f'Aguardando reset da caixa ECI — ainda {r} s…', TEXT_DIM))
            time.sleep(1.0)
        self.root.after(0, self._finish_hand_disconnect)

    def _finish_hand_disconnect(self) -> None:
        """Callback Tkinter: atualiza botão após o worker de desconexão concluir."""
        self._hand_connect_btn.set_state('⚡', 'Conectar', PRIMARY, 'white')
        self._set_status('Driver da mão desconectado — LED apagado.', TEXT_DIM)

    # ── Watchdog + re-spawn automático (mão COVVI) ───────────────────
    def _start_hand_watchdog(self) -> None:
        thr = self._hand_watchdog_thread
        if thr is not None and thr.is_alive():
            return
        self._hand_watchdog_stop.clear()
        self._hand_watchdog_thread = threading.Thread(
            target=self._hand_watchdog_loop, daemon=True)
        self._hand_watchdog_thread.start()

    def _stop_hand_watchdog(self) -> None:
        self._hand_watchdog_stop.set()
        thr = self._hand_watchdog_thread
        if thr is not None and thr is not threading.current_thread():
            thr.join(timeout=0.5)
        self._hand_watchdog_thread = None

    def _hand_watchdog_loop(self) -> None:
        """Poll @2 s do `Popen.poll()`. Se o driver morrer sem desconexão
        deliberada, dispara re-spawn no thread Tk."""
        WATCHDOG_PERIOD_S = 2.0
        while not self._hand_watchdog_stop.is_set():
            if self._hand_watchdog_stop.wait(WATCHDOG_PERIOD_S):
                return
            if not self._hand_should_be_alive:
                return
            proc = self._hand_proc
            if proc is None:
                continue   # ainda subindo / já encerrado
            if proc.poll() is not None:
                self.get_logger().error(
                    f'covvi_hand_driver morreu (rc={proc.returncode}). '
                    'Tentando re-spawn automático.')
                self.root.after(0, self._on_hand_died)
                return

    def _on_hand_died(self) -> None:
        """Callback Tk: limpa estado interno (ECI/power perdidos junto com
        o driver) e tenta reconectar. Preserva `_hand_should_be_alive`
        para o watchdog seguir monitorando após o re-spawn."""
        if not self._hand_should_be_alive:
            return
        # Estado de software (já estava out-of-sync com o driver morto).
        self._hand_proc = None
        self._eci_enabled = False
        self._hand_powered = False
        self._eci_btn.set_state('◉', 'ECI OFF', BTN_NEUTRAL, TEXT)
        self._pwr_btn.set_state('⏻', 'PWR OFF', BTN_NEUTRAL, TEXT)
        self._hand_connect_btn.set_state('⏳', 'Reconectando…', WARN, 'white')
        # Aguarda 15 s antes de re-spawnar: a caixa ECI precisa desse tempo
        # para liberar o estado TCP após a conexão quebrada (ExistingConnectionError).
        self._set_status(
            'Driver da mão caiu — re-spawn automático em 15 s…', WARN)
        self.root.after(15000, self._on_hand_respawn)

    def _on_hand_respawn(self) -> None:
        """Callback Tk: re-spawn da mão após o delay de reset da caixa ECI."""
        if not self._hand_should_be_alive:
            return
        self._set_status('Re-spawn automático do driver da mão…', WARN)
        self._connect_real_hand()

    def _send_hand_poweroff_blocking(self, timeout_s: float) -> None:
        """Chama SetHandPowerOff e espera o future completar (com timeout).

        Rodamos no thread Tkinter; o spin do ROS está na thread auxiliar,
        então o future é resolvido em paralelo. Apenas dormimos checando
        `future.done()` em intervalos curtos para não congelar a UI."""
        if self._cli_hand_pwr_off is None or self._eci_srv is None:
            return
        try:
            if not self._cli_hand_pwr_off.service_is_ready():
                # Sem serviço pronto não há como cortar o power via ECI;
                # ainda assim seguimos para o SIGTERM.
                return
            future = self._cli_hand_pwr_off.call_async(
                self._eci_srv.SetHandPowerOff.Request())
        except Exception as exc:
            self.get_logger().warning(f'PowerOff falhou: {exc}')
            return
        deadline = time.time() + max(0.05, timeout_s)
        while time.time() < deadline:
            if future.done():
                return
            time.sleep(0.02)
        self.get_logger().warning(
            f'PowerOff não concluiu em {timeout_s:.1f} s — '
            'driver será terminado mesmo assim.')

    def _terminate_hand_subprocess(self) -> None:
        """SIGINT → espera 2 s (shutdown ROS2 gracioso, fecha sockets ECI);
        se ainda vivo, SIGTERM → espera 2 s; por último SIGKILL. Idempotente."""
        proc = self._hand_proc
        self._hand_proc = None
        if proc is None or proc.poll() is not None:
            return
        # SIGINT first: triggers rclpy shutdown handlers → sockets closed cleanly
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except (OSError, ProcessLookupError) as exc:
            self.get_logger().debug(f'SIGINT da mão ignorado ({exc}).')
        try:
            proc.wait(timeout=2.0)
            return
        except subprocess.TimeoutExpired:
            pass
        # Fallback: SIGTERM
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except (OSError, ProcessLookupError) as exc:
            self.get_logger().debug(f'SIGTERM da mão ignorado ({exc}).')
        try:
            proc.wait(timeout=2.0)
            return
        except subprocess.TimeoutExpired:
            self.get_logger().warn(
                'Driver da mão não saiu em 2 s após SIGTERM — forçando SIGKILL.')
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except (OSError, ProcessLookupError):
            pass
        try:
            proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            self.get_logger().error(
                'Driver da mão ficou zumbi após SIGKILL.')

    def _toggle_eci(self) -> None:
        """Liga/desliga o canal lógico ECI (cliente dos serviços COVVI).

        Precisa do pacote `covvi_interfaces` sourceado no workspace.
        ECI OFF corta a alimentação da mão imediatamente (LED azul apaga).
        ECI ON ativa os clientes e auto-liga a alimentação após 800 ms.
        """
        if self._eci_enabled:
            # Cortar alimentação antes de desativar o canal
            if self._hand_powered and self._cli_hand_pwr_off is not None:
                try:
                    if self._cli_hand_pwr_off.service_is_ready():
                        self._cli_hand_pwr_off.call_async(
                            self._eci_srv.SetHandPowerOff.Request())
                except Exception:
                    pass
            self._hand_powered = False
            self._pwr_btn.set_state('⏻', 'PWR OFF', BTN_NEUTRAL, TEXT)
            self._eci_enabled = False
            self._eci_btn.set_state('◉', 'ECI OFF', BTN_NEUTRAL, TEXT)
            self._set_status('Canal ECI desativado — alimentação cortada.', TEXT_DIM)
            return
        try:
            import covvi_interfaces.srv as _eci_srv
            import covvi_interfaces.msg as _eci_msg
        except ImportError:
            self._set_status(
                'covvi_interfaces não disponível — source o workspace.',
                DANGER)
            return
        self._eci_srv = _eci_srv
        self._eci_msg = _eci_msg
        # Nomes CamelCase conforme o covvi_hand_driver expõe no grafo ROS2
        if self._cli_eci_grip is None:
            self._cli_eci_grip = self.create_client(
                _eci_srv.SetCurrentGrip,
                f'{self._eci_prefix}/SetCurrentGrip')
        if self._cli_eci_posn is None:
            self._cli_eci_posn = self.create_client(
                _eci_srv.SetDigitPosn,
                f'{self._eci_prefix}/SetDigitPosn')
        if self._cli_hand_pwr_on is None:
            self._cli_hand_pwr_on = self.create_client(
                _eci_srv.SetHandPowerOn,
                f'{self._eci_prefix}/SetHandPowerOn')
        if self._cli_hand_pwr_off is None:
            self._cli_hand_pwr_off = self.create_client(
                _eci_srv.SetHandPowerOff,
                f'{self._eci_prefix}/SetHandPowerOff')
        self._eci_enabled = True
        self._eci_btn.set_state('◉', 'ECI ON', OK, 'white')
        self._set_status('Canal ECI ativo — aguardando power da mão…', OK)
        # Aguarda o driver registrar os serviços no grafo ROS2 antes de ligar
        self.root.after(800, self._auto_power_on_hand)

    def _auto_power_on_hand(self) -> None:
        """Auto-power-on da mão 800 ms após o ECI ser ativado."""
        if not self._eci_enabled or self._cli_hand_pwr_on is None or self._hand_powered:
            return
        if not self._cli_hand_pwr_on.service_is_ready():
            self._set_status(
                'ECI ativo — serviço de power indisponível '
                '(verifique o IP e o driver da mão).', WARN)
            return
        self._cli_hand_pwr_on.call_async(self._eci_srv.SetHandPowerOn.Request())
        self._hand_powered = True
        self._pwr_btn.set_state('⏻', 'PWR ON', OK, 'white')
        self._set_status('Canal ECI ativo — alimentação ligada (LED azul aceso).', OK)

    def _toggle_hand_power(self) -> None:
        """Liga/desliga a alimentação da mão COVVI via SetHandPowerOn/Off."""
        if not self._eci_enabled:
            self._set_status(
                'Ative o canal ECI antes de ligar a mão.', WARN)
            return
        if self._hand_powered:
            cli = self._cli_hand_pwr_off
            req = self._eci_srv.SetHandPowerOff.Request()
            target_on = False
        else:
            cli = self._cli_hand_pwr_on
            req = self._eci_srv.SetHandPowerOn.Request()
            target_on = True
        if cli is None or not cli.service_is_ready():
            self._set_status(
                'Serviço de power indisponível (espere a inicialização).',
                WARN)
            return
        cli.call_async(req)
        self._hand_powered = target_on
        if target_on:
            self._pwr_btn.set_state('⏻', 'PWR ON', OK, 'white')
            self._set_status('Power da mão LIGADO (LED azul aceso).', OK)
        else:
            self._pwr_btn.set_state('⏻', 'PWR OFF', BTN_NEUTRAL, TEXT)
            self._set_status('Power da mão DESLIGADO.', TEXT_DIM)

    # ──────────────────────────────────────────────────────────────────
    # ROBÔ CR10 — conexão TCP/IP
    # ──────────────────────────────────────────────────────────────────
    def _connect_real_robot(self) -> None:
        if not _REAL_DRIVER_OK:
            self._set_status(
                'Driver CR10 indisponível (módulo real_driver não carregou).',
                DANGER)
            return
        if self._robot_connected and self._real_driver is not None:
            self._disconnect_real_robot()
            return
        ip = (self._robot_ip_var.get() or '').strip()
        if not ip:
            self._set_status('Informe o IP do controlador CR10.', DANGER)
            return
        if self._robot_connecting:
            return
        # Conexão em background — evita congelar a GUI durante os ~5 s de
        # handshake TCP + sequência ClearError/EnableRobot/SpeedFactor.
        self._robot_connecting = True
        self._robot_connect_btn.set_state('⏳', 'Conectando…', BTN_NEUTRAL, TEXT)
        self._set_status(f'Abrindo sockets para CR10 em {ip}…', PRIMARY)
        threading.Thread(
            target=self._connect_robot_worker, args=(ip,), daemon=True).start()

    def _connect_robot_worker(self, ip: str) -> None:
        """Roda em thread daemon — conecta e habilita o CR10 sem bloquear a GUI."""
        log.info('[ROBOT] Iniciando conexão com CR10 em %s', ip)
        try:
            cfg = CR10RealDriverConfig(ip=ip)
            log.info('[ROBOT] Config: timeout=%.1fs, speed=%d%%, '
                     'payload=%.2fkg, collision=%d',
                     cfg.connect_timeout_s, cfg.speed_factor,
                     cfg.payload_kg, cfg.collision_level)
            drv = CR10RealDriver(ip=ip, dry_run=False, config=cfg)

            log.info('[ROBOT] Abrindo sockets TCP '
                     '(29999 dashboard / 30004 feedback)…')
            self.root.after(0, lambda: self._set_status(
                f'Conectando sockets TCP em {ip}:29999/30004…', PRIMARY))
            drv.connect()
            log.info('[ROBOT] Sockets abertos com sucesso')
            self.root.after(0, lambda: self._set_status(
                f'CR10 {ip}: sockets OK — enviando ClearError/EnableRobot…',
                PRIMARY))

            log.info('[ROBOT] Executando sequência de enable '
                     '(ClearError → EnableRobot → SpeedFactor → SetCollisionLevel → PayLoad)…')
            drv.enable()
            log.info('[ROBOT] Enable concluído')

            # Aguarda o firmware completar EnableRobot antes de ler o modo.
            log.info('[ROBOT] Aguardando firmware (1.5 s)…')
            time.sleep(1.5)

            mode_raw = drv.robot_mode() or ''
            log.info('[ROBOT] RobotMode() → %r', mode_raw)
            self.root.after(
                0, lambda d=drv, m=mode_raw: self._finish_robot_connect(ip, d, m))
        except CR10RealDriverError as exc:
            log.error('[ROBOT] Falha na conexão: %s', exc)
            self.root.after(0, lambda e=str(exc): self._fail_robot_connect(e))
        except Exception as exc:
            log.exception('[ROBOT] Erro inesperado durante conexão')
            self.root.after(
                0, lambda e=str(exc): self._fail_robot_connect(
                    f'Erro inesperado: {e}'))

    def _finish_robot_connect(self, ip: str, drv,
                               mode_raw: str) -> None:
        """Callback no thread Tkinter após conexão bem-sucedida."""
        log.warning('[DBG] _finish_robot_connect: ip=%s mode_raw=%r robot_mode=%r',
                    ip, mode_raw, self._robot_mode)
        self._robot_connecting = False
        self._robot_reconnecting = False
        self._real_driver = drv
        self._robot_connected = True
        log.warning('[DBG] _finish_robot_connect: _robot_connected=True drv=%s', drv)
        self._robot_connect_btn.set_state('⚡', 'Desconectar', OK, 'white')
        # Conexão deu certo — persistir o IP para reusar no próximo boot.
        self._save_robot_config()
        # Heartbeat só inicia após uma conexão saudável; se cair, tenta
        # reconectar com backoff automaticamente.
        self._start_robot_heartbeat()
        # Aplica SpeedFactor do slider GUI ao braço real imediatamente após a
        # conexão (enable() usa SPEED_FACTOR_DEFAULT=10%; aqui sincronizamos com o slider).
        try:
            sf = int(max(SPEED_FACTOR_MIN,
                         min(SPEED_FACTOR_MAX, self.speed_factor_var.get())))
            drv._send_dash(f'SpeedFactor({sf})')
            log.warning('[CONNECT] SpeedFactor(%d)%% aplicado ao CR10', sf)
        except Exception as exc:
            log.warning('[CONNECT] SpeedFactor falhou na conexão: %s', exc)
            sf = drv.cfg.speed_factor
        # Modo 5 = ENABLE (pronto); 9 = ERROR no Dobot CR.
        # Usa regex \{9\} para evitar falso-positivo em IPs ou timestamps que
        # contenham '9' (ex.: 192.168.1.9 → '9' in mode_raw = True erroneamente).
        mode_note = f'  [RobotMode: {mode_raw[:60].strip()}]' if mode_raw else ''
        color = DANGER if re.search(r'\{9\}', mode_raw) else OK
        self._set_status(
            f'CR10 conectado em {ip} '
            f'(SpeedFactor={sf}%){mode_note}.', color)
        # Force bridge desativado: sensor de força externo será usado no lugar.
        # _start_force_bridge() — manter _wrench_pub e GUI de força intactos.
        if self._robot_mode == 'MIRROR':
            self._set_status(
                f'CR10 conectado em {ip} — modo MIRROR ativo '
                f'(SpeedFactor={sf}%): mova os sliders ou inicie palpação.', OK)

    def _fail_robot_connect(self, error: str) -> None:
        """Callback no thread Tkinter após falha na conexão."""
        self._robot_connecting = False
        self._robot_connect_btn.set_state('⚡', 'Conectar', PRIMARY, 'white')
        self._set_status(f'Falha ao conectar CR10: {error}', DANGER)

    def _disconnect_real_robot(self) -> None:
        # Mirror timer e bridge precisam parar ANTES de fechar os
        # sockets — senão as threads ainda tentam I/O em socket morto.
        # Heartbeat e reconexão são desligados primeiro: caso contrário a
        # detecção de "perda" dispararia logo após o usuário clicar
        # Desconectar (false positive).
        self._stop_robot_heartbeat()
        self._robot_reconnecting = False
        with self._mirror_timer_lock:
            if self._mirror_timer is not None:
                self._mirror_timer.cancel()
                self._mirror_timer = None
            self._mirror_last_target = None
        self._stop_force_bridge()
        drv = self._real_driver
        if drv is None:
            self._robot_connected = False
            self._robot_connect_btn.set_state(
                '⚡', 'Conectar', PRIMARY, 'white')
            return
        try:
            drv.stop()
        except CR10RealDriverError as exc:
            self.get_logger().debug(f'drv.stop() falhou no disconnect: {exc}')
        try:
            drv.close()
        except OSError as exc:
            self.get_logger().debug(f'drv.close() falhou no disconnect: {exc}')
        self._real_driver = None
        self._robot_connected = False
        self._robot_connect_btn.set_state('⚡', 'Conectar', PRIMARY, 'white')
        self._set_status('CR10 desconectado.', TEXT_DIM)

    # ── Heartbeat + reconexão automática (braço CR10) ────────────────
    def _start_robot_heartbeat(self) -> None:
        """Inicia thread daemon que sonda `RobotMode()` a cada 2 s.
        Após 3 falhas consecutivas, dispara o worker de reconexão."""
        thr = self._robot_heartbeat_thread
        if thr is not None and thr.is_alive():
            return
        self._robot_heartbeat_stop.clear()
        self._robot_heartbeat_thread = threading.Thread(
            target=self._robot_heartbeat_loop, daemon=True)
        self._robot_heartbeat_thread.start()

    def _stop_robot_heartbeat(self) -> None:
        self._robot_heartbeat_stop.set()
        thr = self._robot_heartbeat_thread
        if thr is not None and thr is not threading.current_thread():
            thr.join(timeout=0.5)
        self._robot_heartbeat_thread = None

    def _robot_heartbeat_loop(self) -> None:
        HEARTBEAT_PERIOD_S = 1.0   # 1 s — permite detecção de drag em ~1 s
        MAX_FAILURES = 8           # 8 s antes de reconectar (era 3 × 5 s = 15 s)
        failures = 0
        _prev_mode: int | None = None
        while not self._robot_heartbeat_stop.is_set():
            if self._robot_heartbeat_stop.wait(HEARTBEAT_PERIOD_S):
                return
            if not self._robot_connected or self._real_driver is None:
                return
            ok = False
            mode_int: int | None = None
            try:
                drv = self._real_driver
                if drv is None:
                    return
                resp = drv.robot_mode()
                ok = bool(resp) and '{' in resp
                if ok:
                    import re as _re
                    m = _re.search(r'\{(\d+)\}', resp)
                    if m:
                        mode_int = int(m.group(1))
            except (CR10RealDriverError, OSError):
                ok = False
            if ok:
                failures = 0
                if mode_int is not None and mode_int != _prev_mode:
                    self.get_logger().info(
                        '[DRAG-WATCH] RobotMode mudou: %d → %d',
                        _prev_mode if _prev_mode is not None else -1, mode_int)
                    _prev_mode = mode_int
                # Auto-detecção de drag: modo ≠ 5 (idle) e ≠ 6 (running) indica
                # que o controlador mudou de estado — tipicamente modo 9 = DragTeach.
                # Ajustar ROBOT_MODE_DRAG em real_driver.py se necessário.
                from touch_pack.real_driver import ROBOT_MODE_DRAG
                with self._lock:
                    phase = self._latest_phase
                if phase in ('IDLE', 'DONE', 'ABORTED'):
                    is_drag = (mode_int == ROBOT_MODE_DRAG)
                    if is_drag and not self._drag_enabled:
                        self.get_logger().warning(
                            '[DRAG-WATCH] Drag físico detectado (modo=%d) — '
                            'tracking real→sim activado automaticamente.', mode_int)
                        self._drag_enabled = True
                        self.root.after(0, self._update_drag_btn_auto, True)
                    elif not is_drag and self._drag_enabled:
                        self.get_logger().info(
                            '[DRAG-WATCH] Drag desactivado (modo=%d) — '
                            'tracking real→sim desligado.', mode_int)
                        self._drag_enabled = False
                        self.root.after(0, self._update_drag_btn_auto, False)
            else:
                failures += 1
                self.get_logger().warn(
                    f'Heartbeat CR10 falhou ({failures}/{MAX_FAILURES}).')
                if failures >= MAX_FAILURES:
                    self.root.after(0, self._on_robot_connection_lost)
                    return

    def _update_drag_btn_auto(self, active: bool) -> None:
        """Actualiza o botão de drag a partir do watcher (thread Tk-safe)."""
        btn = self._drag_btn
        if btn is None:
            return
        if active:
            btn.config(text='🖐 Drag (auto)', bg=WARN, fg='white',
                       activebackground='#b45309')
            self._set_status(
                'Drag físico detectado — simulação a seguir o braço real.', WARN)
        else:
            btn.config(text='🖐 Drag OFF', bg=BTN_NEUTRAL, fg=TEXT,
                       activebackground=_shade(BTN_NEUTRAL, -0.08))
            self._set_status('Drag desactivado.', OK)

    def _on_robot_connection_lost(self) -> None:
        """Callback Tk — heartbeat detectou perda. Marca desconectado,
        derruba os recursos dependentes e dispara reconexão automática."""
        if self._robot_reconnecting or not self._robot_connected:
            return
        self._robot_reconnecting = True
        self._robot_connected = False
        self._robot_connect_btn.set_state(
            '⏳', 'Reconectando…', WARN, 'white')
        self._set_status(
            'Conexão CR10 perdida — tentando reconectar automaticamente…',
            WARN)
        # Para o bridge de força (vai tentar ler de socket morto).
        self._stop_force_bridge()
        drv = self._real_driver
        self._real_driver = None
        if drv is not None:
            try:
                drv.close()
            except OSError:
                pass
        # Heartbeat acabou de sair (return após dispatch). Não precisa
        # parar de novo — apenas dispara o worker.
        self._spawn_robot_reconnect()

    def _spawn_robot_reconnect(self) -> None:
        """Inicia worker que tenta reconectar com backoff exponencial."""
        thr = self._robot_reconnect_thread
        if thr is not None and thr.is_alive():
            return
        ip = (self._robot_ip_var.get()
              or self._robot_cfg.get('robot_ip', '192.168.5.2')).strip()
        self._robot_reconnect_thread = threading.Thread(
            target=self._robot_reconnect_worker, args=(ip,), daemon=True)
        self._robot_reconnect_thread.start()

    def _robot_reconnect_worker(self, ip: str) -> None:
        """Backoff exponencial 2→3→4.5→…→30 s. Para quando reconectar
        ou quando o usuário desconecta/fecha (cancela via flag)."""
        backoff = 2.0
        max_backoff = 30.0
        attempt = 0
        while (not self._stop_event.is_set()
               and self._robot_reconnecting):
            attempt += 1
            self.get_logger().info(
                f'[ROBOT] Reconexão tentativa {attempt} → {ip}')
            try:
                cfg = CR10RealDriverConfig(ip=ip)
                drv = CR10RealDriver(ip=ip, dry_run=False, config=cfg)
                drv.connect()
                drv.enable()
                time.sleep(1.5)
                mode_raw = drv.robot_mode() or ''
                self.root.after(
                    0, lambda d=drv, m=mode_raw: self._finish_robot_connect(
                        ip, d, m))
                return
            except CR10RealDriverError as exc:
                self.get_logger().warn(
                    f'Reconexão {attempt} falhou: {exc} '
                    f'(próxima em {backoff:.0f} s)')
                self.root.after(0, lambda a=attempt, b=backoff: self._set_status(
                    f'Reconectando CR10 — tentativa {a} falhou, '
                    f'próxima em {b:.0f} s.', WARN))
            if self._stop_event.wait(backoff):
                return
            backoff = min(max_backoff, backoff * 1.5)
        self.root.after(0, lambda: self._robot_connect_btn.set_state(
            '⚡', 'Conectar', PRIMARY, 'white'))

    def _set_robot_mode(self, selected: str) -> None:
        mode = (selected or '').strip().upper()
        if mode not in ('SIM_ONLY', 'MIRROR'):
            return
        if self._robot_connecting:
            # Conexão em andamento — recusa a troca para não corrermos
            # com o worker que ainda vai setar `_real_driver`.
            self._robot_mode_var.set(self._robot_mode)
            self._set_status(
                'Aguarde a conexão concluir antes de trocar de modo.', WARN)
            return
        # Palpação em curso — trocar de SIM_ONLY ↔ MIRROR no meio do
        # experimento poderia perder/comandar o braço real fora de hora.
        # Fases "estáveis" (em que a troca é segura): IDLE, DONE, ABORTED.
        if self._latest_phase not in ('IDLE', 'DONE', 'ABORTED'):
            self._robot_mode_var.set(self._robot_mode)
            self._set_status(
                f'Palpação em curso (fase {self._latest_phase}) — '
                'aguarde o término antes de trocar de modo.', WARN)
            return
        self._robot_mode = mode
        self._save_robot_config()
        if mode == 'MIRROR':
            self._set_status(
                'Modo MIRROR — mova os sliders para controlar o braço real.',
                WARN if not self._robot_connected else OK)
        else:
            with self._mirror_timer_lock:
                if self._mirror_timer is not None:
                    self._mirror_timer.cancel()
                    self._mirror_timer = None
                self._mirror_last_target = None
            self._set_status(
                'Modo SIM_ONLY — comandos só na simulação.', OK)

    # ──────────────────────────────────────────────────────────────────
    # E-STOP (combina parada do robô + abertura da mão)
    # ──────────────────────────────────────────────────────────────────
    def _estop(self) -> None:
        """Parada de emergência: StopRobot+DisableRobot no CR10 (se
        conectado) e abre a mão se o ECI estiver ativo.

        NÃO substitui o botão físico de E-STOP do controlador CR.
        """
        # 1. Aborta o tactile_explorer FSM (CONTACT/HOLD/SLIDING param).
        #    Sem isso o explorer continua publicando setpoints no JTC e o
        #    robô executa trajetórias acumuladas quando se recuperar do Stop.
        stop_msg = String()
        stop_msg.data = 'stop'
        self._stop_pub.publish(stop_msg)

        # 2. Congela o mirror poll loop (evita ServoJ após a parada).
        cur = self._latest_joint_rad
        if cur is not None:
            self._mirror_last_target = np.asarray(cur, dtype=np.float64)

        # 3. Para o braço real.
        if self._real_driver is not None and self._robot_connected:
            try:
                self._real_driver.stop()
            except CR10RealDriverError as exc:
                self.get_logger().error(f'E-STOP real falhou: {exc}')

        # 4. Abre a mão via ECI.
        if self._eci_enabled and self._cli_eci_grip is not None \
                and self._eci_srv is not None:
            try:
                grip = self._eci_msg.CurrentGripID()
                grip.value = 11   # 11 = GLOVE (mão totalmente aberta)
                req = self._eci_srv.SetCurrentGrip.Request()
                req.grip_id = grip
                self._cli_eci_grip.call_async(req)
            except Exception:
                pass
        self._set_status(
            'E-STOP — robô parado, mão aberta.', DANGER)

    # ──────────────────────────────────────────────────────────────────
    # ROS subscriptions (rodam no executor)
    # ──────────────────────────────────────────────────────────────────
    def _cb_wrench(self, msg: WrenchStamped):
        with self._lock:
            # Palpação HORIZONTAL: a força normal vem do eixo Z (vertical).
            self._latest_force_normal = abs(float(msg.wrench.force.z))
            self._latest_force_mag = math.sqrt(
                msg.wrench.force.x ** 2 +
                msg.wrench.force.y ** 2 +
                msg.wrench.force.z ** 2)
            self._fx = float(msg.wrench.force.x)
            self._fy = float(msg.wrench.force.y)
            self._fz = float(msg.wrench.force.z)
            self._last_wrench_ts = (
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def _cb_status(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            new_phase = str(data.get('phase', 'IDLE'))
            if new_phase != self._latest_phase:
                self._phase_t_start = time.time()
            self._latest_phase = new_phase
            # Atualiza durações esperadas só quando o explorer publicar
            # (mantém defaults razoáveis quando os campos faltam).
            try:
                self._latest_speed_mms = float(data.get(
                    'speed_mms', self._latest_speed_mms))
                self._latest_distance_mm = float(data.get(
                    'distance_mm', self._latest_distance_mm))
                self._latest_hold_seconds = float(data.get(
                    'hold_seconds', self._latest_hold_seconds))
            except (TypeError, ValueError):
                pass

    # ──────────────────────────────────────────────────────────────────
    # Refresh do painel direito (Tk thread, 10 Hz)
    # ──────────────────────────────────────────────────────────────────
    def _refresh_status_panel(self):
        # Lê force_var antes do _lock — é um acesso Tk e não deve estar sob lock
        # (TclError dentro do lock causaria deadlock no shutdown).
        try:
            tgt = float(self.force_var.get())
        except (ValueError, tk.TclError):
            tgt = FORCE_DEFAULT
        with self._lock:
            phase = self._latest_phase
            f_normal = self._latest_force_normal
            fx, fy, fz = self._fx, self._fy, self._fz
            last_ts = self._last_wrench_ts
            phase_t0 = self._phase_t_start
            speed_mms = self._latest_speed_mms
            distance_mm = self._latest_distance_mm
            hold_s = self._latest_hold_seconds

        has_data = last_ts > 0.0
        if not has_data:
            self.force_value_lbl.config(text='—   N', fg=TEXT_DIM)
            self.force_status_lbl.config(
                text='aguardando /ft_sensor/wrench', fg=TEXT_DIM)
            self.err_value_lbl.config(text='—  N', fg=TEXT_DIM)
        else:
            err = abs(f_normal - tgt)
            if f_normal < 0.05:
                # F≈0 é tratado como modo CALIBRAÇÃO (não como erro): o
                # explorer mantém o experimento rodando mesmo sem contato.
                color, status = WARN, ('calibração — sem contato; '
                                        'experimento continua')
            elif err <= 0.10:
                color, status = OK, 'estável dentro da tolerância (±0.10 N)'
            elif err <= 0.25:
                color, status = WARN, 'oscilando (±0.25 N)'
            else:
                color, status = DANGER, 'fora da tolerância'
            self.force_value_lbl.config(
                text=f'{f_normal:6.2f}  N', fg=color)
            self.force_status_lbl.config(text=status, fg=color)
            self.err_value_lbl.config(text=f'{err:5.2f}  N', fg=color)

        self.fx_lbl.config(text=f'{fx:+6.2f} N')
        self.fy_lbl.config(text=f'{fy:+6.2f} N')
        self.fz_lbl.config(text=f'{fz:+6.2f} N')

        phase_color = {
            'IDLE': TEXT_MUTED, 'CONTACT': PRIMARY, 'HOLD': WARN,
            'SLIDING': PRIMARY, 'RETRACT': TEXT_MUTED,
            'DONE': OK, 'ABORTED': DANGER,
        }.get(phase, TEXT)
        self.phase_lbl.config(text=phase, fg=phase_color)

        # Cronômetro só por label (sem Progressbar). SLIDING/HOLD têm
        # duração esperada; CONTACT/RETRACT mostram só tempo decorrido.
        elapsed = max(0.0, time.time() - phase_t0)
        expected: float | None = None
        if phase == 'SLIDING' and speed_mms > 0.01:
            expected = distance_mm / speed_mms
        elif phase == 'HOLD' and hold_s > 0.01:
            expected = hold_s
        if phase in ('IDLE', 'DONE', 'ABORTED'):
            self.timer_lbl.config(text='—', fg=TEXT_DIM)
        elif expected is not None:
            pct = min(100.0, 100.0 * elapsed / expected)
            self.timer_lbl.config(
                text=f'{elapsed:4.1f}s / {expected:4.1f}s ({pct:4.0f}%)',
                fg=phase_color)
        else:
            self.timer_lbl.config(text=f'{elapsed:4.1f}s', fg=phase_color)

        self.root.after(100, self._refresh_status_panel)

    # ──────────────────────────────────────────────────────────────────
    # Disparo da palpação
    # ──────────────────────────────────────────────────────────────────
    def _on_stop_palpation(self) -> None:
        """Interrompe o experimento em curso: publica /palpation/stop e
        pausa o braço real imediatamente via Halt()."""
        msg = String()
        msg.data = 'stop'
        self._stop_pub.publish(msg)
        # Halt paralisa o movimento atual do braço real sem desabilitar.
        if self._robot_connected and self._real_driver is not None:
            try:
                self._real_driver.halt()
            except CR10RealDriverError as exc:
                self.get_logger().warning(f'Halt após stop falhou: {exc}')
        # Congela o poll loop: define last_target = posição atual para que
        # o dedup bloqueie novos ServoJ até o braço realmente se mover de novo.
        cur = self._latest_joint_rad
        if cur is not None:
            self._mirror_last_target = np.asarray(cur, dtype=np.float64)
        self._set_status('Palpação interrompida pelo operador.', WARN)

    def _on_start(self):
        # Satura cada parâmetro ao seu intervalo válido antes de enviar,
        # tanto para a publicação quanto para o que o usuário vê nos
        # spinboxes/sliders.
        self._suppressing = True
        try:
            speed = self._clamp_var(self.speed_var, SPEED_MIN, SPEED_MAX)
            force = self._clamp_var(self.force_var, FORCE_MIN, FORCE_MAX)
            dist = self._clamp_var(self.dist_var, DIST_MIN, DIST_MAX)
            tgt = self._clamp_var(self.target_dist_var,
                                    TGT_DIST_CM_MIN, TGT_DIST_CM_MAX)
            approach = self._clamp_var(self.approach_var,
                                        APPROACH_MIN, APPROACH_MAX,
                                        default=APPROACH_DEFAULT)
        finally:
            self._suppressing = False
        if None in (speed, force, dist, tgt, approach):
            self._set_status('Parâmetros inválidos.', DANGER)
            return
        kp = self._clamp_var(self.pid_kp_var, KP_MIN, KP_MAX,
                              default=KP_DEFAULT)
        ki = self._clamp_var(self.pid_ki_var, KI_MIN, KI_MAX,
                              default=KI_DEFAULT)
        kd = self._clamp_var(self.pid_kd_var, KD_MIN, KD_MAX,
                              default=KD_DEFAULT)
        sf_pct = self._clamp_var(self.speed_factor_var,
                                  SPEED_FACTOR_MIN, SPEED_FACTOR_MAX,
                                  default=SPEED_FACTOR_DEFAULT)
        payload = {
            'speed_mms':          float(speed),
            'force_n':            float(force),
            'distance_mm':        float(dist),
            'target_distance_cm': float(tgt),
            # Velocidade da descida CONTACT / subida RETRACT (mm/s).
            # Explorer aplica como approach_v_max_mms; min = 20 % do max.
            'approach_speed_mms': float(approach),
            # Direção XY (mundo) do sliding — string '+X' / '-X' / '+Y' / '-Y'.
            'slide_dir':          self.slide_dir_var.get(),
            # Ganhos do PID de força aplicados no HOLD pelo explorer.
            'kp': float(kp if kp is not None else KP_DEFAULT),
            'ki': float(ki if ki is not None else KI_DEFAULT),
            'kd': float(kd if kd is not None else KD_DEFAULT),
            # Speed factor (1-100 %): limita a velocidade máxima de junta
            # enviada como pt.velocities no JointTrajectoryPoint. O JTC usa
            # esses hints para splines cúbicos contínuos em velocidade,
            # eliminando descontinuidades que causam estalos no hardware.
            'speed_factor_pct':   float(sf_pct if sf_pct is not None
                                         else SPEED_FACTOR_DEFAULT),
            # Home customizada: explorer leva o braço PARA CÁ antes
            # de descer. Em graus URDF, mesma chave/ordem do
            # `_apply_arm_home`.
            'home_deg': {j: float(self._arm_home_deg[j])
                          for j in ARM_JOINTS},
        }
        # Garante SpeedFactor=10% no braço real durante a palpação.
        # Velocidades altas são perigosas nesse protocolo — impõe aqui
        # independente do slider de "Velocidade bruta" da aba manual.
        if self._robot_connected and self._real_driver is not None:
            try:
                self._real_driver._send_dash('SpeedFactor(10)')
                self.get_logger().info('[PALP] SpeedFactor(10) aplicado para palpação')
                # Sincroniza o slider para que a GUI reflita o valor real.
                self._suppressing = True
                try:
                    self.speed_factor_var.set(10)
                finally:
                    self._suppressing = False
            except CR10RealDriverError as exc:
                self.get_logger().warning(f'SpeedFactor(10) falhou: {exc}')

        # Limpa o log de movimentos da sessão anterior para que o dedup do
        # mirror poll não bloqueie os primeiros ServoJ desta sessão.
        with self._mirror_timer_lock:
            self._mirror_last_target = None

        msg = String()
        msg.data = json.dumps(payload)
        self._start_pub.publish(msg)
        # Quando a mão real está conectada via ECI, aciona o grip FINGER
        # (Index estendido) automaticamente, já que o tactile_explorer
        # publica a pose da mão apenas no tópico do sim (ros2_control).
        self._send_eci_grip(7, 'Finger — palpação (Index estendido)')
        # Envia posição explícita com velocidade controlada pelo slider
        # (SetCurrentGrip usa velocidade interna do firmware; SetDigitPosn permite controle).
        if self._eci_enabled:
            self._schedule_eci_posn(HAND_POINT_DEG)
        self._set_status(
            f'/palpation/start — v={payload["speed_mms"]:.1f} mm/s, '
            f'F={payload["force_n"]:.2f} N, '
            f'slide={payload["distance_mm"]:.0f} mm {payload["slide_dir"]}, '
            f'descida={payload["target_distance_cm"]:.1f} cm '
            f'@ {payload["approach_speed_mms"]:.0f} mm/s | '
            f'vel junta {payload["speed_factor_pct"]:.0f}% | '
            f'PID Kp={payload["kp"]:.4g} Ki={payload["ki"]:.4g} '
            f'Kd={payload["kd"]:.4g}.',
            OK)

    def _set_status(self, text: str, color: str = TEXT_MUTED):
        self.status_var.set(text)
        try:
            self._status_lbl.config(fg=color)
        except AttributeError:
            pass  # statusbar ainda não foi construída

    # ──────────────────────────────────────────────────────────────────
    # Loop ROS em thread separada
    # ──────────────────────────────────────────────────────────────────
    def _spin_ros(self):
        while not self._stop_event.is_set() and rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.05)
            except Exception as exc:
                log.error('[SPIN] spin_once falhou: %s', exc)
                if not rclpy.ok():
                    break
                # Continua girando — uma exceção isolada não deve parar o executor.

    def _on_close(self):
        self._stop_event.set()
        # Parar heartbeat e cancelar reconexão antes de fechar sockets.
        self._stop_robot_heartbeat()
        self._robot_reconnecting = False
        # Idem para o watchdog da mão — se não desligar, o re-spawn vai
        # ser tentado durante o shutdown.
        self._hand_should_be_alive = False
        self._stop_hand_watchdog()
        # Encerra o force_receiver_node se estiver rodando.
        self._force_rx_should_be_alive = False
        rx_proc = self._force_rx_proc
        self._force_rx_proc = None
        if rx_proc is not None and rx_proc.poll() is None:
            try:
                os.killpg(os.getpgid(rx_proc.pid), signal.SIGTERM)
                rx_proc.wait(timeout=2.0)
            except Exception:
                pass
        self._stop_force_bridge()
        # Cancela callbacks Tk pendentes — disparar após `root.destroy()`
        # gera TclError ou crash.
        if self._eci_posn_after is not None:
            try:
                self.root.after_cancel(self._eci_posn_after)
            except Exception:
                pass
            self._eci_posn_after = None
        with self._mirror_timer_lock:
            if self._mirror_timer is not None:
                self._mirror_timer.cancel()
                self._mirror_timer = None
        # Apaga o LED da mão antes de matar o subprocesso (mesmo caminho
        # de _disconnect_real_hand). Sem isso o driver TCP cai com a mão
        # ainda energizada e o LED azul permanece aceso.
        if self._eci_enabled and self._hand_powered:
            self._send_hand_poweroff_blocking(timeout_s=3.0)
            self._hand_powered = False
        self._terminate_hand_subprocess()
        # Fecha sockets do CR10.
        if self._real_driver is not None:
            try:
                self._real_driver.stop()
            except CR10RealDriverError:
                pass
            try:
                self._real_driver.close()
            except OSError:
                pass
        try:
            self.root.destroy()
        except Exception:
            pass


def main(args=None):
    import faulthandler, sys
    faulthandler.enable(file=sys.stderr)
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s.%(msecs)03d [%(name)s] %(levelname)s  %(message)s',
        datefmt='%H:%M:%S')
    rclpy.init(args=args)
    gui = PalpationGUI()

    def _sighandler(sig, frame):
        # Fechar a janela Tkinter de forma limpa (roda _on_close via protocol).
        try:
            gui.root.after(0, gui._on_close)
        except Exception:
            pass

    signal.signal(signal.SIGTERM, _sighandler)
    signal.signal(signal.SIGINT, _sighandler)

    try:
        gui.root.mainloop()
    finally:
        gui._stop_event.set()
        try:
            gui.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
