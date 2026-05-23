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

import json
import logging
import math
import os
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

from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped
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

# Tempo (s) que o controller deve levar para alcançar o setpoint a partir
# da posição atual — controla a "rampa" das trajetórias publicadas pela
# aba Controle Manual (Home, sliders, presets da mão).
MOVE_TIME_MIN, MOVE_TIME_MAX, MOVE_TIME_DEFAULT = 0.1, 30.0, 1.0  # s
SPEED_FACTOR_MIN, SPEED_FACTOR_MAX, SPEED_FACTOR_DEFAULT = 1, 100, 50  # %

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
        # Subscrição na trajetória comandada (não na pose medida do sim):
        # captura sliders manuais e palpação autônoma com a mesma latência,
        # sem competir com /joint_states (que lagga atrás do comando).
        self.create_subscription(
            JointTrajectory,
            '/cr10_group_controller/joint_trajectory',
            self._cb_arm_trajectory, 10)

        # ─── Home pose customizável ──────────────────────────────────
        # Default (ARM_HOME_DEG) é sobrescrito se ~/.config/touch_pack/
        # home_pose.json existir. Atualizado pelo botão "💾 Salvar Home".
        self._arm_home_deg: dict[str, float] = dict(ARM_HOME_DEG)
        self._load_home_pose()

        # IPs e modo persistidos — carregar antes da UI para os defaults
        # dos campos refletirem o último valor usado.
        self._robot_cfg: dict[str, str] = dict(ROBOT_CONFIG_DEFAULTS)
        self._load_robot_config()

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

        tab_palp = tk.Frame(nb, bg=BG)
        tab_man = tk.Frame(nb, bg=BG)
        nb.add(tab_palp, text='Palpação')
        nb.add(tab_man,  text='Controle Manual')

        # IMPORTANTE: as abas NÃO são envolvidas em `_scrollable` (Canvas
        # + create_window). Em testes locais essa combinação provocava
        # segfault deterministico no Tk durante a criação do primeiro
        # widget da segunda aba — provavelmente bug da interação entre
        # Canvas embed + ttk.Progressbar + muitos filhos. A
        # responsividade é dada por `minsize=720×460` + cards com
        # `expand=True`. Se a janela ficar menor que o conteúdo, o que
        # passar do limite é cortado, mas a GUI não trava.
        self._build_palpation_tab(tab_palp)
        self._build_manual_tab(tab_man)

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
            delta = -1 if e.num == 5 or e.delta < 0 else 1
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

        # ── Top: controle de velocidade (Duração s + SpeedFactor %) ─────
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

        # Linha 1 — Duracao (Gazebo trajectory controller)
        self._use_duration_var = tk.IntVar(value=1)
        dur_chk_row = tk.Frame(speed_inner, bg=PANEL)
        dur_chk_row.pack(fill='x')
        tk.Checkbutton(dur_chk_row,
                       text='Duracao (s) — trajetoria Gazebo',
                       variable=self._use_duration_var,
                       bg=PANEL, fg=TEXT, font=FONT_LBL,
                       anchor='w'
                       ).pack(side='left')
        self.move_time_var = tk.DoubleVar(value=MOVE_TIME_DEFAULT)
        self._param_row(speed_inner, label='Duracao', unit='s',
                        var=self.move_time_var,
                        vmin=MOVE_TIME_MIN, vmax=MOVE_TIME_MAX, step=0.5)

        # Linha 2 — SpeedFactor % (braco real via TCP)
        self._use_speedfactor_var = tk.IntVar(value=0)
        sf_chk_row = tk.Frame(speed_inner, bg=PANEL)
        sf_chk_row.pack(fill='x', pady=(8, 0))
        tk.Checkbutton(sf_chk_row,
                       text='Velocidade bruta (%) — braco real',
                       variable=self._use_speedfactor_var,
                       bg=PANEL, fg=TEXT, font=FONT_LBL,
                       anchor='w',
                       command=self._apply_speed_factor_if_active
                       ).pack(side='left')
        self.speed_factor_var = tk.DoubleVar(value=SPEED_FACTOR_DEFAULT)
        self._param_row(speed_inner, label='SpeedFactor', unit='%',
                        var=self.speed_factor_var,
                        vmin=SPEED_FACTOR_MIN, vmax=SPEED_FACTOR_MAX, step=1)
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
        """Retorna duração em s para a trajetória do Gazebo.

        Se 'Duração' estiver ativa, usa o slider de segundos.
        Se desativada (só SpeedFactor), usa 0.5 s como mínimo seguro."""
        if self._use_duration_var.get():
            dur = self._clamp_var(self.move_time_var,
                                   MOVE_TIME_MIN, MOVE_TIME_MAX,
                                   default=MOVE_TIME_DEFAULT)
            return float(dur if dur is not None else MOVE_TIME_DEFAULT)
        return 0.5

    def _apply_speed_factor_if_active(self) -> None:
        """Envia SpeedFactor(%) ao braço real se o checkbox estiver ativo."""
        if not self._use_speedfactor_var.get():
            return
        if not self._robot_connected or self._real_driver is None:
            return
        try:
            v = int(max(SPEED_FACTOR_MIN,
                        min(SPEED_FACTOR_MAX, self.speed_factor_var.get())))
        except (ValueError, tk.TclError):
            return
        try:
            with self._real_lock:
                self._real_driver._send_dash(f'SpeedFactor({v})')
            self.get_logger().debug('SpeedFactor(%d) aplicado', v)
        except CR10RealDriverError as exc:
            self.get_logger().warning('SpeedFactor falhou: %s', exc)

    @staticmethod
    def _duration_msg(seconds: float) -> Duration:
        sec = int(seconds)
        nsec = int((seconds - sec) * 1e9)
        return Duration(sec=sec, nanosec=nsec)

    # ── Publicação direta nos controllers ─────────────────────────────
    def _publish_arm_from_sliders(self):
        if self._suppressing:
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

        Dedup por tolerância (0.5°): trajetórias repetidas com o mesmo
        alvo não disparam MovJ redundante.
        """
        q_new = np.asarray(positions_rad, dtype=np.float64)
        with self._mirror_timer_lock:
            last = self._mirror_last_target
            if last is not None and np.max(np.abs(q_new - last)) < 0.0087:
                return  # já enviado/agendado mesmo alvo — pular
            if self._mirror_timer is not None:
                self._mirror_timer.cancel()
            self._mirror_timer = threading.Timer(
                0.08, self._mirror_movj_send, args=[q_new.tolist()])
            self._mirror_timer.daemon = True
            self._mirror_timer.start()

    def _mirror_movj_send(self, positions_rad: list[float]) -> None:
        """Converte URDF→DOBOT e envia MovJ(joint={...}) ao braço real.

        Toda checagem de estado do driver é feita DENTRO do `_real_lock`
        para evitar TOCTOU com `_disconnect_real_robot` em outra thread.
        """
        try:
            q_dobot_rad = _urdf_to_dobot(
                np.array(positions_rad, dtype=np.float64))
            q_dobot_deg = [math.degrees(float(v)) for v in q_dobot_rad]
            with self._real_lock:
                drv = self._real_driver
                if (drv is None or not self._robot_connected
                        or self._robot_mode != 'MIRROR'):
                    return
                drv.mov_j_joint_deg(q_dobot_deg)
            self._mirror_last_target = np.asarray(
                positions_rad, dtype=np.float64)
            self.get_logger().debug('Mirror MovJ → %s',
                                    [f'{v:.2f}' for v in q_dobot_deg])
        except CR10RealDriverError as exc:
            self.get_logger().warning('Mirror MovJ falhou: %s', exc)

    # ── Subscrição no tópico de trajetória comandada ─────────────────
    def _cb_arm_trajectory(self, msg: JointTrajectory) -> None:
        """Captura toda trajetória publicada em /cr10_group_controller/joint_trajectory
        (sliders manuais + palpação autônoma) e encaminha o ÚLTIMO ponto
        como MovJ para o braço real, com debounce de 80 ms."""
        if (self._robot_mode != 'MIRROR' or not self._robot_connected
                or self._real_driver is None or _urdf_to_dobot is None):
            return
        if not msg.points:
            return
        idx = {n: i for i, n in enumerate(msg.joint_names)}
        try:
            positions_rad = [float(msg.points[-1].positions[idx[j]])
                              for j in ARM_JOINTS]
        except (KeyError, IndexError):
            return
        self._mirror_movj_debounced(positions_rad)

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
                with self._real_lock:
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
        req.speed.value = 50
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
                                self._arm_home_deg[j] = float(data[j])
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

    def _build_statusbar(self):
        self.status_var = tk.StringVar(value='pronto.')
        bar = tk.Frame(self.root, bg=PANEL, height=28)
        bar.pack(side='bottom', fill='x')
        tk.Label(bar, textvariable=self.status_var, bg=PANEL, fg=TEXT_MUTED,
                 anchor='w', font=FONT_LBL).pack(side='left', padx=18)

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
        try:
            self._hand_proc = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
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
        """Desconexão limpa: PowerOff síncrono → SIGTERM → wait → SIGKILL.

        A chamada `call_async` retorna antes do request realmente chegar ao
        hardware; se matarmos o subprocesso imediatamente, o ECI fecha o
        socket e a mão fica energizada (LED azul aceso). Esperamos o future
        completar (com timeout) antes do SIGTERM, e ainda damos `proc.wait`
        para o driver fechar o socket TCP da mão de forma ordeira.
        """
        # 0) Intenção do usuário é encerrar — desliga watchdog para não
        #    tentar re-spawn após o SIGTERM.
        self._hand_should_be_alive = False
        self._stop_hand_watchdog()
        # 1) PowerOff síncrono com timeout (LED azul vai apagar antes da kill).
        if self._eci_enabled:
            self._send_hand_poweroff_blocking(timeout_s=1.0)
            self._hand_powered = False
            self._pwr_btn.set_state('⏻', 'PWR OFF', BTN_NEUTRAL, TEXT)
            self._eci_enabled = False
            self._eci_btn.set_state('◉', 'ECI OFF', BTN_NEUTRAL, TEXT)

        # 2) SIGTERM + wait, com fallback SIGKILL se o driver não sair.
        self._terminate_hand_subprocess()
        self._hand_connect_btn.set_state('⚡', 'Conectar', PRIMARY, 'white')
        self._set_status(
            'Driver da mão desconectado — LED apagado.', TEXT_DIM)

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
        self._set_status(
            'Driver da mão caiu — re-spawn automático em curso…', WARN)
        # `_connect_real_hand` re-spawna o subprocesso e re-ativa ECI+power
        # via `_post_connect_real_hand`. Como `_hand_proc=None` aqui, o
        # caminho de desconexão (toggle) é evitado.
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
        """Envia SIGTERM ao grupo de processos do driver e espera até 2 s;
        se ainda estiver vivo, força SIGKILL. Idempotente."""
        proc = self._hand_proc
        self._hand_proc = None
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except (OSError, ProcessLookupError) as exc:
            self.get_logger().debug(f'SIGTERM da mão ignorado ({exc}).')
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(
                'Driver da mão não saiu em 2 s — forçando SIGKILL.')
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
                     '(29999 dashboard / 30003 motion / 30004 feedback)…')
            self.root.after(0, lambda: self._set_status(
                f'Conectando sockets TCP em {ip}:29999/30003/30004…', PRIMARY))
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
        self._robot_connecting = False
        self._robot_reconnecting = False
        self._real_driver = drv
        self._robot_connected = True
        self._robot_connect_btn.set_state('⚡', 'Desconectar', OK, 'white')
        # Conexão deu certo — persistir o IP para reusar no próximo boot.
        self._save_robot_config()
        # Heartbeat só inicia após uma conexão saudável; se cair, tenta
        # reconectar com backoff automaticamente.
        self._start_robot_heartbeat()
        # Modo 5 = ENABLE (pronto); 9 = ERROR no Dobot CR.
        mode_note = f'  [RobotMode: {mode_raw[:60].strip()}]' if mode_raw else ''
        color = DANGER if '9' in mode_raw and '5' not in mode_raw else OK
        self._set_status(
            f'CR10 conectado em {ip} '
            f'(SpeedFactor={drv.cfg.speed_factor}%){mode_note}.', color)
        self._start_force_bridge()
        if self._robot_mode == 'MIRROR':
            self._set_status(
                f'CR10 conectado em {ip} — modo MIRROR ativo: '
                'mova os sliders para controlar o braço real.', OK)

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
        HEARTBEAT_PERIOD_S = 2.0
        MAX_FAILURES = 3
        failures = 0
        while not self._robot_heartbeat_stop.is_set():
            if self._robot_heartbeat_stop.wait(HEARTBEAT_PERIOD_S):
                return
            if not self._robot_connected or self._real_driver is None:
                return
            ok = False
            try:
                with self._real_lock:
                    drv = self._real_driver
                    if drv is None:
                        return
                    resp = drv.robot_mode()
                ok = bool(resp) and '{' in resp
            except (CR10RealDriverError, OSError):
                ok = False
            if ok:
                failures = 0
            else:
                failures += 1
                self.get_logger().warn(
                    f'Heartbeat CR10 falhou ({failures}/{MAX_FAILURES}).')
                if failures >= MAX_FAILURES:
                    self.root.after(0, self._on_robot_connection_lost)
                    return

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
        if self._real_driver is not None and self._robot_connected:
            try:
                self._real_driver.stop()
            except CR10RealDriverError as exc:
                self.get_logger().error(f'E-STOP real falhou: {exc}')
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
        with self._lock:
            phase = self._latest_phase
            f_normal = self._latest_force_normal
            tgt = float(self.force_var.get())
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
            # Home customizada: explorer leva o braço PARA CÁ antes
            # de descer. Em graus URDF, mesma chave/ordem do
            # `_apply_arm_home`.
            'home_deg': {j: float(self._arm_home_deg[j])
                          for j in ARM_JOINTS},
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._start_pub.publish(msg)
        # Quando a mão real está conectada via ECI, aciona o grip FINGER
        # (Index estendido) automaticamente, já que o tactile_explorer
        # publica a pose da mão apenas no tópico do sim (ros2_control).
        self._send_eci_grip(7, 'Finger — palpação (Index estendido)')
        self._set_status(
            f'/palpation/start — v={payload["speed_mms"]:.1f} mm/s, '
            f'F={payload["force_n"]:.2f} N, '
            f'slide={payload["distance_mm"]:.0f} mm {payload["slide_dir"]}, '
            f'descida={payload["target_distance_cm"]:.1f} cm '
            f'@ {payload["approach_speed_mms"]:.0f} mm/s | '
            f'PID Kp={payload["kp"]:.4g} Ki={payload["ki"]:.4g} '
            f'Kd={payload["kd"]:.4g}.',
            OK)

    def _set_status(self, text: str, color: str = TEXT_MUTED):
        self.status_var.set(text)

    # ──────────────────────────────────────────────────────────────────
    # Loop ROS em thread separada
    # ──────────────────────────────────────────────────────────────────
    def _spin_ros(self):
        while not self._stop_event.is_set() and rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.05)
            except Exception:
                break

    def _on_close(self):
        self._stop_event.set()
        # Parar heartbeat e cancelar reconexão antes de fechar sockets.
        self._stop_robot_heartbeat()
        self._robot_reconnecting = False
        # Idem para o watchdog da mão — se não desligar, o re-spawn vai
        # ser tentado durante o shutdown.
        self._hand_should_be_alive = False
        self._stop_hand_watchdog()
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
            self._send_hand_poweroff_blocking(timeout_s=1.0)
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
