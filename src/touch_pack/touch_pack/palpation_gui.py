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

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Driver TCP/IP do CR10 real (cabeada via 192.168.5.1 / LAN1).
try:
    from .real_driver import (
        CR10RealDriver, CR10RealDriverConfig, CR10RealDriverError,
    )
    from .kinematics import urdf_to_dobot as _urdf_to_dobot
    _REAL_DRIVER_OK = True
except Exception:  # pragma: no cover
    CR10RealDriver = None
    CR10RealDriverConfig = None
    CR10RealDriverError = Exception
    _urdf_to_dobot = None
    _REAL_DRIVER_OK = False


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

# Tempo (s) que o controller deve levar para alcançar o setpoint a partir
# da posição atual — controla a "rampa" das trajetórias publicadas pela
# aba Controle Manual (Home, sliders, presets da mão).
MOVE_TIME_MIN, MOVE_TIME_MAX, MOVE_TIME_DEFAULT = 0.1, 30.0, 1.0  # s

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

# Mimic joints da COVVI: 26 juntas escravas com multiplicadores extraídos
# de linear_covvi_hand_gazebo.urdf (offsets aproximadamente nulos).
# Formato: (nome_da_junta_mimic, junta_primária, multiplicador)
MIMIC_LIST = [
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
        self._start_pub = self.create_publisher(String, '/palpation/start', 5)
        self.create_subscription(
            String, '/palpation/status', self._cb_status, 10)
        self.create_subscription(
            WrenchStamped, '/ft_sensor/wrench', self._cb_wrench, 30)

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

        # ─── Mão COVVI (lazy) ────────────────────────────────────────
        self._hand_proc: subprocess.Popen | None = None
        self._eci_enabled = False
        self._eci_prefix = self.declare_parameter(
            'eci_prefix', '/covvi/hand').value
        self._eci_srv = None
        self._eci_msg = None
        self._cli_eci_grip = None
        self._cli_hand_pwr_on = None
        self._cli_hand_pwr_off = None
        self._hand_powered = False

        # ─── CR10 real (lazy) ────────────────────────────────────────
        self._real_driver = None    # CR10RealDriver | None
        self._real_lock = threading.Lock()
        self._robot_mode: str = 'SIM_ONLY'
        self._robot_connected: bool = False

        # Pump ServoJ — quando o modo é MIRROR e o socket está aberto,
        # uma thread daemon transmite o último /joint_states a 33 Hz para
        # o controlador real. Funciona para sliders manuais E para as
        # trajetórias do tactile_explorer (ambos atualizam o sim, que
        # publica /joint_states, que alimenta o pump).
        self._latest_q_urdf: np.ndarray | None = None
        self._real_pump_thread: threading.Thread | None = None
        self._real_pump_stop = threading.Event()
        self.create_subscription(
            JointState, '/joint_states', self._cb_joint_states, 50)

        # ─── Home pose customizável ──────────────────────────────────
        # Default (ARM_HOME_DEG) é sobrescrito se ~/.config/touch_pack/
        # home_pose.json existir. Atualizado pelo botão "💾 Salvar Home".
        self._arm_home_deg: dict[str, float] = dict(ARM_HOME_DEG)
        self._load_home_pose()

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
        self.root.minsize(1080, 620)

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
        self._hand_ip_var = tk.StringVar(value='192.168.1.123')
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
        self._robot_ip_var = tk.StringVar(value='192.168.5.1')
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
        self._robot_mode_var = tk.StringVar(value='SIM_ONLY')
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

        self._build_palpation_tab(tab_palp)
        self._build_manual_tab(tab_man)

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

        # Seletor de direção do deslizamento (XY mundo). joint1 só pode
        # produzir um arco; aqui o usuário escolhe a ORIENTAÇÃO desejada
        # e o explorer pega o sinal de Δθ que melhor se alinha a ela.
        self._build_slide_dir_selector(params_card)

        btn_wrap = tk.Frame(col_left, bg=BG)
        btn_wrap.pack(fill='x', pady=(14, 0))
        self.start_btn = tk.Button(
            btn_wrap, text='▶  Iniciar Palpação',
            command=self._on_start, bg=PRIMARY, fg='white',
            activebackground=PRIMARY_HV, activeforeground='white',
            font=FONT_HEAD, relief='flat', bd=0, padx=18, pady=12,
            cursor='hand2')
        self.start_btn.pack(fill='x')

        # ── Coluna direita: feedback FT ───────────────────────────────
        fb_card = self._card(col_right, 'Feedback do Sensor FT')

        fnrow = tk.Frame(fb_card, bg=PANEL)
        fnrow.pack(fill='x', pady=(6, 4))
        tk.Label(fnrow, text='Força Normal Executada', font=FONT_LBL,
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

        # ── Top: tempo de movimento (comum a braço e mão) ────────────
        time_card = tk.Frame(body, bg=PANEL,
                              highlightthickness=1,
                              highlightbackground=BORDER,
                              highlightcolor=BORDER)
        time_card.pack(fill='x', pady=(0, 10))
        tk.Label(time_card, text='Tempo de Movimento',
                 bg=PANEL, fg=TEXT, font=FONT_HEAD,
                 anchor='w').pack(fill='x', padx=14, pady=(10, 6))
        tk.Frame(time_card, bg=BORDER, height=1).pack(fill='x')
        time_inner = tk.Frame(time_card, bg=PANEL)
        time_inner.pack(fill='x', padx=14, pady=10)
        self.move_time_var = tk.DoubleVar(value=MOVE_TIME_DEFAULT)
        self._param_row(time_inner, label='Duração do movimento',
                         unit='s', var=self.move_time_var,
                         vmin=MOVE_TIME_MIN, vmax=MOVE_TIME_MAX, step=0.1,
                         hint='Tempo total para a trajetória articular '
                              '(Home / sliders / presets da mão).')

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
                   command=lambda: self._apply_hand_preset(HAND_OPEN_DEG),
                   bg=BTN_NEUTRAL, fg=TEXT,
                   activebackground=_shade(BTN_NEUTRAL, -0.08),
                   activeforeground=TEXT,
                   font=FONT_LBL, relief='flat', bd=0, padx=12, pady=8,
                   cursor='hand2'
                   ).pack(side='left', fill='x', expand=True, padx=(0, 3))
        tk.Button(btns_hand, text='👉  Apontar',
                   command=lambda: self._apply_hand_preset(HAND_POINT_DEG),
                   bg=OK, fg='white',
                   activebackground=_shade(OK, -0.08),
                   activeforeground='white',
                   font=FONT_LBL, relief='flat', bd=0, padx=12, pady=8,
                   cursor='hand2'
                   ).pack(side='left', fill='x', expand=True, padx=3)
        tk.Button(btns_hand, text='✊  Fechar',
                   command=lambda: self._apply_hand_preset(HAND_CLOSE_DEG),
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
        """Lê e satura o slider de Tempo de Movimento (em segundos)."""
        dur = self._clamp_var(self.move_time_var,
                                MOVE_TIME_MIN, MOVE_TIME_MAX,
                                default=MOVE_TIME_DEFAULT)
        return float(dur if dur is not None else MOVE_TIME_DEFAULT)

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
        msg.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions_rad]
        pt.time_from_start = self._duration_msg(duration_s)
        msg.points.append(pt)
        self._arm_pub.publish(msg)
        # Em modo MIRROR o pump (loop ServoJ @33 Hz) já bombeia o
        # /joint_states do sim para o real — não é preciso despachar
        # nada extra aqui.

    # ── /joint_states → pump ServoJ ───────────────────────────────────
    def _cb_joint_states(self, msg: JointState):
        """Cacheia o último q (URDF) do braço para o pump consumir."""
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in ARM_JOINTS):
            return
        try:
            q = np.array([float(msg.position[idx[j]]) for j in ARM_JOINTS],
                          dtype=np.float64)
        except (IndexError, ValueError):
            return
        # Atribuição atômica (GIL) — pump lê esta referência.
        self._latest_q_urdf = q

    def _real_pump_active(self) -> bool:
        return (self._real_pump_thread is not None
                and self._real_pump_thread.is_alive())

    def _start_real_pump(self) -> None:
        """Inicia o pump ServoJ se as pré-condições estiverem OK.

        Faz PTP de alinhamento (`mov_j_joint_deg`) antes do streaming
        para que a primeira leitura do ServoJ não exija um salto grande
        do robô real (evita disparo de proteção / colisão simulada)."""
        if self._real_pump_active():
            return
        if not self._robot_connected or self._real_driver is None:
            return
        if self._latest_q_urdf is None or _urdf_to_dobot is None:
            self._set_status(
                'Aguardando /joint_states antes de iniciar o mirror.',
                WARN)
            return
        self._set_status('Alinhando robô real à pose do sim (PTP)…', WARN)
        threading.Thread(target=self._align_then_pump, daemon=True).start()

    def _align_then_pump(self) -> None:
        q_urdf = self._latest_q_urdf
        if q_urdf is None or _urdf_to_dobot is None:
            return
        try:
            q_dobot_rad = _urdf_to_dobot(q_urdf)
            q_dobot_deg = [math.degrees(float(v)) for v in q_dobot_rad]
            with self._real_lock:
                self._real_driver.mov_j_joint_deg(q_dobot_deg)
                self._real_driver.sync()
        except Exception as exc:
            self.get_logger().error(f'PTP de alinhamento falhou: {exc}')
            self._set_status(
                f'Falha no PTP de alinhamento: {exc}', DANGER)
            return
        # Pode ter sido cancelado durante o PTP.
        if self._robot_mode != 'MIRROR' or not self._robot_connected:
            self._set_status(
                'PTP concluído, mas modo mudou — pump não iniciado.',
                TEXT_DIM)
            return
        self._real_pump_stop.clear()
        self._real_pump_thread = threading.Thread(
            target=self._real_pump_loop, daemon=True)
        self._real_pump_thread.start()
        self._set_status(
            'Pump ServoJ ativo — real seguindo o sim @33 Hz.', OK)

    def _real_pump_loop(self) -> None:
        """Loop @33 Hz: ServoJ com o último q_urdf cacheado."""
        period = 0.030
        while not self._real_pump_stop.is_set():
            t0 = time.time()
            q = self._latest_q_urdf
            if (q is not None and self._real_driver is not None
                    and self._robot_connected):
                try:
                    with self._real_lock:
                        self._real_driver.servo_j_urdf(q.tolist())
                except Exception as exc:    # pragma: no cover
                    self.get_logger().error(
                        f'ServoJ pump erro: {exc}')
            dt = time.time() - t0
            if dt < period:
                # Wait responsivo ao stop event.
                self._real_pump_stop.wait(period - dt)

    def _stop_real_pump(self) -> None:
        if not self._real_pump_active():
            self._real_pump_stop.set()
            self._real_pump_thread = None
            return
        self._real_pump_stop.set()
        thr = self._real_pump_thread
        if thr is not None:
            thr.join(timeout=1.5)
        self._real_pump_thread = None

    def _publish_hand_from_sliders(self):
        if self._suppressing:
            return
        self._suppressing = True
        try:
            primary_rad: dict[str, float] = {}
            for j in HAND_JOINTS:
                lo, hi = HAND_LIMITS_DEG[j]
                v = self._clamp_var(self.hand_sliders[j], lo, hi)
                if v is None:
                    return
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
        msg.joint_names = names
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions]
        pt.time_from_start = self._duration_msg(duration_s)
        msg.points.append(pt)
        self._hand_pub.publish(msg)

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

    def _apply_hand_preset(self, preset_deg: dict[str, float]):
        """Aplica um preset de mão (Abrir/Fechar)."""
        self._suppressing = True
        try:
            for j in HAND_JOINTS:
                self.hand_sliders[j].set(preset_deg.get(j, 0))
        finally:
            self._suppressing = False
        self._publish_hand_from_sliders()

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
        cmd = ['ros2', 'run', 'covvi_hand_driver', 'server', ip,
                f'--ros-args', '-r', f'__ns:={self._eci_prefix}']
        try:
            self._hand_proc = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        except FileNotFoundError:
            self._set_status('ros2 não está no PATH (source o workspace).',
                              DANGER)
            self._hand_proc = None
            return
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
        self._set_status(
            f'Driver da mão ativo no namespace {self._eci_prefix}.', OK)

    def _disconnect_real_hand(self) -> None:
        proc = self._hand_proc
        if proc is not None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception:
                pass
        self._hand_proc = None
        # Desliga ECI/PWR junto.
        if self._eci_enabled:
            self._toggle_eci()
        if self._hand_powered:
            self._toggle_hand_power()
        self._hand_connect_btn.set_state(
            '⚡', 'Conectar', PRIMARY, 'white')
        self._set_status('Driver da mão desconectado.', TEXT_DIM)

    def _toggle_eci(self) -> None:
        """Liga/desliga o canal lógico ECI (cliente dos serviços COVVI).

        Precisa do pacote `covvi_interfaces` sourceado no workspace.
        """
        if self._eci_enabled:
            self._eci_enabled = False
            self._eci_btn.set_state('◉', 'ECI OFF', BTN_NEUTRAL, TEXT)
            self._set_status('Canal ECI desativado.', TEXT_DIM)
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
        if self._cli_eci_grip is None:
            self._cli_eci_grip = self.create_client(
                _eci_srv.SetCurrentGrip,
                f'{self._eci_prefix}/set_current_grip')
        if self._cli_hand_pwr_on is None:
            self._cli_hand_pwr_on = self.create_client(
                _eci_srv.SetHandPowerOn,
                f'{self._eci_prefix}/set_hand_power_on')
        if self._cli_hand_pwr_off is None:
            self._cli_hand_pwr_off = self.create_client(
                _eci_srv.SetHandPowerOff,
                f'{self._eci_prefix}/set_hand_power_off')
        self._eci_enabled = True
        self._eci_btn.set_state('◉', 'ECI ON', OK, 'white')
        self._set_status('Canal ECI ativo — pronto para grips e power.', OK)

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
        try:
            cfg = CR10RealDriverConfig(ip=ip)
            drv = CR10RealDriver(ip=ip, dry_run=False, config=cfg)
            drv.connect()
            drv.enable()
        except CR10RealDriverError as exc:
            self._set_status(
                f'Falha ao conectar CR10 ({ip}): {exc}', DANGER)
            return
        except Exception as exc:    # pragma: no cover
            self._set_status(
                f'Erro inesperado conectando CR10: {exc}', DANGER)
            return
        self._real_driver = drv
        self._robot_connected = True
        self._robot_connect_btn.set_state('⚡', 'Desconectar', OK, 'white')
        self._set_status(
            f'CR10 conectado em {ip} (modo {self._robot_mode}).', OK)
        # Se o usuário já escolheu MIRROR antes de conectar, sobe o pump.
        if self._robot_mode == 'MIRROR':
            self._start_real_pump()

    def _disconnect_real_robot(self) -> None:
        # Pump precisa parar ANTES de fechar os sockets, senão a thread
        # ainda tenta `servo_j` num socket morto.
        self._stop_real_pump()
        drv = self._real_driver
        if drv is None:
            self._robot_connected = False
            self._robot_connect_btn.set_state(
                '⚡', 'Conectar', PRIMARY, 'white')
            return
        try: drv.stop()
        except Exception: pass
        try: drv.close()
        except Exception: pass
        self._real_driver = None
        self._robot_connected = False
        self._robot_connect_btn.set_state('⚡', 'Conectar', PRIMARY, 'white')
        self._set_status('CR10 desconectado.', TEXT_DIM)

    def _set_robot_mode(self, selected: str) -> None:
        mode = (selected or '').strip().upper()
        if mode not in ('SIM_ONLY', 'MIRROR'):
            return
        self._robot_mode = mode
        if mode == 'MIRROR':
            # Sobe o pump se já conectado; caso contrário aguarda a
            # conexão (o `_connect_real_robot` checa o modo no final).
            self._set_status(
                'Modo MIRROR — robô real espelha o sim @33 Hz (ServoJ).',
                DANGER)
            if self._robot_connected:
                self._start_real_pump()
        else:
            self._stop_real_pump()
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
        # Parar o pump primeiro — caso contrário ele continuaria a
        # mandar ServoJ no socket logo depois do StopRobot.
        self._stop_real_pump()
        if self._real_driver is not None and self._robot_connected:
            try:
                self._real_driver.stop()
            except Exception as exc:
                self.get_logger().error(f'E-STOP real falhou: {exc}')
        if self._eci_enabled and self._cli_eci_grip is not None \
                and self._eci_srv is not None:
            try:
                req = self._eci_srv.SetCurrentGrip.Request()
                # 11 = Glove (mão totalmente aberta — mesmo ID usado no
                # manual_control original).
                req.grip = self._eci_msg.Grip()
                req.grip.value = 11
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
            self._latest_phase = str(data.get('phase', 'IDLE'))

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

        has_data = last_ts > 0.0
        if not has_data:
            self.force_value_lbl.config(text='—   N', fg=TEXT_DIM)
            self.force_status_lbl.config(
                text='aguardando /ft_sensor/wrench', fg=TEXT_DIM)
            self.err_value_lbl.config(text='—  N', fg=TEXT_DIM)
        else:
            err = abs(f_normal - tgt)
            if f_normal < 0.05:
                color, status = DANGER, 'sem contato detectado'
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
        finally:
            self._suppressing = False
        if None in (speed, force, dist, tgt):
            self._set_status('Parâmetros inválidos.', DANGER)
            return
        payload = {
            'speed_mms':          float(speed),
            'force_n':            float(force),
            'distance_mm':        float(dist),
            'target_distance_cm': float(tgt),
            # Direção XY (mundo) do sliding — string '+X' / '-X' / '+Y' / '-Y'.
            'slide_dir':          self.slide_dir_var.get(),
            # Home customizada: explorer leva o braço PARA CÁ antes
            # de descer. Em graus URDF, mesma chave/ordem do
            # `_apply_arm_home`.
            'home_deg': {j: float(self._arm_home_deg[j])
                          for j in ARM_JOINTS},
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._start_pub.publish(msg)
        self._set_status(
            f'/palpation/start publicado — v={payload["speed_mms"]:.1f} mm/s, '
            f'F={payload["force_n"]:.2f} N, '
            f'slide={payload["distance_mm"]:.0f} mm {payload["slide_dir"]}, '
            f'descida={payload["target_distance_cm"]:.1f} cm.',
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
        self._stop_real_pump()
        # Mata subprocesso da mão (se aberto).
        if self._hand_proc is not None and self._hand_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._hand_proc.pid), signal.SIGTERM)
            except Exception:
                pass
        # Fecha sockets do CR10.
        if self._real_driver is not None:
            try: self._real_driver.stop()
            except Exception: pass
            try: self._real_driver.close()
            except Exception: pass
        try:
            self.root.destroy()
        except Exception:
            pass


def main(args=None):
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
