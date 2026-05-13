"""
Controle manual da célula de manufatura — CR10 + COVVI + Esteira + Mão Real.

GUI Tkinter que combina:
  • Sliders por junta do braço (6 sliders, em graus)
  • Sliders por junta da mão  (6 sliders 0..200, mapeados para rad)
  • Botões de pose pré-calculada do braço (Pick Frasco/Tubo/Ampola)
  • Botões de preensão do projeto (Palm/Claw/Fingertip Grip) → Gazebo
  • Botões de preensão ECI nativa (14 grips COVVI) → Gazebo + Mão Real
  • Controles da esteira (spawn / remover / reset)
  • Toggle de mão real: habilita/desabilita envio ao controlador ECI

Parâmetros ROS:
  eci_prefix  (string, default '/covvi/hand')
      Prefixo do servidor ECI: /{namespace}/{server_name}
      Exemplo: servidor iniciado com --remap __ns:=/covvi --remap __name:=hand

Uso:
  ros2 launch grasp_ml_pack conveyor_cell.launch.py no_gui:=true
  ros2 run   grasp_ml_pack manual_control

  # ou com prefixo ECI customizado:
  ros2 run grasp_ml_pack manual_control --ros-args -p eci_prefix:=/test/server_1

Verificar comandos enviados à mão real:
  ros2 topic echo /covvi/hand/CurrentGripMsg
  ros2 topic echo /covvi/hand/DigitPosnAllMsg

(Pode rodar lado-a-lado com o `gui_control` automático — os tópicos
de junta aceitam múltiplos publishers.)
"""

from __future__ import annotations

import json
import math
import queue
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

# ECI modules are loaded lazily inside _toggle_eci() so that importing
# covvi_interfaces (which loads FastDDS C extensions) never happens at
# module-load time — those extensions corrupt Tcl's internal state before
# tk.Tk() is called and cause a segfault on startup.


# ── ECI built-in grip IDs (CurrentGripID.value) ───────────────────────
ECI_GRIP_IDS: dict[str, int] = {
    'Tripod':       1,
    'Power':        2,
    'Trigger':      3,
    'Prec. Open':   4,
    'Prec. Closed': 5,
    'Key':          6,
    'Finger':       7,
    'Cylinder':     8,
    'Column':       9,
    'Relaxed':     10,
    'Glove':       11,
    'Tap':         12,
    'Grab':        13,
    'Tripod Open': 14,
}
ECI_GRIP_NAMES: dict[int, str] = {v: k for k, v in ECI_GRIP_IDS.items()}

# Approximate Gazebo joint configs for each ECI grip (0..200 slider scale).
# Values derived from COVVI grip geometry documentation + IK fitting.
ECI_GRIP_GAZEBO: dict[str, dict[str, int]] = {
    'Tripod':       {'Thumb': 125, 'Index': 115, 'Middle': 115, 'Ring':   0, 'Little':  0, 'Rotate': 145},
    'Power':        {'Thumb': 155, 'Index': 165, 'Middle': 165, 'Ring': 160, 'Little': 155, 'Rotate':  40},
    'Trigger':      {'Thumb': 100, 'Index':   0, 'Middle': 140, 'Ring': 140, 'Little': 140, 'Rotate':  70},
    'Prec. Open':   {'Thumb':  50, 'Index':  50, 'Middle':   0, 'Ring':   0, 'Little':   0, 'Rotate': 155},
    'Prec. Closed': {'Thumb': 105, 'Index': 100, 'Middle':   0, 'Ring':   0, 'Little':   0, 'Rotate': 155},
    'Key':          {'Thumb': 115, 'Index': 130, 'Middle': 130, 'Ring': 125, 'Little': 115, 'Rotate':  10},
    'Finger':       {'Thumb':  60, 'Index':   0, 'Middle': 100, 'Ring': 100, 'Little': 100, 'Rotate':  60},
    'Cylinder':     {'Thumb': 130, 'Index': 150, 'Middle': 155, 'Ring': 150, 'Little': 140, 'Rotate':  35},
    'Column':       {'Thumb': 100, 'Index': 140, 'Middle': 140, 'Ring': 140, 'Little': 140, 'Rotate':  80},
    'Relaxed':      {'Thumb':  20, 'Index':  20, 'Middle':  20, 'Ring':  20, 'Little':  20, 'Rotate':   5},
    'Glove':        {'Thumb':   0, 'Index':   0, 'Middle':   0, 'Ring':   0, 'Little':   0, 'Rotate':   0},
    'Tap':          {'Thumb':   0, 'Index':   0, 'Middle': 160, 'Ring': 160, 'Little': 160, 'Rotate':  50},
    'Grab':         {'Thumb': 165, 'Index': 175, 'Middle': 175, 'Ring': 175, 'Little': 170, 'Rotate':  45},
    'Tripod Open':  {'Thumb':  60, 'Index':  50, 'Middle':  50, 'Ring':   0, 'Little':   0, 'Rotate': 145},
}

# Closest ECI grip for each project preset (sent to real hand when ECI enabled)
PROJECT_GRIP_ECI: dict[str, int] = {
    'Palm Grip (frasco)':      8,   # Cylinder — full wrap
    'Claw Grip (tubo)':        1,   # Tripod   — 3-finger partial
    'Fingertip Grip (ampola)': 5,   # Prec. Closed — fingertip precision
}


# ── Mimic map COVVI (extraído do URDF combinado) ──────────────────────
MIMIC_JOINTS = [
    ('_lisa_j01',            'Rotate', 1.07337741974876),
    ('_thumb_chassis_j01',   'Rotate', 1.53339618284689),
    ('_thumb_proximal_j01',  'Thumb',  0.72022188617106),
    ('_thumb_distal_j01',    'Thumb',  1.06686018440504),
    ('_thumb_link_j01',      'Thumb',  0.76799454671462),
    ('_thumb_follower_j01',  'Thumb',  0.93732763826281),
    ('_index_proximal_j01',  'Index',  1.51604339913514),
    ('_index_distal_j01',    'Index',  1.33574108836936),
    ('_index_knuckle_j01',   'Index',  1.25181519799450),
    ('_index_follower_j01',  'Index',  0.26422627443924),
    ('_index_link_j01',      'Index',  1.33574038782548),
    ('_middle_proximal_j01', 'Middle', 1.51604368978713),
    ('_middle_distal_j01',   'Middle', 1.34986011532341),
    ('_middle_knuckle_j01',  'Middle', 1.25181499257525),
    ('_middle_follower_j01', 'Middle', 0.26422641895880),
    ('_middle_link_j01',     'Middle', 1.34986028913701),
    ('_ring_proximal_j01',   'Ring',   1.51604328762194),
    ('_ring_distal_j01',     'Ring',   1.34878317629563),
    ('_ring_knuckle_j01',    'Ring',   1.25181510906761),
    ('_ring_follower_j01',   'Ring',   0.26423062522385),
    ('_ring_link_j01',       'Ring',   1.34878364034377),
    ('_little_proximal_j01', 'Little', 1.51604353824541),
    ('_little_distal_j01',   'Little', 1.31664152870820),
    ('_little_knuckle_j01',  'Little', 1.25181529061989),
    ('_little_follower_j01', 'Little', 0.26422625333146),
    ('_little_link_j01',     'Little', 1.31664159359670),
]

MAX_RAD = {'Thumb': 1.6, 'Index': 1.6, 'Middle': 1.6,
           'Ring':  1.6, 'Little': 1.6, 'Rotate': 1.0}

HAND_JOINTS = ['Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Rotate']
ARM_JOINTS  = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

ARM_LIMITS_DEG = {
    'joint1': (-180, 180), 'joint2': (-260,  80),
    'joint3': (-135, 135), 'joint4': (-260,  80),
    'joint5': (-135, 135), 'joint6': (-360, 360),
}

ARM_PRESETS_BASE = {
    'Home':      {'joint1':   0, 'joint2':   0, 'joint3':  90,
                  'joint4': -90, 'joint5': -90, 'joint6':   0},
    'Vertical':  {'joint1':   0, 'joint2': -90, 'joint3':   0,
                  'joint4':   0, 'joint5':  90, 'joint6':   0},
    'Estendido': {'joint1':   0, 'joint2':   0, 'joint3': -90,
                  'joint4':   0, 'joint5':   0, 'joint6':   0},
}

PICK_POSES_DEG = {
    'Pick Frasco': {
        'joint1': +30.0, 'joint2': -19.0, 'joint3': -82.0,
        'joint4': -80.0, 'joint5': +31.0, 'joint6':  +0.0,
    },
    'Pick Tubo': {
        'joint1': +28.0, 'joint2': -18.0, 'joint3': -82.0,
        'joint4': -81.0, 'joint5': +24.0, 'joint6':  +0.0,
    },
    'Pick Ampola': {
        'joint1': +30.0, 'joint2': -20.0, 'joint3': -84.0,
        'joint4': -76.0, 'joint5': +34.0, 'joint6':  +0.0,
    },
}

APPROACH_POSE_DEG = {
    'joint1': +23.5, 'joint2': -16.5, 'joint3': -82.0,
    'joint4': -81.0, 'joint5': +23.5, 'joint6':  +0.0,
}

DELIVERY_POSES_DEG = {
    'frasco': {
        'joint1': +108.1, 'joint2': -20.5, 'joint3': -90.1,
        'joint4':  -69.4, 'joint5': +108.1, 'joint6':  +0.0,
    },
    'tubo': {
        'joint1':  +85.8, 'joint2': -24.1, 'joint3': -85.0,
        'joint4':  -70.9, 'joint5':  +85.8, 'joint6':  +0.0,
    },
    'ampola': {
        'joint1':  +65.8, 'joint2': -99.0, 'joint3': +63.5,
        'joint4': -144.5, 'joint5':  +67.2, 'joint6':  -1.4,
    },
}

HAND_GRIPS = {
    'Open':                     {j: 0 for j in HAND_JOINTS},
    'Palm Grip (frasco)': {
        'Thumb': 138, 'Index': 156, 'Middle': 156,
        'Ring':  150, 'Little': 138, 'Rotate': 50,
    },
    'Claw Grip (tubo)': {
        'Thumb': 112, 'Index': 138, 'Middle': 138,
        'Ring':  131, 'Little': 119, 'Rotate': 90,
    },
    'Fingertip Grip (ampola)': {
        'Thumb': 94, 'Index': 88, 'Middle': 81,
        'Ring':   6, 'Little':  6, 'Rotate': 164,
    },
}

OBJ_RECIPE = {
    'frasco': ('Pick Frasco', 'Palm Grip (frasco)'),
    'tubo':   ('Pick Tubo',   'Claw Grip (tubo)'),
    'ampola': ('Pick Ampola', 'Fingertip Grip (ampola)'),
}

# ── Palette ───────────────────────────────────────────────────────────
BG, PANEL_BG, HEADER_BG = '#1a1a2e', '#16213e', '#0f3460'
ACCENT_ARM, ACCENT_HND  = '#4fc3f7', '#69f0ae'
TEXT_MAIN, TEXT_DIM, TEXT_VAL = '#e0e0e0', '#9e9e9e', '#ffd54f'
BTN_PRESET, TROUGH       = '#37474f', '#2a2a4a'
BTN_CONV_FWD, BTN_CONV_RV, BTN_CONV_RST = '#388e3c', '#c62828', '#5d4037'
COLOR_FRASCO, COLOR_TUBO, COLOR_AMPOLA = '#e65100', '#1565c0', '#2e7d32'

# ECI grip button colours — grouped by grip family
_ECI_CLR: dict[str, str] = {
    'Tripod':       '#6a1b9a', 'Tripod Open':  '#8e24aa',
    'Power':        '#b71c1c', 'Grab':         '#c62828',
    'Trigger':      '#e65100', 'Tap':          '#bf360c',
    'Prec. Open':   '#0277bd', 'Prec. Closed': '#01579b',
    'Key':          '#00695c', 'Cylinder':     '#2e7d32',
    'Column':       '#4e342e', 'Finger':       '#37474f',
    'Relaxed':      '#37474f', 'Glove':        '#455a64',
}


class ManualControlNode(Node):
    """Nó ROS 2 + GUI Tkinter para controle manual da célula."""

    def __init__(self, root: tk.Tk):
        super().__init__('manual_control')
        self._ready = False
        self._suppressing = False
        self._pending_hand_after: str | None = None
        self.root = root   # Created in main() before rclpy.init()

        cb = ReentrantCallbackGroup()

        self.arm_pub = self.create_publisher(
            JointTrajectory, '/cr10_group_controller/joint_trajectory', 10)
        self.hand_pub = self.create_publisher(
            JointTrajectory, '/hand_position_controller/joint_trajectory', 10)

        self._cli = {
            'retreat':       self.create_client(
                Trigger, '/conveyor/retreat',       callback_group=cb),
            'reset':         self.create_client(
                Trigger, '/conveyor/reset',         callback_group=cb),
            'spawn_frasco':  self.create_client(
                Trigger, '/conveyor/spawn_frasco',  callback_group=cb),
            'spawn_tubo':    self.create_client(
                Trigger, '/conveyor/spawn_tubo',    callback_group=cb),
            'spawn_ampola':  self.create_client(
                Trigger, '/conveyor/spawn_ampola',  callback_group=cb),
        }

        self._conveyor_state: dict = {}
        self._last_detection: str | None = None
        self._status_q: queue.Queue = queue.Queue()

        self.create_subscription(
            String, '/conveyor/status', self._cb_conveyor, 10,
            callback_group=cb)
        self.create_subscription(
            Detection2DArray, '/detected_objects', self._cb_detection, 10,
            callback_group=cb)

        # ── ECI real-hand setup (lazy — clients created on first toggle) ──
        self._eci_enabled = False
        eci_prefix = self.declare_parameter('eci_prefix', '/covvi/hand').value
        self._eci_prefix: str = eci_prefix
        self._eci_grip_id: int | None = None      # last received from CurrentGripMsg
        self._eci_posn: dict[str, int] = {}        # last received from DigitPosnAllMsg
        self._eci_last_sent: str | None = None     # name of last sent ECI grip
        self._eci_posn_after: str | None = None    # debounce token for SetDigitPosn
        self._eci_srv = None                       # covvi_interfaces.srv (loaded lazily)
        self._eci_msg = None                       # covvi_interfaces.msg (loaded lazily)
        self._cli_eci = None
        self._cli_eci_posn = None

        self._build_ui()
        self.root.deiconify()   # show window now that it's fully built
        self._ready = True
        self._spin_ros()
        self._poll_status()

    # ──────────────────────────────────────────────────────────────────
    # Conveyor & detection callbacks
    # ──────────────────────────────────────────────────────────────────
    def _cb_conveyor(self, msg: String):
        try:
            self._conveyor_state = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _cb_detection(self, msg: Detection2DArray):
        if msg.detections:
            try:
                cls = msg.detections[0].results[0].hypothesis.class_id
                self._last_detection = cls if cls in DELIVERY_POSES_DEG else None
            except (IndexError, AttributeError):
                self._last_detection = None
        else:
            self._last_detection = None

    def _call_conveyor(self, key: str, pending_msg: str | None = None):
        cli = self._cli[key]
        if not cli.service_is_ready():
            self._set_status(f'Serviço /conveyor/{key} indisponível', _CLR=TEXT_DIM)
            return
        if pending_msg:
            self._set_status(pending_msg, _CLR='#fab387')
        future = cli.call_async(Trigger.Request())

        def _done(f):
            try:
                res = f.result()
                color = ACCENT_HND if res.success else '#f38ba8'
                self._status_q.put((res.message, color))
            except Exception as exc:
                self._status_q.put((str(exc), '#f38ba8'))

        future.add_done_callback(_done)

    # ──────────────────────────────────────────────────────────────────
    # ECI callbacks
    # ──────────────────────────────────────────────────────────────────
    def _cb_eci_grip(self, msg) -> None:
        self._eci_grip_id = int(msg.grip_id.value)

    def _cb_eci_posn(self, msg) -> None:
        self._eci_posn = {
            'Thumb':  msg.thumb_pos,
            'Index':  msg.index_pos,
            'Middle': msg.middle_pos,
            'Ring':   msg.ring_pos,
            'Little': msg.little_pos,
            'Rotate': msg.rotate_pos,
        }

    def _send_eci_grip(self, grip_id: int, label: str) -> None:
        """Call SetCurrentGrip on the ECI server (non-blocking)."""
        if not self._eci_enabled or self._cli_eci is None:
            return
        if not self._cli_eci.service_is_ready():
            self._status_q.put(
                (f'ECI SetCurrentGrip indisponível ({self._eci_prefix})', '#f38ba8'))
            return
        req = self._eci_srv.SetCurrentGrip.Request()
        req.grip_id = self._eci_msg.CurrentGripID()
        req.grip_id.value = grip_id
        self._eci_last_sent = label
        future = self._cli_eci.call_async(req)

        def _done(f):
            try:
                f.result()
                self._status_q.put(
                    (f'ECI → {label} (id={grip_id}) enviado', ACCENT_HND))
            except Exception as exc:
                self._status_q.put((f'ECI erro: {exc}', '#f38ba8'))

        future.add_done_callback(_done)

    def _schedule_eci_posn(self, vals: dict) -> None:
        """Debounce digit-position sends (60 ms): cancels any pending call and
        reschedules so rapid slider drags collapse into one service call."""
        if not self._eci_enabled or self._cli_eci_posn is None:
            return
        if self._eci_posn_after is not None:
            try:
                self.root.after_cancel(self._eci_posn_after)
            except Exception:
                pass
        self._eci_posn_after = self.root.after(
            60, lambda v=dict(vals): self._send_eci_posn_now(v))

    def _send_eci_posn_now(self, vals: dict) -> None:
        """Fire SetDigitPosn to the real hand (0-200 scale, speed=50)."""
        self._eci_posn_after = None
        if not self._eci_enabled or self._cli_eci_posn is None:
            return
        if not self._cli_eci_posn.service_is_ready():
            self._status_q.put(
                (f'ECI SetDigitPosn indisponível ({self._eci_prefix})', '#f38ba8'))
            return
        req = self._eci_srv.SetDigitPosn.Request()
        req.speed = self._eci_msg.Speed()
        req.speed.value = 50          # mid-range (MIN=15, MAX=100)
        req.thumb  = int(vals.get('Thumb',  0))
        req.index  = int(vals.get('Index',  0))
        req.middle = int(vals.get('Middle', 0))
        req.ring   = int(vals.get('Ring',   0))
        req.little = int(vals.get('Little', 0))
        req.rotate = int(vals.get('Rotate', 0))
        self._eci_last_sent = (
            f"T:{req.thumb} I:{req.index} M:{req.middle} "
            f"R:{req.ring} L:{req.little} Rot:{req.rotate}"
        )
        future = self._cli_eci_posn.call_async(req)
        future.add_done_callback(lambda f: None)   # fire-and-forget

    def _toggle_eci(self) -> None:
        if self._eci_enabled:
            # Turn OFF
            self._eci_enabled = False
            self._eci_toggle_btn.config(text='Real Hand: OFF ✗', fg=TEXT_DIM)
            self._set_status('Mão real desativada', _CLR=TEXT_DIM)
            return

        # Turn ON — lazy-load covvi_interfaces and create clients/subs
        if self._cli_eci is None:
            try:
                import covvi_interfaces.srv as _eci_srv
                import covvi_interfaces.msg as _eci_msg
            except ImportError:
                self._set_status(
                    'covvi_interfaces não instalado — instale e recompile o workspace.',
                    _CLR='#f38ba8')
                return
            self._eci_srv = _eci_srv
            self._eci_msg = _eci_msg
            cb = ReentrantCallbackGroup()
            self._cli_eci = self.create_client(
                _eci_srv.SetCurrentGrip,
                f'{self._eci_prefix}/SetCurrentGrip',
                callback_group=cb)
            self._cli_eci_posn = self.create_client(
                _eci_srv.SetDigitPosn,
                f'{self._eci_prefix}/SetDigitPosn',
                callback_group=cb)
            self.create_subscription(
                _eci_msg.CurrentGripMsg,
                f'{self._eci_prefix}/CurrentGripMsg',
                self._cb_eci_grip, 10, callback_group=cb)
            self.create_subscription(
                _eci_msg.DigitPosnAllMsg,
                f'{self._eci_prefix}/DigitPosnAllMsg',
                self._cb_eci_posn, 10, callback_group=cb)

        self._eci_enabled = True
        self._eci_toggle_btn.config(text='Real Hand: ON  ✔', fg=ACCENT_HND)
        self._set_status(
            f'Mão real ativada ({self._eci_prefix}/SetCurrentGrip)',
            _CLR=ACCENT_HND)

    # ──────────────────────────────────────────────────────────────────
    # UI construction
    # ──────────────────────────────────────────────────────────────────
    def _build_ui(self):
        self.root.title('Célula de Manufatura — Controle Manual')
        self.root.configure(bg=BG)
        self.root.minsize(1060, 700)

        style = ttk.Style()
        style.theme_use('clam')

        # ── Cabeçalho ────────────────────────────────────────────────
        hdr = tk.Frame(self.root, bg=HEADER_BG)
        hdr.pack(fill='x')
        tk.Label(hdr, text='CR10 + COVVI  —  Modo Manual da Célula',
                 font=('Arial', 14, 'bold'), bg=HEADER_BG, fg=TEXT_MAIN,
                 pady=10).pack(side='left', padx=18)

        # ECI toggle (right side of header)
        self._eci_toggle_btn = tk.Button(
            hdr, text='Real Hand: OFF ✗',
            font=('Courier', 10, 'bold'), bg=HEADER_BG, fg=TEXT_DIM,
            activebackground=HEADER_BG, activeforeground=ACCENT_HND,
            relief='flat', padx=10, pady=4,
            command=self._toggle_eci)
        self._eci_toggle_btn.pack(side='right', padx=10)

        self.obj_lbl = tk.Label(
            hdr, text='○ esteira vazia',
            font=('Courier', 11, 'bold'), bg=HEADER_BG, fg=TEXT_DIM)
        self.obj_lbl.pack(side='right', padx=16)

        # ── Barra da esteira ─────────────────────────────────────────
        bar = tk.Frame(self.root, bg=BG)
        bar.pack(fill='x', padx=12, pady=8)

        conv = tk.Frame(bar, bg=BG)
        conv.pack(side='left')
        tk.Label(conv, text='Spawn: ', font=('Arial', 10),
                 bg=BG, fg=TEXT_DIM).pack(side='left', padx=(0, 6))
        for label, color, srv_key in [
            ('Frasco', COLOR_FRASCO, 'spawn_frasco'),
            ('Tubo',   COLOR_TUBO,   'spawn_tubo'),
            ('Ampola', COLOR_AMPOLA, 'spawn_ampola'),
        ]:
            tk.Button(
                conv, text=label, bg=color, fg='white',
                activebackground='#212121', activeforeground='white',
                relief='flat', padx=10, pady=4, font=('Arial', 9, 'bold'),
                command=lambda k=srv_key, l=label: self._call_conveyor(
                    k, f'Spawnando {l}...'),
            ).pack(side='left', padx=2)
        tk.Button(conv, text='◀  Remover', bg=BTN_CONV_RV, fg='white',
                  activebackground='#a31818', activeforeground='white',
                  relief='flat', padx=10, pady=4, font=('Arial', 9),
                  command=lambda: self._call_conveyor('retreat', 'Removendo objeto...')
                  ).pack(side='left', padx=2)
        tk.Button(conv, text='⟳  Reset', bg=BTN_CONV_RST, fg='white',
                  activebackground='#795548', activeforeground='white',
                  relief='flat', padx=10, pady=4, font=('Arial', 9),
                  command=lambda: self._call_conveyor('reset', 'Resetando esteira...')
                  ).pack(side='left', padx=2)

        dur = tk.Frame(bar, bg=BG)
        dur.pack(side='right')
        tk.Label(dur, text='Duração:', font=('Arial', 10),
                 bg=BG, fg=TEXT_DIM).pack(side='left', padx=(0, 6))
        self.dur_val_lbl = tk.Label(dur, text='2.0 s',
                                     font=('Courier', 10, 'bold'),
                                     bg=BG, fg=TEXT_VAL, width=6)
        self.dur_val_lbl.pack(side='right', padx=(6, 0))
        self.time_sl = tk.Scale(
            dur, from_=0.3, to=8.0, resolution=0.1,
            orient='horizontal', length=220, showvalue=False,
            bg=BG, fg=TEXT_DIM, troughcolor=TROUGH,
            activebackground=ACCENT_ARM, highlightthickness=0,
            command=lambda v: self.dur_val_lbl.config(text=f'{float(v):.1f} s'))
        self.time_sl.set(2.0)
        self.time_sl.pack(side='left')

        # ── Recipe row ───────────────────────────────────────────────
        recipe_row = tk.Frame(self.root, bg=BG)
        recipe_row.pack(fill='x', padx=12, pady=(0, 6))
        tk.Label(recipe_row, text='Pick (braço → mão):',
                 font=('Arial', 9, 'italic'),
                 bg=BG, fg=TEXT_DIM).pack(side='left', padx=(0, 8))
        for obj, (arm_key, grip_key) in OBJ_RECIPE.items():
            color = {'frasco': COLOR_FRASCO, 'tubo': COLOR_TUBO,
                     'ampola': COLOR_AMPOLA}[obj]
            tk.Button(
                recipe_row, text=f'⚙  {obj.upper()}',
                bg=color, fg='white',
                activebackground='#212121', activeforeground='white',
                relief='flat', padx=10, pady=5, font=('Arial', 9, 'bold'),
                command=lambda a=arm_key, g=grip_key: self._apply_recipe(a, g),
            ).pack(side='left', padx=4)

        tk.Label(recipe_row, text='   ·   ', font=('Arial', 11),
                 bg=BG, fg=TEXT_DIM).pack(side='left')
        self._btn_entrega = tk.Button(
            recipe_row, text='📦  ENTREGA',
            bg='#7e57c2', fg='white',
            activebackground='#5e35b1', activeforeground='white',
            relief='flat', padx=14, pady=5, font=('Arial', 10, 'bold'),
            command=self._do_delivery,
        )
        self._btn_entrega.pack(side='left', padx=4)
        self._entrega_hint = tk.Label(
            recipe_row, text='(aguardando detecção)',
            font=('Arial', 8, 'italic'), bg=BG, fg=TEXT_DIM)
        self._entrega_hint.pack(side='left', padx=(4, 0))

        # ── Colunas — Braço | Mão ────────────────────────────────────
        cols = tk.Frame(self.root, bg=BG)
        cols.pack(fill='both', expand=True, padx=10, pady=(0, 6))
        cols.columnconfigure(0, weight=1)
        cols.columnconfigure(1, weight=1)
        cols.rowconfigure(0, weight=1)
        self._build_arm_panel(cols)
        self._build_hand_panel(cols)

        # ── Status bar ───────────────────────────────────────────────
        self.status_var = tk.StringVar(value='Pronto.')
        sb = tk.Frame(self.root, bg=PANEL_BG)
        sb.pack(fill='x')
        self.status_lbl = tk.Label(sb, textvariable=self.status_var,
                                    font=('Arial', 10), bg=PANEL_BG,
                                    fg=TEXT_MAIN, anchor='w', padx=10, pady=6)
        self.status_lbl.pack(fill='x')

    # ──────────────────────────────────────────────────────────── BRAÇO
    def _build_arm_panel(self, parent):
        outer = tk.Frame(parent, bg=BG)
        outer.grid(row=0, column=0, sticky='nsew', padx=(0, 5))

        header = tk.Frame(outer, bg=HEADER_BG)
        header.pack(fill='x')
        tk.Label(header, text='🦾  Braço CR10', font=('Arial', 12, 'bold'),
                 bg=HEADER_BG, fg=ACCENT_ARM, pady=7, padx=10
                 ).pack(side='left')

        body = tk.Frame(outer, bg=PANEL_BG, padx=10, pady=8)
        body.pack(fill='both', expand=True)

        self.arm_sliders, self.arm_labels = {}, {}
        for j in ARM_JOINTS:
            lo, hi = ARM_LIMITS_DEG[j]
            row = tk.Frame(body, bg=PANEL_BG)
            row.pack(fill='x', pady=3)
            tk.Label(row, text=j, font=('Arial', 10, 'bold'),
                     bg=PANEL_BG, fg=TEXT_MAIN, width=8, anchor='w'
                     ).pack(side='left')
            val = tk.Label(row, text='   0°', font=('Courier', 10, 'bold'),
                            bg=PANEL_BG, fg=TEXT_VAL, width=7, anchor='e')
            val.pack(side='right')
            sl = tk.Scale(row, from_=lo, to=hi, resolution=1,
                           orient='horizontal', showvalue=False,
                           bg=PANEL_BG, fg=TEXT_DIM, troughcolor=TROUGH,
                           activebackground=ACCENT_ARM, highlightthickness=0,
                           command=lambda v, lbl=val, jn=j: self._arm_changed(v, lbl, jn))
            sl.set(0)
            sl.pack(side='left', fill='x', expand=True)
            self.arm_sliders[j], self.arm_labels[j] = sl, val

        b1 = tk.Frame(body, bg=PANEL_BG); b1.pack(fill='x', pady=(10, 2))
        for label, preset in ARM_PRESETS_BASE.items():
            tk.Button(b1, text=label, bg=BTN_PRESET, fg=TEXT_MAIN,
                      activebackground='#546e7a', activeforeground=TEXT_MAIN,
                      relief='flat', padx=10, pady=5, font=('Arial', 9),
                      command=lambda p=preset: self._apply_arm(p)
                      ).pack(side='left', padx=3, fill='x', expand=True)

        tk.Label(body, text='—— Aproximação e Captura ——',
                 font=('Arial', 9, 'italic'), bg=PANEL_BG, fg=TEXT_DIM
                 ).pack(fill='x', pady=(8, 2))
        b2 = tk.Frame(body, bg=PANEL_BG); b2.pack(fill='x')
        tk.Button(b2, text='Approach (15 cm acima)', bg='#37474f', fg=TEXT_MAIN,
                  activebackground='#546e7a', activeforeground=TEXT_MAIN,
                  relief='flat', padx=8, pady=5, font=('Arial', 9, 'bold'),
                  command=lambda: self._apply_arm(APPROACH_POSE_DEG)
                  ).pack(side='left', padx=3, fill='x', expand=True)
        b3 = tk.Frame(body, bg=PANEL_BG); b3.pack(fill='x', pady=(4, 2))
        pick_colors = {'Pick Frasco': COLOR_FRASCO,
                       'Pick Tubo':   COLOR_TUBO,
                       'Pick Ampola': COLOR_AMPOLA}
        for label, pose in PICK_POSES_DEG.items():
            tk.Button(b3, text=label, bg=pick_colors[label], fg='white',
                      activebackground='#212121', activeforeground='white',
                      relief='flat', padx=6, pady=5, font=('Arial', 9, 'bold'),
                      command=lambda p=pose: self._apply_arm(p)
                      ).pack(side='left', padx=3, fill='x', expand=True)

    # ──────────────────────────────────────────────────────────── MÃO
    def _build_hand_panel(self, parent):
        outer = tk.Frame(parent, bg=BG)
        outer.grid(row=0, column=1, sticky='nsew', padx=(5, 0))

        header = tk.Frame(outer, bg=HEADER_BG)
        header.pack(fill='x')
        tk.Label(header, text='✋  Mão COVVI', font=('Arial', 12, 'bold'),
                 bg=HEADER_BG, fg=ACCENT_HND, pady=7, padx=10
                 ).pack(side='left')
        tk.Label(header, text='0 = aberta  ·  200 = fechada',
                 font=('Arial', 9), bg=HEADER_BG, fg=TEXT_DIM
                 ).pack(side='right', padx=10)

        body = tk.Frame(outer, bg=PANEL_BG, padx=10, pady=8)
        body.pack(fill='both', expand=True)

        # ── Sliders ──────────────────────────────────────────────────
        self.hand_sliders, self.hand_labels = {}, {}
        for j in HAND_JOINTS:
            row = tk.Frame(body, bg=PANEL_BG)
            row.pack(fill='x', pady=3)
            tk.Label(row, text=j, font=('Arial', 10, 'bold'),
                     bg=PANEL_BG, fg=TEXT_MAIN, width=8, anchor='w'
                     ).pack(side='left')
            val = tk.Label(row, text='   0', font=('Courier', 10, 'bold'),
                            bg=PANEL_BG, fg=TEXT_VAL, width=6, anchor='e')
            val.pack(side='right')
            sl = tk.Scale(row, from_=0, to=200, resolution=1,
                           orient='horizontal', showvalue=False,
                           bg=PANEL_BG, fg=TEXT_DIM, troughcolor=TROUGH,
                           activebackground=ACCENT_HND, highlightthickness=0,
                           command=lambda v, lbl=val, jn=j: self._hand_changed(v, lbl, jn))
            sl.set(0)
            sl.pack(side='left', fill='x', expand=True)
            self.hand_sliders[j], self.hand_labels[j] = sl, val

        # ── Abrir / Fechar / Pinça ───────────────────────────────────
        b1 = tk.Frame(body, bg=PANEL_BG); b1.pack(fill='x', pady=(10, 2))
        for label, val in (('Abrir', 0), ('Pinça (50%)', 100), ('Fechar', 200)):
            tk.Button(b1, text=label, bg=BTN_PRESET, fg=TEXT_MAIN,
                      activebackground='#546e7a', activeforeground=TEXT_MAIN,
                      relief='flat', padx=10, pady=5, font=('Arial', 9),
                      command=lambda v=val: self._apply_hand_uniform(v)
                      ).pack(side='left', padx=3, fill='x', expand=True)

        # ── Preensões do projeto ──────────────────────────────────────
        tk.Label(body, text='—— Preensões do Projeto ——',
                 font=('Arial', 9, 'italic'), bg=PANEL_BG, fg=TEXT_DIM
                 ).pack(fill='x', pady=(8, 2))
        grip_colors = {
            'Palm Grip (frasco)':      COLOR_FRASCO,
            'Claw Grip (tubo)':        COLOR_TUBO,
            'Fingertip Grip (ampola)': COLOR_AMPOLA,
        }
        b2 = tk.Frame(body, bg=PANEL_BG); b2.pack(fill='x')
        for label, vals in HAND_GRIPS.items():
            if label == 'Open':
                continue
            tk.Button(b2, text=label, bg=grip_colors[label], fg='white',
                      activebackground='#212121', activeforeground='white',
                      relief='flat', padx=4, pady=5, font=('Arial', 8, 'bold'),
                      command=lambda v=vals, l=label: self._apply_hand(v, project_label=l)
                      ).pack(side='left', padx=2, fill='x', expand=True)

        # ── COVVI Built-in Grips (ECI) ────────────────────────────────
        tk.Label(body, text='—— COVVI Built-in Grips (ECI) ——',
                 font=('Arial', 9, 'italic'), bg=PANEL_BG, fg=TEXT_DIM
                 ).pack(fill='x', pady=(10, 2))

        grip_names = list(ECI_GRIP_IDS.keys())
        for row_names in (grip_names[:7], grip_names[7:]):
            row = tk.Frame(body, bg=PANEL_BG)
            row.pack(fill='x', pady=2)
            for name in row_names:
                clr = _ECI_CLR.get(name, BTN_PRESET)
                tk.Button(
                    row, text=name, bg=clr, fg='white',
                    activebackground='#212121', activeforeground='white',
                    relief='flat', padx=4, pady=5, font=('Arial', 8, 'bold'),
                    command=lambda n=name: self._apply_eci_grip(n),
                ).pack(side='left', padx=2, fill='x', expand=True)

        # ── ECI feedback panel ────────────────────────────────────────
        fb = tk.Frame(body, bg='#0d1117', relief='sunken', bd=1)
        fb.pack(fill='x', pady=(10, 0))
        tk.Label(fb, text='ECI feedback', font=('Arial', 8, 'bold'),
                 bg='#0d1117', fg=TEXT_DIM, anchor='w', padx=6
                 ).pack(fill='x')

        row_grip = tk.Frame(fb, bg='#0d1117'); row_grip.pack(fill='x', padx=6)
        tk.Label(row_grip, text='grip atual:', font=('Courier', 8),
                 bg='#0d1117', fg=TEXT_DIM, width=12, anchor='w'
                 ).pack(side='left')
        self._eci_grip_lbl = tk.Label(
            row_grip, text='—', font=('Courier', 8, 'bold'),
            bg='#0d1117', fg=TEXT_VAL, anchor='w')
        self._eci_grip_lbl.pack(side='left', fill='x', expand=True)

        row_sent = tk.Frame(fb, bg='#0d1117'); row_sent.pack(fill='x', padx=6)
        tk.Label(row_sent, text='último env.:', font=('Courier', 8),
                 bg='#0d1117', fg=TEXT_DIM, width=12, anchor='w'
                 ).pack(side='left')
        self._eci_sent_lbl = tk.Label(
            row_sent, text='—', font=('Courier', 8, 'bold'),
            bg='#0d1117', fg=ACCENT_HND, anchor='w')
        self._eci_sent_lbl.pack(side='left', fill='x', expand=True)

        row_posn = tk.Frame(fb, bg='#0d1117'); row_posn.pack(fill='x', padx=6, pady=(0, 4))
        tk.Label(row_posn, text='posições:', font=('Courier', 8),
                 bg='#0d1117', fg=TEXT_DIM, width=12, anchor='w'
                 ).pack(side='left')
        self._eci_posn_lbl = tk.Label(
            row_posn, text='T:- I:- M:- R:- L:- Rot:-',
            font=('Courier', 8), bg='#0d1117', fg=TEXT_VAL, anchor='w')
        self._eci_posn_lbl.pack(side='left', fill='x', expand=True)

    # ──────────────────────────────────────────────────────── HANDLERS
    def _arm_changed(self, val, lbl, _joint):
        lbl.config(text=f'{int(float(val)):4d}°')
        if not self._suppressing:
            self._publish_arm()

    def _hand_changed(self, val, lbl, _joint):
        lbl.config(text=f'{int(float(val)):4d}')
        if not self._suppressing:
            self._publish_hand()

    def _apply_arm(self, preset_deg: dict):
        positions_rad = [math.radians(preset_deg[j]) for j in ARM_JOINTS]
        self._publish_arm_positions(positions_rad)
        self.root.after_idle(self._update_arm_sliders, preset_deg)

    def _update_arm_sliders(self, preset_deg: dict):
        self._suppressing = True
        try:
            for j, deg in preset_deg.items():
                self.arm_sliders[j].set(deg)
                self.arm_labels[j].config(text=f'{int(deg):4d}°')
        finally:
            self._suppressing = False

    def _apply_hand_uniform(self, value: int):
        vals = {j: value for j in HAND_JOINTS}
        self._publish_hand_values(vals)
        self._schedule_eci_posn(vals)
        self.root.after_idle(self._update_hand_sliders, vals)

    def _apply_hand(self, vals: dict, project_label: str | None = None):
        """Apply a project grip (Gazebo + optionally ECI real hand)."""
        self._publish_hand_values(vals)
        self.root.after_idle(self._update_hand_sliders, dict(vals))
        if project_label and self._eci_enabled:
            eci_id = PROJECT_GRIP_ECI.get(project_label)
            if eci_id is not None:
                eci_name = ECI_GRIP_NAMES.get(eci_id, str(eci_id))
                self._send_eci_grip(eci_id, f'{project_label} → {eci_name}')

    def _apply_eci_grip(self, name: str):
        """Apply a COVVI built-in grip: Gazebo simulation + real hand via ECI."""
        gazebo_vals = ECI_GRIP_GAZEBO[name]
        self._publish_hand_values(gazebo_vals)
        self.root.after_idle(self._update_hand_sliders, dict(gazebo_vals))
        self._send_eci_grip(ECI_GRIP_IDS[name], name)
        self._set_status(f'ECI grip: {name}', _CLR=ACCENT_HND)

    def _update_hand_sliders(self, vals: dict):
        self._suppressing = True
        try:
            for j, v in vals.items():
                self.hand_sliders[j].set(v)
                self.hand_labels[j].config(text=f'{int(v):4d}')
        finally:
            self._suppressing = False

    def _apply_recipe(self, arm_key: str, grip_key: str):
        self._apply_arm(PICK_POSES_DEG[arm_key])
        self._set_status(f'⏳ Braço indo para {arm_key}...', _CLR='#fab387')
        duration_ms = int(float(self.time_sl.get()) * 1000) + 250

        def _close_hand_after_arm():
            self._pending_hand_after = None
            self._apply_hand(HAND_GRIPS[grip_key], project_label=grip_key)
            self._set_status(
                f'✓ Pick aplicado: {arm_key} + {grip_key}', _CLR=ACCENT_HND)

        if self._pending_hand_after is not None:
            try:
                self.root.after_cancel(self._pending_hand_after)
            except Exception:
                pass
        self._pending_hand_after = self.root.after(duration_ms, _close_hand_after_arm)

    def _do_delivery(self):
        obj = self._last_detection
        if obj is None or obj not in DELIVERY_POSES_DEG:
            self._set_status(
                '⚠ Nenhum objeto detectado — spawne um objeto antes de entregar.',
                _CLR='#f38ba8')
            return
        box_color = {'frasco': 'vermelha', 'tubo': 'verde', 'ampola': 'azul'}[obj]
        self._apply_arm(DELIVERY_POSES_DEG[obj])
        self._set_status(f'📦 Entrega: {obj} → caixa {box_color}', _CLR=ACCENT_HND)

    # ──────────────────────────────────────────────────────── PUBLISH
    def _publish_arm(self):
        if not self._ready:
            return
        positions = [math.radians(self.arm_sliders[j].get()) for j in ARM_JOINTS]
        self._publish_arm_positions(positions)

    def _publish_arm_positions(self, positions_rad: list):
        if not self._ready:
            return
        msg = JointTrajectory()
        msg.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = list(positions_rad)
        dur = float(self.time_sl.get())
        pt.time_from_start = Duration(sec=int(dur),
                                       nanosec=int((dur - int(dur)) * 1e9))
        msg.points.append(pt)
        self.arm_pub.publish(msg)

    def _publish_hand(self):
        if not self._ready:
            return
        vals = {j: self.hand_sliders[j].get() for j in HAND_JOINTS}
        self._publish_hand_values(vals)
        self._schedule_eci_posn(vals)

    def _publish_hand_values(self, vals: dict):
        if not self._ready:
            return
        primary_rad = {j: vals[j] / 200.0 * MAX_RAD[j] for j in HAND_JOINTS}
        names = list(HAND_JOINTS)
        positions = [primary_rad[j] for j in HAND_JOINTS]
        for mimic, driver, mult in MIMIC_JOINTS:
            names.append(mimic)
            positions.append(primary_rad[driver] * mult)

        msg = JointTrajectory()
        msg.joint_names = names
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(sec=0, nanosec=800_000_000)
        msg.points.append(pt)
        self.hand_pub.publish(msg)

    # ──────────────────────────────────────────────────────── STATUS
    def _set_status(self, msg: str, _CLR=TEXT_MAIN):
        self.status_var.set(msg)
        self.status_lbl.configure(fg=_CLR)

    def _spin_ros(self):
        try:
            rclpy.spin_once(self, timeout_sec=0.0)
        except Exception:
            return
        self.root.after(20, self._spin_ros)

    def _poll_status(self):
        try:
            while True:
                msg, color = self._status_q.get_nowait()
                self._set_status(msg, _CLR=color)
        except queue.Empty:
            pass

        # Conveyor state
        cs = self._conveyor_state
        if cs:
            obj = cs.get('current_obj', 'none')
            has = '✔' if cs.get('has_object') else '○'
            self.obj_lbl.config(
                text=f'{has}  esteira: {obj}',
                fg=ACCENT_HND if cs.get('has_object') else TEXT_DIM)

        # Entrega hint
        det = self._last_detection
        if hasattr(self, '_entrega_hint'):
            if det in DELIVERY_POSES_DEG:
                box_color = {'frasco': 'vermelha', 'tubo': 'verde', 'ampola': 'azul'}[det]
                color = {'frasco': COLOR_FRASCO, 'tubo': COLOR_TUBO,
                         'ampola': COLOR_AMPOLA}[det]
                self._entrega_hint.config(
                    text=f'→ {det} para caixa {box_color}', fg=color)
                self._btn_entrega.config(state='normal')
            else:
                self._entrega_hint.config(
                    text='(aguardando detecção da câmera)', fg=TEXT_DIM)
                self._btn_entrega.config(state='disabled')

        # ECI feedback panel
        if hasattr(self, '_eci_grip_lbl'):
            grip_id = self._eci_grip_id
            grip_name = ECI_GRIP_NAMES.get(grip_id, f'id={grip_id}') if grip_id else '—'
            self._eci_grip_lbl.config(text=grip_name)

            sent = self._eci_last_sent or '—'
            self._eci_sent_lbl.config(text=sent)

            p = self._eci_posn
            if p:
                posn_str = (
                    f"T:{p.get('Thumb','-'):3}  I:{p.get('Index','-'):3}  "
                    f"M:{p.get('Middle','-'):3}  R:{p.get('Ring','-'):3}  "
                    f"L:{p.get('Little','-'):3}  Rot:{p.get('Rotate','-'):3}"
                )
                self._eci_posn_lbl.config(text=posn_str)

        self.root.after(400, self._poll_status)


def main(args=None):
    # tk.Tk() MUST be called before rclpy.init() — rclpy.init() starts
    # FastDDS background threads whose C extensions corrupt Tcl's allocator
    # when Tcl is initialised afterwards, causing a segfault.
    root = tk.Tk()
    root.withdraw()

    rclpy.init(args=args)
    node = ManualControlNode(root=root)
    try:
        root.mainloop()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
