"""
Controle manual da célula de manufatura — CR10 + COVVI + Esteira.

GUI Tkinter que combina:
  • Sliders por junta do braço (6 sliders, em graus)
  • Sliders por junta da mão  (6 sliders 0..200, mapeados para rad)
  • Botões de pose pré-calculada do braço (Pick Frasco/Tubo/Ampola)
  • Botões de preensão da mão (Palm/Claw/Fingertip Grip)
  • Controles da esteira (Próximo Objeto / Anterior / Reset)

Diferente do `gui_control` automático (que dispara o ciclo `/cell/execute_grasp`
de uma vez só), este nó publica DIRETAMENTE nas trajetórias de junta — o
operador alinha visualmente o braço sobre o objeto, fecha a mão na preensão
correta e levanta, junta por junta.

Uso:
  ros2 launch grasp_ml_pack conveyor_cell.launch.py no_gui:=true   # sem a GUI auto
  ros2 run   grasp_ml_pack manual_control                          # GUI manual

(Pode rodar lado-a-lado com o `gui_control` automático também — os tópicos
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

# Poses de pick (graus) — calculadas via IK do projeto para os 3 objetos
# na estação de pick (x=0.75, y=0). Convergem para o mesmo ramo IK
# (cotovelo abaixado, alcance frontal).
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

# Pose intermediária acima da estação (Approach 15 cm) — gera transição
# segura quando o braço está em HOME. Convergem todos para mesmo branch.
APPROACH_POSE_DEG = {
    'joint1': +23.5, 'joint2': -16.5, 'joint3': -82.0,
    'joint4': -81.0, 'joint5': +23.5, 'joint6':  +0.0,
}

# Poses de entrega — TCP a 15 cm acima da abertura da caixa correspondente.
# Calculadas via IK para approach_box em world frame:
#   box1 (vermelha, frasco) → (-0.05, 0.65, 0.75)
#   box2 (verde,    tubo)   → ( 0.25, 0.65, 0.75)
#   box3 (azul,     ampola) → ( 0.55, 0.65, 0.75)
# frasco e tubo convergem para o mesmo ramo (cotovelo abaixado); ampola usa
# um ramo distinto (cotovelo retraído) devido à geometria de alcance — ambas
# soluções têm `_arm_clears_world=True` e erro de IK < 2 mm.
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

# Configurações de preensão (valor 0..200 de cada slider). Idênticas às
# HAND_CONFIGS do kinematics.py do executor automático.
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

# Associação didática objeto → preset de braço + preset de mão.
# Permite executar o pick passo-a-passo: HOME → APPROACH → PICK → GRIP.
OBJ_RECIPE = {
    'frasco': ('Pick Frasco', 'Palm Grip (frasco)'),
    'tubo':   ('Pick Tubo',   'Claw Grip (tubo)'),
    'ampola': ('Pick Ampola', 'Fingertip Grip (ampola)'),
}

# Paleta — alinhada com o tema escuro do projeto
BG, PANEL_BG, HEADER_BG = '#1a1a2e', '#16213e', '#0f3460'
ACCENT_ARM, ACCENT_HND  = '#4fc3f7', '#69f0ae'
TEXT_MAIN, TEXT_DIM, TEXT_VAL = '#e0e0e0', '#9e9e9e', '#ffd54f'
BTN_PRESET, TROUGH       = '#37474f', '#2a2a4a'
BTN_CONV_FWD, BTN_CONV_RV, BTN_CONV_RST = '#388e3c', '#c62828', '#5d4037'
COLOR_FRASCO, COLOR_TUBO, COLOR_AMPOLA = '#e65100', '#1565c0', '#2e7d32'


class ManualControlNode(Node):
    """Nó ROS 2 + GUI Tkinter para controle manual da célula."""

    def __init__(self):
        super().__init__('manual_control')
        self._ready = False
        # Flag para suprimir publishes durante atualizações em lote de
        # sliders (substitui o antigo padrão de "mute lambda" que causava
        # `Tcl_Release couldn't find reference` por double-delete de
        # comandos Tcl quando o GC liberava lambdas já desregistradas).
        self._suppressing = False
        # Token do `after` pendente para fechar a mão depois do braço —
        # evita empilhar fechamentos se o usuário clica várias vezes.
        self._pending_hand_after: str | None = None
        cb = ReentrantCallbackGroup()

        # Publishers de trajetória (chega aos joint_trajectory_controllers)
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/cr10_group_controller/joint_trajectory', 10)
        self.hand_pub = self.create_publisher(
            JointTrajectory, '/hand_position_controller/joint_trajectory', 10)

        # Clientes de serviço (esteira). Cada objeto tem seu próprio spawn
        # dedicado: o operador escolhe diretamente qual peça aparece na pick
        # station, em vez de avançar ciclicamente.
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

        # Estado da esteira + última detecção da câmera
        self._conveyor_state: dict = {}
        self._last_detection: str | None = None
        # Fila thread-safe: callbacks ROS (rodando na thread do rclpy executor)
        # NUNCA podem tocar widgets Tk diretamente nem chamar `root.after()` —
        # o Tcl não é thread-safe e gera `Tcl_Release couldn't find reference`
        # seguido de SIGABRT. Em vez disso, callbacks empilham mensagens aqui
        # e o `_poll_status` (que roda na thread do Tk) drena a fila.
        self._status_q: queue.Queue = queue.Queue()

        self.create_subscription(
            String, '/conveyor/status', self._cb_conveyor, 10,
            callback_group=cb)
        self.create_subscription(
            Detection2DArray, '/detected_objects', self._cb_detection, 10,
            callback_group=cb)

        self._build_ui()
        self._ready = True
        # Loop ROS executado dentro da thread do Tk via spin_once periódico.
        # Garante que TODOS os callbacks (subs, futures de service) rodem na
        # mesma thread que mexe nos widgets — elimina o `Tcl_Release` que
        # ocorre quando o rclpy executor em outra thread mexe em estado
        # compartilhado com o Tcl.
        self._spin_ros()
        self._poll_status()

    # ──────────────────────────────────────────────────────────────────
    def _cb_conveyor(self, msg: String):
        try:
            self._conveyor_state = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _cb_detection(self, msg: Detection2DArray):
        """Armazena a classe do objeto detectado pela câmera (visão HSV/YOLO)."""
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
            # Roda na thread do rclpy — NÃO toca em widgets Tk. Empurra o
            # resultado para a fila; _poll_status drena na thread certa.
            try:
                res = f.result()
                color = ACCENT_HND if res.success else '#f38ba8'
                self._status_q.put((res.message, color))
            except Exception as exc:
                self._status_q.put((str(exc), '#f38ba8'))

        future.add_done_callback(_done)

    # ──────────────────────────────────────────────────────────────────
    def _build_ui(self):
        self.root = tk.Tk()
        self.root.title('Célula de Manufatura — Controle Manual')
        self.root.configure(bg=BG)
        self.root.minsize(960, 640)

        style = ttk.Style()
        style.theme_use('clam')

        # Cabeçalho
        hdr = tk.Frame(self.root, bg=HEADER_BG)
        hdr.pack(fill='x')
        tk.Label(hdr, text='CR10 + COVVI  —  Modo Manual da Célula',
                 font=('Arial', 14, 'bold'), bg=HEADER_BG, fg=TEXT_MAIN,
                 pady=10).pack(side='left', padx=18)
        self.obj_lbl = tk.Label(
            hdr, text='○ esteira vazia',
            font=('Courier', 11, 'bold'), bg=HEADER_BG, fg=TEXT_DIM)
        self.obj_lbl.pack(side='right', padx=16)

        # Barra da esteira — escolha direta do objeto (botões coloridos
        # por classe, mesma palette dos bounding boxes da visão)
        bar = tk.Frame(self.root, bg=BG)
        bar.pack(fill='x', padx=12, pady=8)

        conv = tk.Frame(bar, bg=BG)
        conv.pack(side='left')
        tk.Label(conv, text='Spawn: ', font=('Arial', 10),
                 bg=BG, fg=TEXT_DIM).pack(side='left', padx=(0, 6))
        spawn_specs = [
            ('Frasco', COLOR_FRASCO, 'spawn_frasco'),
            ('Tubo',   COLOR_TUBO,   'spawn_tubo'),
            ('Ampola', COLOR_AMPOLA, 'spawn_ampola'),
        ]
        for label, color, srv_key in spawn_specs:
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

        # Atalhos por objeto (recipe pick = pose do braço + preensão).
        # A mão SÓ fecha depois que a trajetória do braço termina (espera
        # `duration_slider` + 250 ms) — garante que os dedos só se fecham
        # quando a palma já está sobre o objeto.
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

        # Botão ENTREGA — lê a classe detectada pela câmera (/detected_objects)
        # e leva o braço à caixa correspondente (frasco→vermelha, tubo→verde,
        # ampola→azul). A mão NÃO é tocada — o operador deve abrir a mão
        # manualmente após o braço chegar para soltar o objeto na caixa.
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

        # Colunas — Braço | Mão
        cols = tk.Frame(self.root, bg=BG)
        cols.pack(fill='both', expand=True, padx=10, pady=(0, 6))
        cols.columnconfigure(0, weight=1)
        cols.columnconfigure(1, weight=1)
        cols.rowconfigure(0, weight=1)
        self._build_arm_panel(cols)
        self._build_hand_panel(cols)

        # Status bar
        self.status_var = tk.StringVar(value='Pronto.')
        sb = tk.Frame(self.root, bg=PANEL_BG)
        sb.pack(fill='x')
        self.status_lbl = tk.Label(sb, textvariable=self.status_var,
                                    font=('Arial', 10), bg=PANEL_BG,
                                    fg=TEXT_MAIN, anchor='w', padx=10, pady=6)
        self.status_lbl.pack(fill='x')

    # ────────────────────────────────────────────── PAINEL BRAÇO ─
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

        # Botões de pose básica
        b1 = tk.Frame(body, bg=PANEL_BG); b1.pack(fill='x', pady=(10, 2))
        for label, preset in ARM_PRESETS_BASE.items():
            tk.Button(b1, text=label, bg=BTN_PRESET, fg=TEXT_MAIN,
                      activebackground='#546e7a', activeforeground=TEXT_MAIN,
                      relief='flat', padx=10, pady=5, font=('Arial', 9),
                      command=lambda p=preset: self._apply_arm(p)
                      ).pack(side='left', padx=3, fill='x', expand=True)

        # Approach + poses de pick
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

    # ────────────────────────────────────────────── PAINEL MÃO ─
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

        # Abrir / Fechar / Pinça
        b1 = tk.Frame(body, bg=PANEL_BG); b1.pack(fill='x', pady=(10, 2))
        for label, val in (('Abrir', 0), ('Pinça (50%)', 100), ('Fechar', 200)):
            tk.Button(b1, text=label, bg=BTN_PRESET, fg=TEXT_MAIN,
                      activebackground='#546e7a', activeforeground=TEXT_MAIN,
                      relief='flat', padx=10, pady=5, font=('Arial', 9),
                      command=lambda v=val: self._apply_hand_uniform(v)
                      ).pack(side='left', padx=3, fill='x', expand=True)

        # Preensões do projeto
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
                      command=lambda v=vals: self._apply_hand(v)
                      ).pack(side='left', padx=2, fill='x', expand=True)

    # ────────────────────────────────────────────── HANDLERS ─
    def _arm_changed(self, val, lbl, _joint):
        lbl.config(text=f'{int(float(val)):4d}°')
        if not self._suppressing:
            self._publish_arm()

    def _hand_changed(self, val, lbl, _joint):
        lbl.config(text=f'{int(float(val)):4d}')
        if not self._suppressing:
            self._publish_hand()

    def _apply_arm(self, preset_deg: dict):
        """Aplica pose ao braço.

        IMPORTANTE: publica a trajetória ANTES de mexer nos sliders, lendo
        os valores do próprio `preset_deg`. Tocar nos sliders dentro de um
        botão Tk + chamar `publish` (que entra no DDS, libera o GIL e pode
        rodar GC) na mesma callstack vinha causando `Tcl_Release couldn't
        find reference …` → SIGABRT. Desacoplar resolve.
        """
        positions_rad = [math.radians(preset_deg[j]) for j in ARM_JOINTS]
        self._publish_arm_positions(positions_rad)
        # Atualização visual diferida — roda quando o Tk estiver ocioso,
        # depois do publish ter concluído.
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
        self.root.after_idle(self._update_hand_sliders, vals)

    def _apply_hand(self, vals: dict):
        self._publish_hand_values(vals)
        self.root.after_idle(self._update_hand_sliders, dict(vals))

    def _update_hand_sliders(self, vals: dict):
        self._suppressing = True
        try:
            for j, v in vals.items():
                self.hand_sliders[j].set(v)
                self.hand_labels[j].config(text=f'{int(v):4d}')
        finally:
            self._suppressing = False

    def _apply_recipe(self, arm_key: str, grip_key: str):
        """
        Atalho pick: aplica pose do braço, ESPERA o braço chegar à posição
        e SÓ ENTÃO fecha a mão na preensão correspondente.

        A espera é `duration_slider + 250 ms` (margem para a trajetória ser
        recebida e completada pelo controlador). Sem essa sincronização, a
        mão fechava no meio do caminho e o objeto não era pego.
        """
        # 1. Despacha o braço imediatamente
        self._apply_arm(PICK_POSES_DEG[arm_key])
        self._set_status(f'⏳ Braço indo para {arm_key}...', _CLR='#fab387')

        # 2. Programa o fechamento da mão para depois do braço terminar
        duration_ms = int(float(self.time_sl.get()) * 1000) + 250

        def _close_hand_after_arm():
            self._pending_hand_after = None
            self._apply_hand(HAND_GRIPS[grip_key])
            self._set_status(
                f'✓ Pick aplicado: {arm_key} + {grip_key}', _CLR=ACCENT_HND)

        # Cancela fechamento pendente (clique anterior) para não sobrepor
        if self._pending_hand_after is not None:
            try:
                self.root.after_cancel(self._pending_hand_after)
            except Exception:
                pass
        self._pending_hand_after = self.root.after(
            duration_ms, _close_hand_after_arm)

    def _do_delivery(self):
        """
        Entrega — move o braço para a caixa correspondente ao objeto
        atualmente detectado pela câmera (`/detected_objects`):
          frasco → caixa vermelha  (box1, x=-0.05)
          tubo   → caixa verde     (box2, x= 0.25)
          ampola → caixa azul      (box3, x= 0.55)

        A mão NÃO é movida: o operador decide quando abrir a mão para
        soltar o objeto na caixa (botão "Abrir" do painel da mão).
        """
        obj = self._last_detection
        if obj is None or obj not in DELIVERY_POSES_DEG:
            self._set_status(
                '⚠ Nenhum objeto detectado pela câmera — '
                'spawne um objeto antes de entregar.',
                _CLR='#f38ba8')
            return
        box_color = {'frasco':'vermelha','tubo':'verde','ampola':'azul'}[obj]
        self._apply_arm(DELIVERY_POSES_DEG[obj])
        self._set_status(
            f'📦 Entrega: {obj} → caixa {box_color}', _CLR=ACCENT_HND)

    # ────────────────────────────────────────────── PUBLISH ─
    def _publish_arm(self):
        """Publica posição corrente dos sliders (callback de slider arrastado)."""
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
        # Mão é rápida: 0.8 s é suficiente, evita pausa entre dedos
        pt.time_from_start = Duration(sec=0, nanosec=800_000_000)
        msg.points.append(pt)
        self.hand_pub.publish(msg)

    # ────────────────────────────────────────────── STATUS ─
    def _set_status(self, msg: str, _CLR=TEXT_MAIN):
        self.status_var.set(msg)
        self.status_lbl.configure(fg=_CLR)

    def _spin_ros(self):
        """Tick ROS rodando NA THREAD DO TK (via root.after). Drena callbacks
        de subs/services num único passo e reagenda. Mantém todo o trabalho
        ROS+Tk em uma única thread → sem corrupção do interpretador Tcl."""
        try:
            rclpy.spin_once(self, timeout_sec=0.0)
        except Exception:
            # rclpy pode estar em shutdown — não derruba a UI
            return
        self.root.after(20, self._spin_ros)   # ~50 Hz

    def _poll_status(self):
        # Drena resultados de chamadas de serviço enfileiradas por callbacks
        # do rclpy (que agora rodam na mesma thread, mas mantemos a fila para
        # desacoplar a ordem de execução).
        try:
            while True:
                msg, color = self._status_q.get_nowait()
                self._set_status(msg, _CLR=color)
        except queue.Empty:
            pass

        cs = self._conveyor_state
        if cs:
            obj  = cs.get('current_obj', 'none')
            has  = '✔' if cs.get('has_object') else '○'
            self.obj_lbl.config(
                text=f'{has}  esteira: {obj}',
                fg=ACCENT_HND if cs.get('has_object') else TEXT_DIM)

        # Atualiza o hint do botão ENTREGA com a detecção atual da câmera
        det = self._last_detection
        if hasattr(self, '_entrega_hint'):
            if det in DELIVERY_POSES_DEG:
                box_color = {'frasco':'vermelha','tubo':'verde','ampola':'azul'}[det]
                color = {'frasco':COLOR_FRASCO,'tubo':COLOR_TUBO,'ampola':COLOR_AMPOLA}[det]
                self._entrega_hint.config(
                    text=f'→ {det} para caixa {box_color}', fg=color)
                self._btn_entrega.config(state='normal')
            else:
                self._entrega_hint.config(
                    text='(aguardando detecção da câmera)', fg=TEXT_DIM)
                self._btn_entrega.config(state='disabled')
        self.root.after(400, self._poll_status)


def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()
    try:
        # Tk mainloop é o loop principal; ROS spin_once é chamado dentro
        # dele a cada 20 ms via root.after — tudo em uma thread só.
        node.root.mainloop()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
