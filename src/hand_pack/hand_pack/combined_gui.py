import math
import threading
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tkinter as tk
from tkinter import ttk
from tkinter import font as tkfont

# ---------- Hand constants (mimic joint multipliers from URDF) ----------
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

MAX_RAD = {
    'Thumb': 1.6, 'Index': 1.6, 'Middle': 1.6,
    'Ring':  1.6, 'Little': 1.6, 'Rotate': 1.0,
}

HAND_JOINTS = ['Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Rotate']
ARM_JOINTS   = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

ARM_LIMITS_DEG = {
    'joint1': (-360, 360),
    'joint2': (-360, 360),
    'joint3': (-164, 164),
    'joint4': (-360, 360),
    'joint5': (-360, 360),
    'joint6': (-360, 360),
}

# Arm preset poses  {label: {joint: degrees}}
ARM_PRESETS = {
    'Home':     {j: 0   for j in ARM_JOINTS},
    'Vertical': {'joint1': 0, 'joint2': -90, 'joint3': 0,
                 'joint4': 0, 'joint5': 90, 'joint6': 0},
    'Estendido': {'joint1': 0, 'joint2': 0, 'joint3': -90,
                  'joint4': 0, 'joint5': 0, 'joint6': 0},
}

# Poses de pick — ângulos em graus calculados via IK do grasp_ml_pack
# para os 3 objetos farmacêuticos na estação de pick (x=0.75, y=0).
# Permitem verificar visualmente o alinhamento da mão sobre o objeto antes
# de rodar o ciclo automático da célula. As três soluções convergem para o
# mesmo ramo (q3 ≈ -83°, "elbow down, reaching forward").
PICK_POSES = {
    'Pick Frasco (z=0.866)': {
        'joint1': +23.5, 'joint2': -16.6, 'joint3': -83.2,
        'joint4': -80.2, 'joint5': +23.5, 'joint6':  +0.0,
    },
    'Pick Tubo (z=0.896)': {
        'joint1': +23.5, 'joint2': -16.2, 'joint3': -80.7,
        'joint4': -83.1, 'joint5': +23.5, 'joint6':  +0.0,
    },
    'Pick Ampola (z=0.851)': {
        'joint1': +23.6, 'joint2': -16.8, 'joint3': -84.5,
        'joint4': -78.7, 'joint5': +23.3, 'joint6':  +0.2,
    },
}

# Configurações de grasp da mão COVVI (extraído de kinematics.HAND_CONFIGS),
# em valor de slider 0..200 (0=aberto, 200=fechado em max_rad).
# Mapeamento: cfg_rad_j → slider_j = round(cfg_rad_j / MAX_RAD[j] * 200)
#   palm_grip      → frasco (preensão palmar, garrafa)
#   claw_grip      → tubo de ensaio (garra fina)
#   fingertip_grip → ampola (pinça de pontas)
# Valores derivados:
#   palm_grip   = {Thumb:1.10,Index:1.25,Middle:1.25,Ring:1.20,Little:1.10,Rotate:0.25}
#               = {138, 156, 156, 150, 138, 50}
#   claw_grip   = {Thumb:0.90,Index:1.10,Middle:1.10,Ring:1.05,Little:0.95,Rotate:0.45}
#               = {112, 138, 138, 131, 119, 90}
#   fingertip   = {Thumb:0.75,Index:0.70,Middle:0.65,Ring:0.05,Little:0.05,Rotate:0.82}
#               = {94, 88, 81, 6, 6, 164}
HAND_GRIPS = {
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

# Colour palette
BG         = '#1a1a2e'
PANEL_BG   = '#16213e'
HEADER_BG  = '#0f3460'
ACCENT_ARM = '#4fc3f7'
ACCENT_HND = '#69f0ae'
TEXT_MAIN  = '#e0e0e0'
TEXT_DIM   = '#9e9e9e'
TEXT_VAL   = '#ffd54f'
BTN_ARM    = '#1565c0'
BTN_HND_O  = '#2e7d32'
BTN_HND_C  = '#c62828'
BTN_PRESET = '#37474f'
TROUGH     = '#2a2a4a'
SLIDER_ARM = ACCENT_ARM
SLIDER_HND = ACCENT_HND


class CombinedControlGUI(Node):
    def __init__(self):
        super().__init__('combined_control_gui')
        self._ready = False  # blocks publishing until UI is fully initialised
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/cr10_group_controller/joint_trajectory', 10)
        self.hand_pub = self.create_publisher(
            JointTrajectory, '/hand_position_controller/joint_trajectory', 10)
        self._build_ui()
        self._ready = True  # UI ready — user interactions may now publish

    # ------------------------------------------------------------------ UI --
    def _build_ui(self):
        self.root = tk.Tk()
        self.root.title('CR10 + COVVI  —  Controle de Simulação')
        self.root.configure(bg=BG)
        self.root.resizable(True, True)
        self.root.minsize(820, 540)

        self._setup_styles()
        self._build_header()
        self._build_duration_bar()

        cols = tk.Frame(self.root, bg=BG)
        cols.pack(fill='both', expand=True, padx=10, pady=(0, 6))
        cols.columnconfigure(0, weight=1)
        cols.columnconfigure(1, weight=1)
        cols.rowconfigure(0, weight=1)

        self._build_arm_panel(cols)
        self._build_hand_panel(cols)
        self._build_statusbar()

    def _setup_styles(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('Separator.TSeparator', background=HEADER_BG)

    def _build_header(self):
        hdr = tk.Frame(self.root, bg=HEADER_BG)
        hdr.pack(fill='x')
        tk.Label(
            hdr,
            text='CR10 + COVVI Hand  —  Simulação Gazebo',
            font=('Arial', 15, 'bold'),
            bg=HEADER_BG, fg=TEXT_MAIN,
            pady=10,
        ).pack(side='left', padx=18)
        tk.Label(
            hdr,
            text='● Controle Manual',
            font=('Arial', 9),
            bg=HEADER_BG, fg=ACCENT_HND,
        ).pack(side='right', padx=16)

    def _build_duration_bar(self):
        bar = tk.Frame(self.root, bg=BG)
        bar.pack(fill='x', padx=12, pady=(8, 2))

        tk.Label(
            bar, text='Duração do Movimento:',
            font=('Arial', 10), bg=BG, fg=TEXT_DIM,
            width=22, anchor='w',
        ).pack(side='left')

        self.dur_val_lbl = tk.Label(
            bar, text='2.0 s',
            font=('Courier', 10, 'bold'), bg=BG, fg=TEXT_VAL, width=6,
        )
        self.dur_val_lbl.pack(side='right')

        self.time_sl = tk.Scale(
            bar, from_=0.1, to=10.0, resolution=0.1,
            orient='horizontal', showvalue=False,
            bg=BG, fg=TEXT_DIM,
            troughcolor=TROUGH, activebackground=ACCENT_ARM,
            highlightthickness=0,
            command=self._dur_changed,
        )
        self.time_sl.set(2.0)
        self.time_sl.pack(side='left', fill='x', expand=True, padx=(4, 6))

        ttk.Separator(self.root, orient='horizontal').pack(fill='x', padx=8, pady=4)

    def _dur_changed(self, val):
        self.dur_val_lbl.config(text=f'{float(val):.1f} s')

    # ----------------------------------------------------------------- ARM --
    def _build_arm_panel(self, parent):
        outer = tk.Frame(parent, bg=BG)
        outer.grid(row=0, column=0, sticky='nsew', padx=(0, 5))

        header = tk.Frame(outer, bg=HEADER_BG)
        header.pack(fill='x')
        tk.Label(
            header, text='⚙  Braço CR10',
            font=('Arial', 12, 'bold'), bg=HEADER_BG, fg=ACCENT_ARM,
            pady=7, padx=10,
        ).pack(side='left')
        tk.Label(
            header, text='graus por junta',
            font=('Arial', 9), bg=HEADER_BG, fg=TEXT_DIM,
        ).pack(side='right', padx=10)

        body = tk.Frame(outer, bg=PANEL_BG, padx=10, pady=8)
        body.pack(fill='both', expand=True)

        self.arm_sliders = {}
        self.arm_labels  = {}

        for i, j in enumerate(ARM_JOINTS):
            lo, hi = ARM_LIMITS_DEG[j]
            row = tk.Frame(body, bg=PANEL_BG)
            row.pack(fill='x', pady=3)

            tk.Label(
                row, text=j, font=('Arial', 10, 'bold'),
                bg=PANEL_BG, fg=TEXT_MAIN, width=9, anchor='w',
            ).pack(side='left')

            val_lbl = tk.Label(
                row, text='    0°',
                font=('Courier', 10, 'bold'),
                bg=PANEL_BG, fg=TEXT_VAL, width=8, anchor='e',
            )
            val_lbl.pack(side='right')

            sl = tk.Scale(
                row, from_=lo, to=hi, resolution=1,
                orient='horizontal', showvalue=False,
                bg=PANEL_BG, fg=TEXT_DIM,
                troughcolor=TROUGH, activebackground=SLIDER_ARM,
                highlightthickness=0,
                command=lambda v, lbl=val_lbl, jn=j: self._arm_changed(v, lbl, jn),
            )
            sl.set(0)
            sl.pack(side='left', fill='x', expand=True)
            self.arm_sliders[j] = sl
            self.arm_labels[j]  = val_lbl

        # Botões de poses básicas
        btn_row = tk.Frame(body, bg=PANEL_BG)
        btn_row.pack(fill='x', pady=(10, 2))
        for label, preset in ARM_PRESETS.items():
            tk.Button(
                btn_row, text=label,
                bg=BTN_PRESET, fg=TEXT_MAIN,
                activebackground='#546e7a', activeforeground=TEXT_MAIN,
                relief='flat', padx=10, pady=5,
                font=('Arial', 9),
                command=lambda p=preset: self._arm_apply_preset(p),
            ).pack(side='left', padx=3)

        # Poses de pick — verifica alinhamento antes de operar a célula.
        # Aplica os ângulos via IK e publica imediatamente para movimento fluido.
        sep = tk.Label(body, text='—— Poses de Pick ——',
                       font=('Arial', 9, 'italic'),
                       bg=PANEL_BG, fg=TEXT_DIM)
        sep.pack(fill='x', pady=(8, 2))
        pick_row = tk.Frame(body, bg=PANEL_BG)
        pick_row.pack(fill='x', pady=(0, 2))
        for label, pose in PICK_POSES.items():
            tk.Button(
                pick_row, text=label,
                bg='#5d4037', fg='white',
                activebackground='#795548', activeforeground='white',
                relief='flat', padx=8, pady=5,
                font=('Arial', 9, 'bold'),
                command=lambda p=pose: self._arm_apply_preset(p),
            ).pack(side='left', padx=3, fill='x', expand=True)

    def _arm_changed(self, val, lbl, _joint):
        lbl.config(text=f'{int(float(val)):5d}°')
        self._publish_arm()

    def _arm_apply_preset(self, preset):
        # Atualiza os sliders SEM disparar callbacks individuais (que poderiam
        # publicar 6 trajetórias separadas e gerar trancos). Depois publica
        # uma única trajetória com todas as juntas → movimento suave e fluido.
        for j, deg in preset.items():
            sl = self.arm_sliders[j]
            sl.config(command=lambda v: None)   # mute callback
            sl.set(deg)
            self.arm_labels[j].config(text=f'{int(deg):5d}°')
        # Reata callbacks normais
        for j in preset.keys():
            sl = self.arm_sliders[j]
            lbl = self.arm_labels[j]
            sl.config(command=lambda v, lbl=lbl, jn=j: self._arm_changed(v, lbl, jn))
        # Publica trajetória única
        self._publish_arm()

    def _publish_arm(self):
        if not self._ready:
            return
        msg = JointTrajectory()
        msg.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [
            math.radians(self.arm_sliders[j].get()) for j in ARM_JOINTS]
        dur = self.time_sl.get()
        pt.time_from_start = Duration(sec=int(dur),
                                      nanosec=int((dur % 1) * 1e9))
        msg.points.append(pt)
        self.arm_pub.publish(msg)

    # ---------------------------------------------------------------- HAND --
    def _build_hand_panel(self, parent):
        outer = tk.Frame(parent, bg=BG)
        outer.grid(row=0, column=1, sticky='nsew', padx=(5, 0))

        header = tk.Frame(outer, bg=HEADER_BG)
        header.pack(fill='x')
        tk.Label(
            header, text='✋  Mão COVVI',
            font=('Arial', 12, 'bold'), bg=HEADER_BG, fg=ACCENT_HND,
            pady=7, padx=10,
        ).pack(side='left')
        tk.Label(
            header, text='0 = aberta  ·  200 = fechada',
            font=('Arial', 9), bg=HEADER_BG, fg=TEXT_DIM,
        ).pack(side='right', padx=10)

        body = tk.Frame(outer, bg=PANEL_BG, padx=10, pady=8)
        body.pack(fill='both', expand=True)

        self.hand_sliders = {}
        self.hand_labels  = {}

        for j in HAND_JOINTS:
            row = tk.Frame(body, bg=PANEL_BG)
            row.pack(fill='x', pady=3)

            tk.Label(
                row, text=j, font=('Arial', 10, 'bold'),
                bg=PANEL_BG, fg=TEXT_MAIN, width=8, anchor='w',
            ).pack(side='left')

            val_lbl = tk.Label(
                row, text='   0',
                font=('Courier', 10, 'bold'),
                bg=PANEL_BG, fg=TEXT_VAL, width=6, anchor='e',
            )
            val_lbl.pack(side='right')

            sl = tk.Scale(
                row, from_=0, to=200, resolution=1,
                orient='horizontal', showvalue=False,
                bg=PANEL_BG, fg=TEXT_DIM,
                troughcolor=TROUGH, activebackground=SLIDER_HND,
                highlightthickness=0,
                command=lambda v, lbl=val_lbl, jn=j: self._hand_changed(v, lbl, jn),
            )
            sl.set(0)
            sl.pack(side='left', fill='x', expand=True)
            self.hand_sliders[j] = sl
            self.hand_labels[j]  = val_lbl

        btn_row = tk.Frame(body, bg=PANEL_BG)
        btn_row.pack(fill='x', pady=(10, 2))
        tk.Button(
            btn_row, text='Abrir Tudo',
            bg=BTN_HND_O, fg='white',
            activebackground='#388e3c', activeforeground='white',
            relief='flat', padx=12, pady=5, font=('Arial', 9),
            command=lambda: self._hand_preset(0),
        ).pack(side='left', padx=3)
        tk.Button(
            btn_row, text='Fechar Tudo',
            bg=BTN_HND_C, fg='white',
            activebackground='#d32f2f', activeforeground='white',
            relief='flat', padx=12, pady=5, font=('Arial', 9),
            command=lambda: self._hand_preset(200),
        ).pack(side='left', padx=3)
        tk.Button(
            btn_row, text='Pinça (50%)',
            bg=BTN_PRESET, fg=TEXT_MAIN,
            activebackground='#546e7a', activeforeground=TEXT_MAIN,
            relief='flat', padx=12, pady=5, font=('Arial', 9),
            command=lambda: self._hand_preset(100),
        ).pack(side='left', padx=3)

        # Configurações de preensão para os 3 objetos farmacêuticos.
        # Aplica HAND_CONFIGS do projeto, idêntico ao que o executor automático
        # usa em ciclo. Botões coloridos por classe (laranja/azul/verde =
        # mesma palette dos bounding boxes da detecção).
        sep = tk.Label(body, text='—— Preensões do Projeto ——',
                       font=('Arial', 9, 'italic'),
                       bg=PANEL_BG, fg=TEXT_DIM)
        sep.pack(fill='x', pady=(8, 2))
        grip_row = tk.Frame(body, bg=PANEL_BG)
        grip_row.pack(fill='x', pady=(0, 2))
        grip_colors = {
            'Palm Grip (frasco)':       '#e65100',   # laranja
            'Claw Grip (tubo)':         '#1565c0',   # azul
            'Fingertip Grip (ampola)':  '#2e7d32',   # verde
        }
        for label, vals in HAND_GRIPS.items():
            tk.Button(
                grip_row, text=label,
                bg=grip_colors[label], fg='white',
                activebackground='#212121', activeforeground='white',
                relief='flat', padx=4, pady=5,
                font=('Arial', 8, 'bold'),
                command=lambda v=vals: self._hand_apply_grip(v),
            ).pack(side='left', padx=2, fill='x', expand=True)

    def _hand_changed(self, val, lbl, _joint):
        lbl.config(text=f'{int(float(val)):4d}')
        self._publish_hand()

    def _hand_preset(self, value):
        for sl in self.hand_sliders.values():
            sl.set(value)

    def _hand_apply_grip(self, vals: dict):
        """
        Aplica configuração de preensão (palm/claw/fingertip) em todos os
        sliders simultaneamente, com uma única publicação no controlador da
        mão — movimento contínuo e fluido (sem 6 trajetórias separadas).
        """
        for j, v in vals.items():
            sl = self.hand_sliders[j]
            sl.config(command=lambda val: None)         # mute callback
            sl.set(v)
            self.hand_labels[j].config(text=f'{int(v):4d}')
        # Reata callbacks
        for j in vals.keys():
            sl = self.hand_sliders[j]
            lbl = self.hand_labels[j]
            sl.config(command=lambda v, lbl=lbl, jn=j: self._hand_changed(v, lbl, jn))
        # Publica trajetória única
        self._publish_hand()

    def _publish_hand(self):
        if not self._ready:
            return
        vals  = {j: self.hand_sliders[j].get() for j in HAND_JOINTS}
        names = list(HAND_JOINTS)
        positions = [vals[j] / 200.0 * MAX_RAD[j] for j in HAND_JOINTS]
        for mimic, driver, mult in MIMIC_JOINTS:
            names.append(mimic)
            positions.append(vals[driver] / 200.0 * MAX_RAD[driver] * mult)
        msg = JointTrajectory()
        msg.joint_names = names
        pt = JointTrajectoryPoint()
        pt.positions = positions
        dur = self.time_sl.get()
        pt.time_from_start = Duration(sec=int(dur),
                                      nanosec=int((dur % 1) * 1e9))
        msg.points.append(pt)
        self.hand_pub.publish(msg)

    def _build_statusbar(self):
        bar = tk.Frame(self.root, bg=HEADER_BG, height=24)
        bar.pack(fill='x', side='bottom')
        tk.Label(
            bar,
            text='● Conectado ao Simulador  |  Movimento apenas via GUI',
            font=('Arial', 8), bg=HEADER_BG, fg=TEXT_DIM,
            pady=4, padx=10,
        ).pack(side='left')
        tk.Label(
            bar,
            text='CR10 (6 juntas)  +  COVVI (6 dedos)',
            font=('Arial', 8), bg=HEADER_BG, fg=TEXT_DIM,
            pady=4, padx=10,
        ).pack(side='right')


def main(args=None):
    rclpy.init(args=args)
    node = CombinedControlGUI()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    node.root.mainloop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
