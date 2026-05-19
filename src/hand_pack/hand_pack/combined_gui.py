import math
import re
import threading
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tkinter as tk
from tkinter import ttk
from tkinter import font as tkfont

# Cage check é fornecido pelo grasp_ml_pack. Soft-import: a GUI funciona
# mesmo sem o pacote (sem o check), apenas log.
try:
    import numpy as _np
    from grasp_ml_pack.cage_check import cage_status as _cage_status
    _CAGE_OK = True
except Exception:
    _np = None
    _cage_status = None
    _CAGE_OK = False

# Extrai o nome do objeto entre parênteses do label de grip
# (ex.: 'Palm Grip (frasco)' → 'frasco'). Usado para acionar o cage check.
_GRIP_OBJ_RE = re.compile(r'\(([^)]+)\)\s*$')


def _grip_label_to_obj(label: str) -> str | None:
    m = _GRIP_OBJ_RE.search(label or '')
    return m.group(1) if m else None

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

# Limites factíveis derivados do manual técnico da COVVI Hand
# (CV-000918-TC Rev. 6 — "Finger flexion: 81° na PONTA do dedo"). Como as
# juntas mimic somam multiplicadores na cadeia, o cap real do driver é
# driver = fingertip_flex / Σ(mults). Ver hand_pack/urdf_helpers.py para
# o detalhamento. Estes valores DEVEM casar com HAND_DRIVER_LIMITS /
# HAND_DRIVER_LOWER em urdf_helpers.py.
#   MAX_RAD ≡ close_limit (slider 100%)
#   MIN_RAD ≡ open_limit  (slider 0% = rest pose levemente curvado,
#                          equivalente ao DigitConfigMsg.open_limit
#                          calibrado no firmware da mão real)
MAX_RAD = {
    'Thumb': 1.0, 'Index': 1.0, 'Middle': 1.0,
    'Ring':  1.0, 'Little': 1.0, 'Rotate': 1.0,
}
MIN_RAD = {
    'Thumb': 0.08, 'Index': 0.12, 'Middle': 0.12,
    'Ring':  0.12, 'Little': 0.12, 'Rotate': 0.0,
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

# Configurações de grasp da mão COVVI — definidas como FRAÇÕES do cap
# (0.0 = junta aberta, 1.0 = no cap factível). Independente do valor
# absoluto do cap, o formato do grip é preservado.
#   palm_grip      → frasco (preensão palmar, garrafa)
#   claw_grip      → tubo de ensaio (garra fina)
#   fingertip_grip → ampola (pinça de pontas)
HAND_GRIPS_FRAC = {
    'Palm Grip (frasco)': {
        'Thumb': 0.69, 'Index': 0.78, 'Middle': 0.78,
        'Ring':  0.75, 'Little': 0.69, 'Rotate': 0.25,
    },
    # Claw e Fingertip: posições MÁXIMAS de fechamento, equivalentes a
    # slider 0..200 dividido por 200 (mesmo target em rad que o cell
    # pick-and-place em grasp_ml_pack/poses.py). A força final é dada
    # pelo PerfectGrasp/_grasp_with_contact_detection — estes valores
    # apenas definem o envelope onde o fechamento pode parar.
    'Claw Grip (tubo)': {
        'Thumb': 0.375, 'Index': 0.375, 'Middle': 0.40,
        'Ring':  0.41,  'Little': 0.435, 'Rotate': 1.00,
    },
    'Fingertip Grip (ampola)': {
        'Thumb': 0.52, 'Index': 0.31, 'Middle': 0.00,
        'Ring':  0.00, 'Little': 0.00, 'Rotate': 0.73,
    },
}

# Escala dos sliders da mão. 100 = "porcentagem de fechamento" — visualmente
# claro de que o slider no máximo corresponde ao cap factível da mão.
HAND_SLIDER_MAX = 100


def _slider_to_rad(j: str, slider_val: float) -> float:
    """Slider 0..HAND_SLIDER_MAX → rad em [MIN_RAD[j], MAX_RAD[j]]."""
    return MIN_RAD[j] + (slider_val / HAND_SLIDER_MAX) * (MAX_RAD[j] - MIN_RAD[j])


def _rad_to_slider(j: str, rad: float) -> int:
    """Inverso de :func:`_slider_to_rad`, com clamp em [0, HAND_SLIDER_MAX]."""
    span = MAX_RAD[j] - MIN_RAD[j]
    if span <= 1e-9:
        return 0
    return max(0, min(HAND_SLIDER_MAX,
                      round((rad - MIN_RAD[j]) / span * HAND_SLIDER_MAX)))


def clamp_finger_interference(rad_targets: dict) -> dict:
    """Reduz `Thumb` se Rotate alto + Index/Middle fechando ameaçam colisão.

    Quando o polegar está em oposição (Rotate elevado), o eixo do polegar
    paira por cima do volume onde as falanges distais de Index/Middle
    chegam ao fecharem. Sem este clamp, basta o usuário soltar Thumb e
    Rotate em 100 simultaneamente para que `thumb_distal` atravesse
    `index_distal`/`middle_distal` independentemente do cap individual de
    cada junta.

    Heurística (calibrada por inspeção visual):
        rot ≤ 0.4    → sem clamp (polegar ainda afastado lateralmente).
        rot > 0.4    → cap efetivo do Thumb cai linearmente com `rot`
                       e com o ângulo máximo entre Index e Middle.

        thumb_eff = THUMB_CAP - 0.4·(rot - 0.4) - 0.3·max(idx, mid)

    O cap permanece ≥ 0.20 rad para sempre permitir algum fechamento do
    polegar (ele só recua o suficiente para não tocar os outros dedos).

    Args:
        rad_targets: dict {junta → rad} a ser publicado. MUTADO IN-PLACE.

    Returns:
        O mesmo dict (conveniência).
    """
    rot = rad_targets.get('Rotate', 0.0)
    if rot <= 0.4:
        return rad_targets

    opp = max(rad_targets.get('Index', 0.0), rad_targets.get('Middle', 0.0))
    thumb_cap = MAX_RAD['Thumb']
    thumb_eff = thumb_cap - 0.4 * (rot - 0.4) - 0.3 * opp
    thumb_eff = max(thumb_eff, 0.20)

    if rad_targets.get('Thumb', 0.0) > thumb_eff:
        rad_targets['Thumb'] = thumb_eff
    return rad_targets


def _grip_frac_to_slider(grip_frac: dict, slider_max: int = HAND_SLIDER_MAX) -> dict:
    """Converte fração de cap (0..1) em valor de slider (0..slider_max)."""
    return {j: max(0, min(slider_max, round(f * slider_max)))
            for j, f in grip_frac.items()}


HAND_GRIPS = {label: _grip_frac_to_slider(frac)
              for label, frac in HAND_GRIPS_FRAC.items()}

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

        # Posição atual das juntas primárias da mão — alimentada por
        # /joint_states e consultada pelo `_grasp_with_contact_detection`
        # para detectar dedos bloqueados pelo objeto.
        self._latest_hand_pos: dict[str, float] = {}
        self._latest_lock = threading.Lock()
        self._grasp_busy = False  # bloqueia novos grips enquanto rampa anda
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10)

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
            header, text=f'0 = aberta  ·  {HAND_SLIDER_MAX} = fechada (cap manual)',
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
                row, from_=0, to=HAND_SLIDER_MAX, resolution=1,
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
            command=lambda: self._hand_preset(HAND_SLIDER_MAX),
        ).pack(side='left', padx=3)
        tk.Button(
            btn_row, text='Pinça (50%)',
            bg=BTN_PRESET, fg=TEXT_MAIN,
            activebackground='#546e7a', activeforeground=TEXT_MAIN,
            relief='flat', padx=12, pady=5, font=('Arial', 9),
            command=lambda: self._hand_preset(HAND_SLIDER_MAX // 2),
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
                command=lambda v=vals, lab=label: self._hand_apply_grip(v, lab),
            ).pack(side='left', padx=2, fill='x', expand=True)

    def _hand_changed(self, val, lbl, _joint):
        lbl.config(text=f'{int(float(val)):4d}')
        self._publish_hand()

    def _hand_preset(self, value):
        """Set all sliders to `value`. Para fechamento (value > 0) usa
        a rampa de contato em thread separada; abertura é direta."""
        if value <= 0:
            # Abertura: comando direto, rápido — não há ejeção a temer.
            for sl in self.hand_sliders.values():
                sl.set(value)
            return
        # Fechamento: aplica via thread com contact-detection
        target = {j: value for j in HAND_JOINTS}
        self._start_contact_grasp(target)

    def _hand_apply_grip(self, vals: dict, label: str | None = None):
        """Aplica configuração de preensão (palm/claw/fingertip) via
        fechamento incremental com detecção de contato — evita ejeção
        do objeto pelo impulso de fechamento simultâneo dos dedos.

        Se `label` contém um objeto entre parênteses (ex.: 'Palm Grip
        (frasco)'), aciona cage check antes do fechamento.
        """
        # Atualiza sliders visualmente para o target ANTES do fechamento
        # (feedback de "qual configuração foi pedida"). Eles serão
        # re-sincronizados ao final caso algum dedo trave antes do alvo.
        for j, v in vals.items():
            sl = self.hand_sliders[j]
            sl.config(command=lambda val: None)
            sl.set(v)
            self.hand_labels[j].config(text=f'{int(v):4d}')
        for j in vals.keys():
            sl = self.hand_sliders[j]
            lbl = self.hand_labels[j]
            sl.config(command=lambda v, lbl=lbl, jn=j: self._hand_changed(v, lbl, jn))
        self._start_contact_grasp(dict(vals),
                                  obj_class=_grip_label_to_obj(label))

    def _start_contact_grasp(self, target_sliders: dict,
                              obj_class: str | None = None) -> None:
        """Dispara fechamento por contato em thread separada (não trava o UI)."""
        if not self._ready:
            return
        if self._grasp_busy:
            return  # já em andamento — ignora duplo clique
        threading.Thread(
            target=self._grasp_with_contact_detection,
            args=(target_sliders, obj_class),
            daemon=True,
        ).start()

    # ---------------------------- /joint_states callback -----------------
    def _on_joint_state(self, msg: JointState):
        """Captura posição atual das 6 juntas primárias da mão."""
        with self._latest_lock:
            for name, pos in zip(msg.name, msg.position):
                if name in HAND_JOINTS:
                    self._latest_hand_pos[name] = float(pos)

    def _read_hand_pos(self) -> dict:
        with self._latest_lock:
            return dict(self._latest_hand_pos)

    # -------------------- Fechamento incremental com contato -----------
    # Parâmetros do algoritmo (inspirados em grasp_ml_pack.PerfectGrasp,
    # mas inteiramente locais para evitar dependência cruzada de pacote).
    _GRASP_STEP_RAD          = 0.04   # ~2.3° por passo
    _GRASP_STEP_DT           = 0.08   # 80 ms entre passos
    _GRASP_LAG_THRESHOLD_RAD = 0.05   # >2.9° de lag commanded↔actual = contato
    _GRASP_STALL_TICKS       = 2      # ticks consecutivos para confirmar
    _GRASP_MIN_STEPS         = 3      # ignora detecção nos primeiros passos
    _GRASP_TIMEOUT_S         = 6.0

    def _publish_hand_targets_rad(self, rad: dict, duration_s: float = 0.15):
        """Envia trajetória das 6 primárias + 26 mimics resolvidas em RAD."""
        names = list(HAND_JOINTS)
        positions = [rad[j] for j in HAND_JOINTS]
        for mimic, driver, mult in MIMIC_JOINTS:
            names.append(mimic)
            positions.append(rad[driver] * mult)
        msg = JointTrajectory()
        msg.joint_names = names
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(
            sec=int(duration_s),
            nanosec=int((duration_s % 1) * 1e9))
        msg.points.append(pt)
        self.hand_pub.publish(msg)

    def _grasp_with_contact_detection(self, target_sliders: dict,
                                       obj_class: str | None = None):
        """Fechamento incremental com detecção de contato por lag articular.

        Algoritmo (industrial: Robotiq adaptive / Schunk SDH "force-closure"):
          1. Rampa o `commanded` em passos `_GRASP_STEP_RAD` (~2.3°).
          2. A cada tick lê `/joint_states`; se um dedo tem
             `lag = commanded − actual > _GRASP_LAG_THRESHOLD_RAD` por
             `_GRASP_STALL_TICKS` ticks → CONGELA esse dedo na posição
             atual (parou no objeto).
          3. Continua rampa apenas com dedos ainda livres.
          4. Termina quando todos pararam OU atingiram o target OU timeout.

        Os dedos contatam UM A UM em vez de saturar o esforço do PID
        simultaneamente — o impulso aplicado ao objeto fica abaixo do
        atrito da pele/objeto e o objeto NÃO é ejetado.
        """
        if self._grasp_busy:
            return
        self._grasp_busy = True

        try:
            # Aplica clamp anti-interferência ANTES de iniciar a rampa.
            target_rad = {j: _slider_to_rad(j, target_sliders[j])
                          for j in HAND_JOINTS}
            target_rad = clamp_finger_interference(target_rad)

            # Cage check (não-fatal): só executa quando o caller passou
            # contexto de objeto (ex.: clique em "Palm Grip (frasco)").
            # Loga warn se preshape/posição do braço deixam fingertips
            # fora da gaiola; o PerfectGrasp ainda tenta fechar.
            if obj_class and _CAGE_OK:
                try:
                    q_arm = _np.array([
                        math.radians(self.arm_sliders[j].get())
                        for j in ARM_JOINTS])
                    cage = _cage_status(q_arm, target_rad, obj_class)
                    if not cage.valid:
                        self.get_logger().warn(
                            f'[{obj_class}:CAGE] {cage.summary()}')
                    else:
                        self.get_logger().info(
                            f'[{obj_class}:CAGE] gaiola válida — fechando')
                except Exception as exc:
                    self.get_logger().debug(
                        f'[CAGE] check ignorado ({exc!r})')

            # Estado inicial — posição atual lida do /joint_states; se
            # ainda não chegou um estado, assume rest pose (MIN_RAD)
            # equivalente ao open_limit da mão real.
            time.sleep(0.05)
            actual = self._read_hand_pos()
            if not actual:
                actual = {j: MIN_RAD[j] for j in HAND_JOINTS}

            commanded: dict = {j: actual.get(j, 0.0) for j in HAND_JOINTS}
            stalled = {j: False for j in HAND_JOINTS}
            stall_ctr = {j: 0 for j in HAND_JOINTS}

            t0 = time.time()
            step_i = 0
            max_step_per_finger = max(
                abs(target_rad[j] - commanded[j]) for j in HAND_JOINTS)
            max_steps = int(max_step_per_finger / self._GRASP_STEP_RAD) + 4

            while (time.time() - t0) < self._GRASP_TIMEOUT_S:
                # 1. Detecção de stall — só após alguns passos para evitar
                # falsos positivos do delay inicial do controller.
                if step_i >= self._GRASP_MIN_STEPS:
                    cur = self._read_hand_pos()
                    for j in HAND_JOINTS:
                        if stalled[j]:
                            continue
                        lag = abs(commanded[j] - cur.get(j, commanded[j]))
                        if lag > self._GRASP_LAG_THRESHOLD_RAD:
                            stall_ctr[j] += 1
                            if stall_ctr[j] >= self._GRASP_STALL_TICKS:
                                stalled[j] = True
                                # Congela o commanded na ACTUAL → não
                                # acumula erro/integral para frente.
                                commanded[j] = float(cur.get(j, commanded[j]))
                        else:
                            stall_ctr[j] = 0

                # 2. Rampa dedos ainda livres
                all_done = True
                for j in HAND_JOINTS:
                    if stalled[j]:
                        continue
                    if commanded[j] < target_rad[j] - 1e-4:
                        commanded[j] = min(commanded[j] + self._GRASP_STEP_RAD,
                                            target_rad[j])
                        all_done = False
                    elif commanded[j] > target_rad[j] + 1e-4:
                        commanded[j] = max(commanded[j] - self._GRASP_STEP_RAD,
                                            target_rad[j])
                        all_done = False

                # 3. Publica targets
                self._publish_hand_targets_rad(commanded,
                                               duration_s=self._GRASP_STEP_DT * 1.5)

                # 4. Encerra se nada para fazer
                if all_done or all(stalled.values()):
                    # Após estabilização, mantém um pequeno bias por
                    # 200 ms para o atrito desenvolver no contato.
                    time.sleep(0.20)
                    break

                if step_i > max_steps + 30:
                    break  # safety abort

                step_i += 1
                time.sleep(self._GRASP_STEP_DT)

            # 5. Atualiza sliders da GUI para refletirem o estado final
            # (alguns dedos podem ter parado antes do target).
            self._sync_sliders_to_rad(commanded)
        finally:
            self._grasp_busy = False

    def _sync_sliders_to_rad(self, rad: dict) -> None:
        """Reflete posições reais (em rad) nos sliders da GUI."""
        for j in HAND_JOINTS:
            sl = self.hand_sliders.get(j)
            if sl is None:
                continue
            slider_val = _rad_to_slider(j, rad[j])
            sl.config(command=lambda val: None)
            sl.set(slider_val)
            self.hand_labels[j].config(text=f'{int(slider_val):4d}')
            sl.config(command=lambda v, lbl=self.hand_labels[j], jn=j:
                       self._hand_changed(v, lbl, jn))

    def _publish_hand(self):
        if not self._ready:
            return
        vals  = {j: self.hand_sliders[j].get() for j in HAND_JOINTS}
        names = list(HAND_JOINTS)
        # Converte slider→rad antes do clamp para que a interferência
        # opere no espaço físico (independe da escala visual).
        rad_targets = {j: _slider_to_rad(j, vals[j])
                       for j in HAND_JOINTS}
        rad_targets = clamp_finger_interference(rad_targets)

        positions = [rad_targets[j] for j in HAND_JOINTS]
        for mimic, driver, mult in MIMIC_JOINTS:
            names.append(mimic)
            positions.append(rad_targets[driver] * mult)
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
