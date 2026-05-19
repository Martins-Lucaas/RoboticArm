import threading
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tkinter as tk
from tkinter import ttk

try:
    from covvi_interfaces.srv import SetDigitPosn
    COVVI_AVAILABLE = True
except ImportError:
    COVVI_AVAILABLE = False

# (mimic_joint, driver_joint, multiplier_from_urdf)
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

# Limites por junta driver — devem casar com HAND_DRIVER_LIMITS /
# HAND_DRIVER_LOWER em hand_pack/urdf_helpers.py. MIN_RAD funciona
# como o ``open_limit`` calibrado do firmware (rest pose levemente
# curvado da mão real).
MAX_RAD = {'Thumb': 1.0, 'Index': 1.0, 'Middle': 1.0,
           'Ring':  1.0, 'Little': 1.0, 'Rotate': 1.0}
MIN_RAD = {'Thumb': 0.08, 'Index': 0.12, 'Middle': 0.12,
           'Ring':  0.12, 'Little': 0.12, 'Rotate': 0.0}

# Slider 0..HAND_SLIDER_MAX (percent semantics) interpola linearmente
# entre MIN_RAD (open_limit) e MAX_RAD (close_limit) — mesma semântica
# do firmware COVVI (0% = aberto, 100% = fechado).
HAND_SLIDER_MAX = 100

MAIN_JOINTS = ['Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Rotate']


def _slider_to_rad(j: str, slider_val: float) -> float:
    return MIN_RAD[j] + (slider_val / HAND_SLIDER_MAX) * (MAX_RAD[j] - MIN_RAD[j])


class HandControlGUI(Node):
    def __init__(self):
        super().__init__('hand_control_gui')
        self.sim_pub = self.create_publisher(
            JointTrajectory, '/hand_position_controller/joint_trajectory', 10)
        self.real_client = None
        self._build_ui()

    # ------------------------------------------------------------------ UI --
    def _build_ui(self):
        self.root = tk.Tk()
        self.root.title('COVVI Hand Control')
        self.sliders = {}

        tk.Label(self.root, text='COVVI Hand Control',
                 font=('Arial', 13, 'bold')).pack(pady=8)

        # Movement duration (simulation trajectory)
        f = tk.Frame(self.root)
        f.pack(fill='x', padx=20)
        tk.Label(f, text='Duração mov. (s):', width=20, anchor='w').pack(side=tk.LEFT)
        self.time_sl = tk.Scale(f, from_=0.1, to=5.0, resolution=0.1,
                                orient=tk.HORIZONTAL)
        self.time_sl.set(1.0)
        self.time_sl.pack(side=tk.LEFT, fill='x', expand=True)

        ttk.Separator(self.root, orient='horizontal').pack(fill='x', pady=6)

        tk.Label(self.root,
                 text=f'Posição  (0 = aberto  ·  {HAND_SLIDER_MAX} = fechado, cap manual)',
                 font=('Arial', 9, 'italic')).pack()

        for joint in MAIN_JOINTS:
            f = tk.Frame(self.root)
            f.pack(fill='x', padx=20)
            tk.Label(f, text=f'{joint}:', width=8, anchor='w').pack(side=tk.LEFT)
            s = tk.Scale(f, from_=0, to=HAND_SLIDER_MAX, resolution=1,
                         orient=tk.HORIZONTAL,
                         command=lambda _: self._on_change())
            s.pack(side=tk.LEFT, fill='x', expand=True)
            self.sliders[joint] = s

        ttk.Separator(self.root, orient='horizontal').pack(fill='x', pady=6)

        # Real hand section
        tk.Label(self.root, text='Mão Física  (covvi_hand_driver)',
                 font=('Arial', 10, 'bold')).pack()

        f = tk.Frame(self.root)
        f.pack(fill='x', padx=20, pady=2)
        tk.Label(f, text='Namespace:', width=12, anchor='w').pack(side=tk.LEFT)
        self.ns_var = tk.StringVar(value='/test/server_1')
        tk.Entry(f, textvariable=self.ns_var).pack(side=tk.LEFT, fill='x', expand=True)

        f = tk.Frame(self.root)
        f.pack(fill='x', padx=20)
        tk.Label(f, text='Velocidade:', width=12, anchor='w').pack(side=tk.LEFT)
        self.speed_sl = tk.Scale(f, from_=1, to=100, resolution=1,
                                  orient=tk.HORIZONTAL)
        self.speed_sl.set(50)
        self.speed_sl.pack(side=tk.LEFT, fill='x', expand=True)

        f = tk.Frame(self.root)
        f.pack(pady=6)
        self.conn_btn = tk.Button(f, text='Conectar Mão Real',
                                   command=self._toggle_real,
                                   bg='#4CAF50', fg='white', width=20)
        self.conn_btn.pack()

        self.status_lbl = tk.Label(self.root,
                                    text='Mão real: desconectada',
                                    fg='gray', font=('Arial', 9, 'italic'))
        self.status_lbl.pack(pady=4)

    # ---------------------------------------------------------- real hand --
    def _toggle_real(self):
        if self.real_client is None:
            self._connect()
        else:
            self._disconnect()

    def _connect(self):
        if not COVVI_AVAILABLE:
            self._set_status('covvi_interfaces não instalado', 'red')
            return
        ns = self.ns_var.get().rstrip('/')
        self.real_client = self.create_client(SetDigitPosn, f'{ns}/SetDigitPosn')
        self._ns = ns
        self.conn_btn.config(text='Desconectar Mão Real', bg='#f44336')
        self._set_status(f'Conectando a {ns}...', 'orange')
        threading.Thread(target=self._wait_service, daemon=True).start()

    def _wait_service(self):
        ok = self.real_client.wait_for_service(timeout_sec=5.0)
        msg = f'Conectada: {self._ns}' if ok else 'Serviço não encontrado'
        color = 'green' if ok else 'red'
        self.root.after(0, lambda: self._set_status(msg, color))

    def _disconnect(self):
        self.real_client = None
        self.conn_btn.config(text='Conectar Mão Real', bg='#4CAF50')
        self._set_status('Mão real: desconectada', 'gray')

    def _set_status(self, msg, color):
        self.status_lbl.config(text=msg, fg=color)

    # ----------------------------------------------------------- control --
    def _on_change(self):
        self._publish_sim()
        if self.real_client is not None:
            self._send_real()

    def _publish_sim(self):
        vals = {j: self.sliders[j].get() for j in MAIN_JOINTS}

        names, positions = list(MAIN_JOINTS), []
        for j in MAIN_JOINTS:
            positions.append(_slider_to_rad(j, vals[j]))

        for mimic, driver, mult in MIMIC_JOINTS:
            names.append(mimic)
            positions.append(_slider_to_rad(driver, vals[driver]) * mult)

        msg = JointTrajectory()
        msg.joint_names = names
        pt = JointTrajectoryPoint()
        pt.positions = positions
        dur = self.time_sl.get()
        pt.time_from_start = Duration(sec=int(dur),
                                       nanosec=int((dur % 1) * 1e9))
        msg.points.append(pt)
        self.sim_pub.publish(msg)

    def _send_real(self):
        if not COVVI_AVAILABLE or self.real_client is None:
            return
        req = SetDigitPosn.Request()
        req.speed.value = self.speed_sl.get()
        req.thumb  = int(self.sliders['Thumb'].get())
        req.index  = int(self.sliders['Index'].get())
        req.middle = int(self.sliders['Middle'].get())
        req.ring   = int(self.sliders['Ring'].get())
        req.little = int(self.sliders['Little'].get())
        req.rotate = int(self.sliders['Rotate'].get())
        fut = self.real_client.call_async(req)
        fut.add_done_callback(lambda f: self._on_real_resp(f))

    def _on_real_resp(self, future):
        try:
            future.result()
        except Exception as e:
            self.get_logger().warn(f'Real hand error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = HandControlGUI()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    node.root.mainloop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
