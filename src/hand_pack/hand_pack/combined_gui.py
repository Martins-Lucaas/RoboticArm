import math
import threading
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tkinter as tk
from tkinter import ttk

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

# ---------- Arm constants ----------
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

ARM_LIMITS_DEG = {
    'joint1': (-360, 360),
    'joint2': (-360, 360),
    'joint3': (-164, 164),
    'joint4': (-360, 360),
    'joint5': (-360, 360),
    'joint6': (-360, 360),
}


class CombinedControlGUI(Node):
    def __init__(self):
        super().__init__('combined_control_gui')
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/cr10_group_controller/joint_trajectory', 10)
        self.hand_pub = self.create_publisher(
            JointTrajectory, '/hand_position_controller/joint_trajectory', 10)
        self._build_ui()

    def _build_ui(self):
        self.root = tk.Tk()
        self.root.title('CR10 + COVVI Hand — Gazebo Control')
        self.root.configure(bg='#2b2b2b')
        self.root.resizable(True, True)

        style = ttk.Style()
        style.theme_use('clam')

        # Title bar
        title = tk.Label(self.root,
                         text='CR10 + COVVI Hand  —  Simulation Control',
                         font=('Arial', 14, 'bold'),
                         bg='#1e1e1e', fg='#e0e0e0')
        title.pack(fill='x', pady=0, ipady=8)

        # Duration row
        dur_frame = tk.Frame(self.root, bg='#2b2b2b')
        dur_frame.pack(fill='x', padx=20, pady=(8, 2))
        tk.Label(dur_frame, text='Duração mov. (s):', width=20, anchor='w',
                 bg='#2b2b2b', fg='#cccccc').pack(side=tk.LEFT)
        self.time_sl = tk.Scale(
            dur_frame, from_=0.1, to=10.0, resolution=0.1,
            orient=tk.HORIZONTAL, bg='#2b2b2b', fg='#cccccc',
            troughcolor='#444', highlightthickness=0)
        self.time_sl.set(2.0)
        self.time_sl.pack(side=tk.LEFT, fill='x', expand=True)

        ttk.Separator(self.root, orient='horizontal').pack(fill='x', pady=6)

        # Two-column layout
        cols = tk.Frame(self.root, bg='#2b2b2b')
        cols.pack(fill='both', expand=True, padx=8, pady=4)

        self._build_arm_panel(cols)
        self._build_hand_panel(cols)

    # ----------------------------------------------------------------- ARM --
    def _build_arm_panel(self, parent):
        frame = tk.LabelFrame(
            parent, text='  Braço CR10 (graus)  ',
            font=('Arial', 11, 'bold'),
            bg='#333', fg='#80c8ff',
            padx=10, pady=8, bd=2, relief='groove')
        frame.pack(side=tk.LEFT, fill='both', expand=True, padx=6, pady=4)

        self.arm_sliders = {}
        self.arm_labels = {}

        for j in ARM_JOINTS:
            lo, hi = ARM_LIMITS_DEG[j]
            row = tk.Frame(frame, bg='#333')
            row.pack(fill='x', pady=2)
            tk.Label(row, text=f'{j}:', width=9, anchor='w',
                     bg='#333', fg='#cccccc').pack(side=tk.LEFT)
            val_lbl = tk.Label(row, text='   0°', width=7, anchor='e',
                               bg='#333', fg='#ffcc55',
                               font=('Courier', 9))
            val_lbl.pack(side=tk.RIGHT)
            sl = tk.Scale(
                row, from_=lo, to=hi, resolution=1,
                orient=tk.HORIZONTAL, showvalue=False,
                bg='#333', fg='#cccccc',
                troughcolor='#555', activebackground='#80c8ff',
                highlightthickness=0,
                command=lambda v, lbl=val_lbl, jn=j: self._arm_changed(v, lbl, jn))
            sl.set(0)
            sl.pack(side=tk.LEFT, fill='x', expand=True)
            self.arm_sliders[j] = sl
            self.arm_labels[j] = val_lbl

        btn_row = tk.Frame(frame, bg='#333')
        btn_row.pack(pady=8)
        tk.Button(
            btn_row, text='Home (0°)',
            bg='#1565C0', fg='white', relief='flat',
            padx=10, pady=4,
            command=self._arm_home).pack(side=tk.LEFT, padx=4)

    def _arm_changed(self, val, lbl, _joint):
        lbl.config(text=f'{int(float(val)):5d}°')
        self._publish_arm()

    def _arm_home(self):
        for sl in self.arm_sliders.values():
            sl.set(0)

    def _publish_arm(self):
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
        frame = tk.LabelFrame(
            parent, text='  Mão COVVI  (0 = aberto · 200 = fechado)  ',
            font=('Arial', 11, 'bold'),
            bg='#333', fg='#80ffaa',
            padx=10, pady=8, bd=2, relief='groove')
        frame.pack(side=tk.LEFT, fill='both', expand=True, padx=6, pady=4)

        self.hand_sliders = {}

        for j in HAND_JOINTS:
            row = tk.Frame(frame, bg='#333')
            row.pack(fill='x', pady=2)
            tk.Label(row, text=f'{j}:', width=8, anchor='w',
                     bg='#333', fg='#cccccc').pack(side=tk.LEFT)
            val_lbl = tk.Label(row, text='  0', width=5, anchor='e',
                               bg='#333', fg='#ffcc55',
                               font=('Courier', 9))
            val_lbl.pack(side=tk.RIGHT)
            sl = tk.Scale(
                row, from_=0, to=200, resolution=1,
                orient=tk.HORIZONTAL, showvalue=False,
                bg='#333', fg='#cccccc',
                troughcolor='#555', activebackground='#80ffaa',
                highlightthickness=0,
                command=lambda v, lbl=val_lbl, jn=j: self._hand_changed(v, lbl, jn))
            sl.set(0)
            sl.pack(side=tk.LEFT, fill='x', expand=True)
            self.hand_sliders[j] = sl

        btn_row = tk.Frame(frame, bg='#333')
        btn_row.pack(pady=8)
        tk.Button(
            btn_row, text='Abrir Tudo',
            bg='#2E7D32', fg='white', relief='flat',
            padx=10, pady=4,
            command=lambda: self._hand_preset(0)).pack(side=tk.LEFT, padx=4)
        tk.Button(
            btn_row, text='Fechar Tudo',
            bg='#C62828', fg='white', relief='flat',
            padx=10, pady=4,
            command=lambda: self._hand_preset(200)).pack(side=tk.LEFT, padx=4)

    def _hand_changed(self, val, lbl, _joint):
        lbl.config(text=f'{int(float(val)):4d}')
        self._publish_hand()

    def _hand_preset(self, value):
        for sl in self.hand_sliders.values():
            sl.set(value)

    def _publish_hand(self):
        vals = {j: self.hand_sliders[j].get() for j in HAND_JOINTS}
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


def main(args=None):
    rclpy.init(args=args)
    node = CombinedControlGUI()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    node.root.mainloop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
