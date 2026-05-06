"""
Teach Pendant — Modo Manual CR10 + COVVI

Permite jogar as 6 juntas do braço manualmente, gravar waypoints e exportar
a sequência para YAML (para importar no executor automático depois).

Controles:
  • Sliders J1–J6: movem uma junta directamente (envia goal ao controlador)
  • Botões  ◀ ▶: jog incremental (+/- step_size)
  • [Gravar Ponto]: captura a posição actual, pede nome e adiciona à lista
  • [Ir para]: move o braço para o waypoint seleccionado na lista
  • [Apagar]: remove waypoint seleccionado
  • [Exportar YAML]: salva sequência em arquivo .yaml
  • [Exportar Python]: salva snippet .py pronto para copiar no executor
  • [HOME]: envia braço para posição HOME
  • [Mover para posição]: move braço para a posição nos sliders
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, simpledialog
from typing import Optional
import yaml

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ── Constantes ───────────────────────────────────────────────────────────────
_ARM_JOINTS  = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
_JOINT_LABEL = ['J1  Base', 'J2  Ombro', 'J3  Cotovelo',
                 'J4  Pulso1', 'J5  Pulso2', 'J6  Pulso3']

# Limites em graus para os sliders (JOINT_MIN/MAX do kinematics.py)
_J_MIN_DEG = [-180., -260., -135., -260., -135., -360.]
_J_MAX_DEG = [ 180.,   80.,  135.,   80.,  135.,  360.]

_HOME_Q = [0.0, 0.0, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0]

# Durações de movimento
_JOG_DURATION   = 0.6   # segundos — movimento de jog incremental
_SLIDER_DURATION = 1.2  # segundos — arrastar slider
_GOTO_DURATION   = 2.0  # segundos — ir para waypoint gravado

# Esquema de cores (Catppuccin Mocha)
_BG      = '#1e1e2e'
_SURFACE = '#313244'
_OVERLAY = '#45475a'
_TXT     = '#cdd6f4'
_TXT_DIM = '#6c7086'
_GREEN   = '#a6e3a1'
_YELLOW  = '#f9e2af'
_RED     = '#f38ba8'
_BLUE    = '#89b4fa'
_MAUVE   = '#cba6f7'
_PEACH   = '#fab387'


# ── Nó ROS 2 ─────────────────────────────────────────────────────────────────

class TeachPendantNode(Node):

    def __init__(self):
        super().__init__('teach_pendant')
        cb = ReentrantCallbackGroup()

        # Estado actual das juntas
        self._q = np.zeros(6)
        self._q_lock = threading.Lock()

        # Action client do braço
        self._arm_ac = ActionClient(
            self,
            FollowJointTrajectory,
            '/cr10_group_controller/follow_joint_trajectory',
            callback_group=cb,
        )

        # Subscrição ao estado das juntas
        self.create_subscription(
            JointState, '/joint_states', self._cb_joint_state, 10,
            callback_group=cb)

        self.get_logger().info('TeachPendant — aguardando action server...')
        ready = self._arm_ac.wait_for_server(timeout_sec=15.0)
        if not ready:
            self.get_logger().warn('Action server não encontrado — continuando em modo offline.')

    # ── leitura das juntas ────────────────────────────────────────────────
    def _cb_joint_state(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in _ARM_JOINTS:
                idx = _ARM_JOINTS.index(name)
                with self._q_lock:
                    self._q[idx] = msg.position[i]

    def get_q(self) -> np.ndarray:
        with self._q_lock:
            return self._q.copy()

    # ── envio de movimento ────────────────────────────────────────────────
    def move_to(self, q_target: list[float], duration: float = _GOTO_DURATION):
        """Envia goal de trajectória para a posição q_target (radianos)."""
        if not self._arm_ac.server_is_ready():
            self.get_logger().warn('Action server não disponível.')
            return False

        pt = JointTrajectoryPoint()
        pt.positions = list(q_target)
        pt.velocities = [0.0] * 6
        secs = int(duration)
        nsecs = int((duration - secs) * 1e9)
        pt.time_from_start = Duration(sec=secs, nanosec=nsecs)

        traj = JointTrajectory()
        traj.joint_names = _ARM_JOINTS
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self._arm_ac.send_goal(goal)
        return True

    # ── GUI ──────────────────────────────────────────────────────────────
    def run_gui(self):
        root = tk.Tk()
        root.title('Teach Pendant — CR10  |  Modo Manual')
        root.configure(bg=_BG)
        root.geometry('900x720')
        root.resizable(True, True)

        app = TeachPendantApp(root, self)
        root.protocol('WM_DELETE_WINDOW', lambda: _on_close(root, self))
        root.mainloop()


def _on_close(root: tk.Tk, node: TeachPendantNode):
    root.destroy()


# ── Aplicação Tkinter ─────────────────────────────────────────────────────────

class TeachPendantApp:

    def __init__(self, root: tk.Tk, node: TeachPendantNode):
        self._root = root
        self._node = node
        self._waypoints: list[dict] = []   # [{'name': str, 'q': [float×6]}]
        self._step_var  = tk.DoubleVar(value=5.0)   # graus por jog
        self._status_var = tk.StringVar(value='Pronto.')

        # Variáveis dos sliders (em graus)
        self._slider_vars: list[tk.DoubleVar] = [
            tk.DoubleVar(value=math.degrees(q)) for q in _HOME_Q
        ]

        self._build_ui()
        self._poll_joint_state()

    # ─────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        root = self._root

        # ── Cabeçalho ──────────────────────────────────────────────────
        hdr = tk.Frame(root, bg=_SURFACE, padx=14, pady=10)
        hdr.pack(fill='x')
        tk.Label(hdr, text='⚙  TEACH PENDANT  —  CR10 + COVVI',
                 font=('Courier', 13, 'bold'), bg=_SURFACE, fg=_TXT).pack(side='left')
        tk.Label(hdr, text='Modo Manual / Gravação de Trajectória',
                 font=('Courier', 9), bg=_SURFACE, fg=_TXT_DIM).pack(side='right')

        # ── Corpo principal (dois painéis) ─────────────────────────────
        body = tk.Frame(root, bg=_BG)
        body.pack(fill='both', expand=True, padx=8, pady=6)
        body.columnconfigure(0, weight=3)
        body.columnconfigure(1, weight=2)
        body.rowconfigure(0, weight=1)

        self._build_left_panel(body)
        self._build_right_panel(body)

        # ── Barra de status ────────────────────────────────────────────
        bar = tk.Frame(root, bg=_OVERLAY, padx=10, pady=4)
        bar.pack(fill='x', side='bottom')
        tk.Label(bar, textvariable=self._status_var,
                 font=('Courier', 9), bg=_OVERLAY, fg=_TXT,
                 anchor='w').pack(fill='x')

    # ─────────────────────────────────────────────────────────────────────
    def _build_left_panel(self, parent):
        """Painel esquerdo: sliders das juntas + controlos de jog."""
        frame = tk.Frame(parent, bg=_BG)
        frame.grid(row=0, column=0, sticky='nsew', padx=(0, 6))

        tk.Label(frame, text='JUNTAS DO BRAÇO',
                 font=('Courier', 9, 'bold'), bg=_BG, fg=_TXT_DIM).pack(anchor='w', pady=(4, 2))

        # Step size selector
        step_row = tk.Frame(frame, bg=_BG)
        step_row.pack(fill='x', pady=(0, 8))
        tk.Label(step_row, text='Passo jog:', font=('Courier', 9),
                 bg=_BG, fg=_TXT_DIM).pack(side='left')
        for deg in (1, 5, 10, 15):
            tk.Radiobutton(
                step_row, text=f'{deg}°', variable=self._step_var, value=float(deg),
                font=('Courier', 9, 'bold'), bg=_BG, fg=_BLUE,
                selectcolor=_OVERLAY, activebackground=_BG, bd=0,
            ).pack(side='left', padx=4)

        # Uma linha por junta
        self._joint_rows: list[dict] = []
        joint_frame = tk.Frame(frame, bg=_BG)
        joint_frame.pack(fill='both', expand=True)

        for idx in range(6):
            row = tk.Frame(joint_frame, bg=_SURFACE, padx=8, pady=4)
            row.pack(fill='x', pady=2)

            # Label
            tk.Label(row, text=_JOINT_LABEL[idx],
                     font=('Courier', 9, 'bold'), bg=_SURFACE, fg=_TXT,
                     width=14, anchor='w').grid(row=0, column=0)

            # Botão -
            tk.Button(row, text='◀', font=('Courier', 9, 'bold'),
                      bg=_OVERLAY, fg=_RED, relief='flat', bd=0,
                      padx=6, cursor='hand2',
                      command=lambda i=idx: self._jog(i, -1)
                      ).grid(row=0, column=1, padx=2)

            # Slider
            sv = self._slider_vars[idx]
            slider = tk.Scale(
                row, variable=sv, orient='horizontal',
                from_=_J_MIN_DEG[idx], to=_J_MAX_DEG[idx],
                resolution=0.1, length=280,
                bg=_SURFACE, fg=_TXT, troughcolor=_OVERLAY,
                highlightthickness=0, bd=0,
                command=lambda val, i=idx: self._on_slider_change(i, val),
            )
            slider.grid(row=0, column=2)

            # Botão +
            tk.Button(row, text='▶', font=('Courier', 9, 'bold'),
                      bg=_OVERLAY, fg=_GREEN, relief='flat', bd=0,
                      padx=6, cursor='hand2',
                      command=lambda i=idx: self._jog(i, +1)
                      ).grid(row=0, column=3, padx=2)

            # Valor actual (leitura /joint_states)
            val_var = tk.StringVar(value='0.0°')
            tk.Label(row, textvariable=val_var, font=('Courier', 9),
                     bg=_SURFACE, fg=_MAUVE, width=8, anchor='e'
                     ).grid(row=0, column=4, padx=(4, 0))

            self._joint_rows.append({'val_var': val_var, 'slider': slider})

        # Botões de acção directa
        act_frame = tk.Frame(frame, bg=_BG)
        act_frame.pack(fill='x', pady=(10, 4))

        self._btn_home = self._mk_btn(
            act_frame, '⌂  HOME', _YELLOW,
            self._go_home, side='left', padx=4)
        self._btn_move = self._mk_btn(
            act_frame, '▶  Mover para posição', _BLUE,
            self._move_to_sliders, side='left', padx=4)

    # ─────────────────────────────────────────────────────────────────────
    def _build_right_panel(self, parent):
        """Painel direito: lista de waypoints gravados + exportação."""
        frame = tk.Frame(parent, bg=_BG)
        frame.grid(row=0, column=1, sticky='nsew')
        frame.rowconfigure(1, weight=1)

        tk.Label(frame, text='WAYPOINTS GRAVADOS',
                 font=('Courier', 9, 'bold'), bg=_BG, fg=_TXT_DIM).grid(
                     row=0, column=0, columnspan=2, sticky='w', pady=(4, 2))

        # Listbox + scrollbar
        lb_frame = tk.Frame(frame, bg=_BG)
        lb_frame.grid(row=1, column=0, columnspan=2, sticky='nsew', pady=4)
        lb_frame.rowconfigure(0, weight=1)
        lb_frame.columnconfigure(0, weight=1)

        self._listbox = tk.Listbox(
            lb_frame, font=('Courier', 10), bg=_SURFACE, fg=_TXT,
            selectbackground=_OVERLAY, selectforeground=_GREEN,
            relief='flat', bd=0, height=16,
            activestyle='dotbox',
        )
        self._listbox.grid(row=0, column=0, sticky='nsew')
        sb = ttk.Scrollbar(lb_frame, orient='vertical', command=self._listbox.yview)
        sb.grid(row=0, column=1, sticky='ns')
        self._listbox.configure(yscrollcommand=sb.set)
        self._listbox.bind('<<ListboxSelect>>', self._on_wp_select)

        # Info do waypoint seleccionado
        self._wp_info_var = tk.StringVar(value='Seleccione um waypoint.')
        tk.Label(frame, textvariable=self._wp_info_var,
                 font=('Courier', 8), bg=_BG, fg=_TXT_DIM,
                 justify='left', wraplength=260, anchor='w'
                 ).grid(row=2, column=0, columnspan=2, sticky='w', pady=4)

        # Botões de gestão de waypoints
        mgmt = tk.Frame(frame, bg=_BG)
        mgmt.grid(row=3, column=0, columnspan=2, sticky='ew', pady=4)

        self._btn_record = self._mk_btn(
            mgmt, '⏺  Gravar Ponto', _GREEN, self._record_point, side='left', padx=2)
        self._mk_btn(
            mgmt, '🎯  Ir para', _BLUE, self._goto_selected, side='left', padx=2)
        self._mk_btn(
            mgmt, '✕  Apagar', _RED, self._delete_selected, side='left', padx=2)
        self._mk_btn(
            mgmt, '↑', _TXT_DIM, self._move_up, side='left', padx=2)
        self._mk_btn(
            mgmt, '↓', _TXT_DIM, self._move_down, side='left', padx=2)

        ttk.Separator(frame, orient='horizontal').grid(
            row=4, column=0, columnspan=2, sticky='ew', pady=8)

        # Exportação
        tk.Label(frame, text='EXPORTAR',
                 font=('Courier', 9, 'bold'), bg=_BG, fg=_TXT_DIM
                 ).grid(row=5, column=0, columnspan=2, sticky='w', pady=(0, 4))

        exp_frame = tk.Frame(frame, bg=_BG)
        exp_frame.grid(row=6, column=0, columnspan=2, sticky='ew')

        self._mk_btn(exp_frame, '💾  Exportar YAML', _MAUVE,
                     self._export_yaml, side='top', fill='x', pady=3)
        self._mk_btn(exp_frame, '🐍  Exportar Python', _PEACH,
                     self._export_python, side='top', fill='x', pady=3)
        self._mk_btn(exp_frame, '🗑  Limpar Tudo', _RED,
                     self._clear_all, side='top', fill='x', pady=3)

    # ─────────────────────────────────────────────────────────────────────
    @staticmethod
    def _mk_btn(parent, text, color, cmd, side='left',
                padx=4, pady=2, fill=None):
        btn = tk.Button(
            parent, text=text,
            font=('Courier', 9, 'bold'),
            bg=_OVERLAY, fg=color,
            activebackground=color, activeforeground=_BG,
            relief='flat', bd=0, padx=8, pady=6,
            cursor='hand2', command=cmd,
        )
        kw = dict(side=side, padx=padx, pady=pady)
        if fill:
            kw['fill'] = fill
        btn.pack(**kw)
        return btn

    # ─────────────────────────────────────────────────────────────────────
    # Jog e movimento
    # ─────────────────────────────────────────────────────────────────────

    def _jog(self, joint_idx: int, sign: int):
        """Desloca a junta joint_idx por ±step_size graus."""
        step = sign * self._step_var.get()
        cur = self._slider_vars[joint_idx].get()
        new_deg = float(np.clip(cur + step, _J_MIN_DEG[joint_idx], _J_MAX_DEG[joint_idx]))
        self._slider_vars[joint_idx].set(new_deg)
        self._send_current_sliders(_JOG_DURATION)

    def _on_slider_change(self, idx: int, val: str):
        """Chamado quando o utilizador arrasta um slider."""
        # Debounce: só envia se não houver outra chamada em <300ms
        if hasattr(self, '_slider_timer') and self._slider_timer is not None:
            self._root.after_cancel(self._slider_timer)
        self._slider_timer = self._root.after(
            300, lambda: self._send_current_sliders(_SLIDER_DURATION))

    def _slider_timer_id(self):
        if not hasattr(self, '_slider_timer'):
            self._slider_timer = None

    def _send_current_sliders(self, duration: float):
        q = self._sliders_to_rad()
        threading.Thread(
            target=self._node.move_to, args=(q, duration), daemon=True
        ).start()
        self._set_status(f'Movendo para: [{", ".join(f"{math.degrees(v):.1f}°" for v in q)}]')

    def _sliders_to_rad(self) -> list[float]:
        return [math.radians(v.get()) for v in self._slider_vars]

    def _move_to_sliders(self):
        q = self._sliders_to_rad()
        threading.Thread(
            target=self._node.move_to, args=(q, _GOTO_DURATION), daemon=True
        ).start()
        self._set_status('Movendo para posição dos sliders...')

    def _go_home(self):
        for i, v in enumerate(_HOME_Q):
            self._slider_vars[i].set(math.degrees(v))
        threading.Thread(
            target=self._node.move_to, args=(_HOME_Q, _GOTO_DURATION), daemon=True
        ).start()
        self._set_status('Movendo para HOME...')

    # ─────────────────────────────────────────────────────────────────────
    # Gravação de waypoints
    # ─────────────────────────────────────────────────────────────────────

    def _record_point(self):
        """Captura a posição actual (/joint_states) e pede um nome."""
        q_rad = self._node.get_q()
        n = len(self._waypoints) + 1
        default_name = f'ponto_{n}'
        name = simpledialog.askstring(
            'Gravar Ponto',
            f'Nome do waypoint:\n(posição: {self._q_to_str(q_rad)})',
            initialvalue=default_name,
            parent=self._root,
        )
        if name is None:
            return  # utilizador cancelou
        name = name.strip() or default_name

        wp = {'name': name, 'q': q_rad.tolist()}
        self._waypoints.append(wp)
        self._refresh_listbox()
        self._listbox.selection_clear(0, 'end')
        self._listbox.selection_set('end')
        self._listbox.see('end')
        self._set_status(f'Gravado: "{name}"  q=[{self._q_to_str(q_rad)}]')

    def _goto_selected(self):
        idx = self._selected_idx()
        if idx is None:
            return
        wp = self._waypoints[idx]
        q = wp['q']
        for i, v in enumerate(q):
            self._slider_vars[i].set(math.degrees(v))
        threading.Thread(
            target=self._node.move_to, args=(q, _GOTO_DURATION), daemon=True
        ).start()
        self._set_status(f'Indo para: "{wp["name"]}"')

    def _delete_selected(self):
        idx = self._selected_idx()
        if idx is None:
            return
        name = self._waypoints[idx]['name']
        if not messagebox.askyesno('Apagar', f'Apagar waypoint "{name}"?',
                                   parent=self._root):
            return
        self._waypoints.pop(idx)
        self._refresh_listbox()
        self._set_status(f'Apagado: "{name}"')

    def _move_up(self):
        idx = self._selected_idx()
        if idx is None or idx == 0:
            return
        self._waypoints[idx - 1], self._waypoints[idx] = \
            self._waypoints[idx], self._waypoints[idx - 1]
        self._refresh_listbox()
        self._listbox.selection_set(idx - 1)

    def _move_down(self):
        idx = self._selected_idx()
        if idx is None or idx >= len(self._waypoints) - 1:
            return
        self._waypoints[idx + 1], self._waypoints[idx] = \
            self._waypoints[idx], self._waypoints[idx + 1]
        self._refresh_listbox()
        self._listbox.selection_set(idx + 1)

    def _clear_all(self):
        if not self._waypoints:
            return
        if not messagebox.askyesno('Limpar', 'Apagar todos os waypoints?',
                                   parent=self._root):
            return
        self._waypoints.clear()
        self._refresh_listbox()
        self._set_status('Lista de waypoints limpa.')

    # ─────────────────────────────────────────────────────────────────────
    # Exportação
    # ─────────────────────────────────────────────────────────────────────

    def _export_yaml(self):
        if not self._waypoints:
            messagebox.showwarning('Exportar', 'Nenhum waypoint gravado.',
                                   parent=self._root)
            return

        path = filedialog.asksaveasfilename(
            parent=self._root,
            title='Guardar sequência YAML',
            defaultextension='.yaml',
            filetypes=[('YAML', '*.yaml *.yml'), ('Todos', '*.*')],
            initialfile='teach_sequence.yaml',
        )
        if not path:
            return

        data = {
            'teach_sequence': [
                {
                    'name': wp['name'],
                    'q_rad':  [round(v, 6) for v in wp['q']],
                    'q_deg':  [round(math.degrees(v), 3) for v in wp['q']],
                }
                for wp in self._waypoints
            ]
        }

        with open(path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False,
                      allow_unicode=True)

        self._set_status(f'Exportado YAML → {os.path.basename(path)}  '
                         f'({len(self._waypoints)} waypoints)')

    def _export_python(self):
        if not self._waypoints:
            messagebox.showwarning('Exportar', 'Nenhum waypoint gravado.',
                                   parent=self._root)
            return

        path = filedialog.asksaveasfilename(
            parent=self._root,
            title='Guardar snippet Python',
            defaultextension='.py',
            filetypes=[('Python', '*.py'), ('Todos', '*.*')],
            initialfile='teach_sequence.py',
        )
        if not path:
            return

        lines = [
            '# Sequência gravada pelo Teach Pendant — CR10 + COVVI',
            '# Importar no executor: copiar TEACH_WAYPOINTS para grasp_executor.py',
            'import numpy as np',
            '',
            'TEACH_WAYPOINTS = [',
        ]
        for wp in self._waypoints:
            q_str = ', '.join(f'{v:.6f}' for v in wp['q'])
            deg_str = '  # ' + ', '.join(f'{math.degrees(v):.1f}°' for v in wp['q'])
            lines.append(f"    {{'name': {wp['name']!r}, 'q': np.array([{q_str}])}},{deg_str}")
        lines += [
            ']',
            '',
            '# Uso no executor (exemplo):',
            '# for wp in TEACH_WAYPOINTS:',
            '#     self._send_arm(wp["q"])',
            '#     time.sleep(0.5)',
        ]

        with open(path, 'w') as f:
            f.write('\n'.join(lines) + '\n')

        self._set_status(f'Exportado Python → {os.path.basename(path)}  '
                         f'({len(self._waypoints)} waypoints)')

    # ─────────────────────────────────────────────────────────────────────
    # Helpers UI
    # ─────────────────────────────────────────────────────────────────────

    def _refresh_listbox(self):
        self._listbox.delete(0, 'end')
        for i, wp in enumerate(self._waypoints):
            q_deg = [math.degrees(v) for v in wp['q']]
            summary = '  '.join(f'{v:+.0f}°' for v in q_deg)
            self._listbox.insert('end', f'{i+1:2d}. {wp["name"]:<20s}  [{summary}]')

    def _on_wp_select(self, _event=None):
        idx = self._selected_idx()
        if idx is None:
            self._wp_info_var.set('Seleccione um waypoint.')
            return
        wp = self._waypoints[idx]
        q = wp['q']
        q_deg = [math.degrees(v) for v in q]
        info = (
            f'Nome: {wp["name"]}\n'
            f'J1={q_deg[0]:.1f}°  J2={q_deg[1]:.1f}°  J3={q_deg[2]:.1f}°\n'
            f'J4={q_deg[3]:.1f}°  J5={q_deg[4]:.1f}°  J6={q_deg[5]:.1f}°'
        )
        self._wp_info_var.set(info)

    def _selected_idx(self) -> Optional[int]:
        sel = self._listbox.curselection()
        if not sel:
            return None
        return sel[0]

    def _set_status(self, msg: str):
        self._status_var.set(msg)

    @staticmethod
    def _q_to_str(q) -> str:
        return ', '.join(f'{math.degrees(v):.1f}°' for v in q)

    # ─────────────────────────────────────────────────────────────────────
    # Poll /joint_states → actualizar sliders e labels de leitura
    # ─────────────────────────────────────────────────────────────────────

    def _poll_joint_state(self):
        q = self._node.get_q()
        for i, (row, val) in enumerate(
                zip(self._joint_rows, q)):
            row['val_var'].set(f'{math.degrees(val):+.1f}°')
        self._root.after(100, self._poll_joint_state)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = TeachPendantNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_gui()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
