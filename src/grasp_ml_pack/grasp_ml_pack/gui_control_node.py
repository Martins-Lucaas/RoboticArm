"""
GUI de operação da célula de manufatura com esteira.

Interface Tkinter integrada ao ROS 2. O spin do ROS corre em thread de fundo;
o Tkinter ocupa a thread principal (exigência da maioria das plataformas).

Botões:
  [Próximo Objeto]   → /conveyor/advance  (Trigger)
  [Estágio Anterior] → /conveyor/retreat  (Trigger)
  [Agarrar]          → /cell/execute_grasp (Trigger)
  [Home]             → /cell/go_home      (Trigger)
  [Resetar Esteira]  → /conveyor/reset    (Trigger)

Monitora (subscreve):
  /conveyor/status  (String JSON) — estado da esteira
  /cell/status      (String JSON) — estado do executor

Todos os calls de serviço são assíncronos: a GUI não trava durante o movimento.
"""

from __future__ import annotations

import json
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from std_srvs.srv import Trigger


# Esquema de cores
_CLR_BG      = '#1e1e2e'
_CLR_SURFACE = '#313244'
_CLR_BTN_GRN = '#a6e3a1'   # Agarrar / Próximo
_CLR_BTN_YLW = '#f9e2af'   # Home
_CLR_BTN_RED = '#f38ba8'   # Reset / Anterior
_CLR_BTN_BLU = '#89b4fa'   # ações auxiliares
_CLR_TXT     = '#cdd6f4'
_CLR_TXT_DIM = '#6c7086'
_CLR_OK      = '#a6e3a1'
_CLR_ERR     = '#f38ba8'
_CLR_WARN    = '#fab387'


class GUIControlNode(Node):

    def __init__(self):
        super().__init__('gui_control')

        cb = ReentrantCallbackGroup()

        # Clients de serviço
        self._cli: dict[str, rclpy.client.Client] = {
            'advance': self.create_client(
                Trigger, '/conveyor/advance', callback_group=cb),
            'retreat': self.create_client(
                Trigger, '/conveyor/retreat', callback_group=cb),
            'reset':   self.create_client(
                Trigger, '/conveyor/reset',   callback_group=cb),
            'execute': self.create_client(
                Trigger, '/cell/execute_grasp', callback_group=cb),
            'home':    self.create_client(
                Trigger, '/cell/go_home',     callback_group=cb),
        }

        # Estado da esteira e do executor (atualizado via callbacks)
        self._conveyor_state: dict = {}
        self._cell_state:     dict = {}

        self.create_subscription(
            String, '/conveyor/status', self._cb_conveyor, 10,
            callback_group=cb)
        self.create_subscription(
            String, '/cell/status', self._cb_cell, 10,
            callback_group=cb)

        self.get_logger().info('GUI pronta.')

    # ──────────────────────────────────────────────────────────────────
    def _cb_conveyor(self, msg: String):
        try:
            self._conveyor_state = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _cb_cell(self, msg: String):
        try:
            self._cell_state = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    # ──────────────────────────────────────────────────────────────────
    def call_service(self, key: str, on_result=None):
        """Chama um serviço de forma assíncrona (não trava a GUI)."""
        client = self._cli[key]
        if not client.service_is_ready():
            self.get_logger().warn(f'Serviço {key} não disponível.')
            if on_result:
                on_result(False, f'Serviço {key} não disponível.')
            return

        future = client.call_async(Trigger.Request())

        def _done(f):
            if on_result and not f.cancelled():
                try:
                    res = f.result()
                    on_result(res.success, res.message)
                except Exception as exc:
                    on_result(False, str(exc))

        future.add_done_callback(_done)

    # ──────────────────────────────────────────────────────────────────
    def run_gui(self):
        """Cria e executa a janela Tkinter (deve ser chamado na thread principal)."""
        root = tk.Tk()
        root.title('Célula de Manufatura — Painel de Controle')
        root.configure(bg=_CLR_BG)
        root.geometry('640x520')
        root.resizable(False, False)

        _app = CellControlApp(root, self)
        root.protocol('WM_DELETE_WINDOW', lambda: self._on_close(root))
        root.mainloop()

    def _on_close(self, root: tk.Tk):
        root.destroy()


# ──────────────────────────────────────────────────────────────────────────────

class CellControlApp:
    """Widget principal da aplicação de controle da célula."""

    def __init__(self, root: tk.Tk, node: GUIControlNode):
        self._root = root
        self._node = node
        self._feedback_var = tk.StringVar(value='Sistema pronto.')
        self._cell_state_var = tk.StringVar(value='—')
        self._conveyor_var   = tk.StringVar(value='—')

        self._build_ui()
        self._poll_status()

    # ──────────────────────────────────────────────────────────────────
    def _build_ui(self):
        root = self._root

        # Cabeçalho
        hdr = tk.Frame(root, bg=_CLR_SURFACE, padx=16, pady=12)
        hdr.pack(fill='x')
        tk.Label(hdr, text='CÉLULA DE MANUFATURA  CR10 + COVVI',
                 font=('Courier', 14, 'bold'), bg=_CLR_SURFACE,
                 fg=_CLR_TXT).pack(side='left')

        # Status strip
        sstrip = tk.Frame(root, bg=_CLR_BG, padx=16, pady=8)
        sstrip.pack(fill='x')
        tk.Label(sstrip, text='Esteira:', font=('Courier', 10),
                 bg=_CLR_BG, fg=_CLR_TXT_DIM).grid(row=0, column=0, sticky='w')
        tk.Label(sstrip, textvariable=self._conveyor_var,
                 font=('Courier', 10, 'bold'), bg=_CLR_BG,
                 fg=_CLR_BTN_BLU).grid(row=0, column=1, sticky='w', padx=(6, 30))
        tk.Label(sstrip, text='Executor:', font=('Courier', 10),
                 bg=_CLR_BG, fg=_CLR_TXT_DIM).grid(row=0, column=2, sticky='w')
        tk.Label(sstrip, textvariable=self._cell_state_var,
                 font=('Courier', 10, 'bold'), bg=_CLR_BG,
                 fg=_CLR_BTN_GRN).grid(row=0, column=3, sticky='w', padx=6)

        ttk.Separator(root, orient='horizontal').pack(fill='x', padx=16)

        # Painel de botões
        btn_frame = tk.Frame(root, bg=_CLR_BG, padx=24, pady=20)
        btn_frame.pack(fill='both', expand=True)

        # Seção Esteira
        tk.Label(btn_frame, text='CONTROLE DA ESTEIRA',
                 font=('Courier', 9, 'bold'), bg=_CLR_BG,
                 fg=_CLR_TXT_DIM).grid(row=0, column=0, columnspan=2,
                                        sticky='w', pady=(0, 6))

        self._btn_advance = self._make_btn(
            btn_frame, 'Próximo Objeto  ▶', _CLR_BTN_GRN,
            lambda: self._action('advance', 'Avançando esteira...'),
            row=1, col=0)
        self._btn_retreat = self._make_btn(
            btn_frame, '◀  Estágio Anterior', _CLR_BTN_RED,
            lambda: self._action('retreat', 'Recuando esteira...'),
            row=1, col=1)

        self._btn_reset = self._make_btn(
            btn_frame, '⟳  Resetar Esteira', _CLR_BTN_RED,
            lambda: self._confirm_reset(),
            row=2, col=0, colspan=2)

        # Separador
        tk.Label(btn_frame, text='', bg=_CLR_BG).grid(row=3, columnspan=2)
        ttk.Separator(btn_frame, orient='horizontal').grid(
            row=4, column=0, columnspan=2, sticky='ew', pady=8)

        # Seção Robô
        tk.Label(btn_frame, text='CONTROLE DO ROBÔ',
                 font=('Courier', 9, 'bold'), bg=_CLR_BG,
                 fg=_CLR_TXT_DIM).grid(row=5, column=0, columnspan=2,
                                        sticky='w', pady=(0, 6))

        self._btn_grasp = self._make_btn(
            btn_frame, '✋  AGARRAR', _CLR_BTN_BLU,
            lambda: self._action('execute', 'Iniciando ciclo de grasp...'),
            row=6, col=0, colspan=2, large=True)

        self._btn_home = self._make_btn(
            btn_frame, '⌂  Home', _CLR_BTN_YLW,
            lambda: self._action('home', 'Enviando braço ao home...'),
            row=7, col=0, colspan=2)

        # Área de feedback
        fb_frame = tk.Frame(root, bg=_CLR_SURFACE, padx=16, pady=8)
        fb_frame.pack(fill='x', side='bottom')
        tk.Label(fb_frame, textvariable=self._feedback_var,
                 font=('Courier', 10), bg=_CLR_SURFACE,
                 fg=_CLR_TXT, anchor='w', justify='left',
                 wraplength=600).pack(fill='x')

    # ──────────────────────────────────────────────────────────────────
    @staticmethod
    def _make_btn(parent, text, color, command, row, col,
                  colspan=1, large=False) -> tk.Button:
        font_size = 13 if large else 11
        btn = tk.Button(
            parent, text=text,
            font=('Courier', font_size, 'bold'),
            bg=_CLR_SURFACE, fg=color,
            activebackground=color, activeforeground=_CLR_BG,
            relief='flat', bd=0, padx=12,
            pady=16 if large else 10,
            cursor='hand2', command=command)
        btn.grid(row=row, column=col, columnspan=colspan,
                 sticky='ew', padx=4, pady=3)
        parent.columnconfigure(col, weight=1)
        return btn

    # ──────────────────────────────────────────────────────────────────
    def _action(self, key: str, msg: str):
        self._set_feedback(msg, color=_CLR_WARN)
        self._set_buttons(False)

        def on_result(success: bool, message: str):
            color = _CLR_OK if success else _CLR_ERR
            self._root.after(0, lambda: self._set_feedback(message, color))
            self._root.after(0, lambda: self._set_buttons(True))

        self._node.call_service(key, on_result=on_result)

    def _confirm_reset(self):
        if messagebox.askyesno(
                'Confirmar Reset',
                'Isso irá remover o objeto atual e reiniciar\n'
                'a sequência da esteira. Deseja continuar?'):
            self._action('reset', 'Resetando esteira...')

    def _set_feedback(self, msg: str, color: str = _CLR_TXT):
        self._feedback_var.set(msg)
        # Atualiza a cor dinamicamente localizando o label
        for widget in self._root.winfo_children():
            for child in widget.winfo_children():
                if hasattr(child, 'cget') and 'textvariable' in child.keys():
                    try:
                        if child.cget('textvariable') == str(self._feedback_var):
                            child.configure(fg=color)
                    except Exception:
                        pass

    def _set_buttons(self, enabled: bool):
        """Habilita ou desabilita todos os botões de ação durante operação."""
        state = 'normal' if enabled else 'disabled'
        for attr in ('_btn_advance', '_btn_retreat', '_btn_reset',
                     '_btn_grasp', '_btn_home'):
            btn = getattr(self, attr, None)
            if btn:
                btn.configure(state=state)

    # ──────────────────────────────────────────────────────────────────
    def _poll_status(self):
        """Atualiza os labels de status periodicamente (poll Tkinter-safe)."""
        cs = self._node._conveyor_state
        if cs:
            obj  = cs.get('current_obj', '—')
            has  = '✔' if cs.get('has_object') else '○'
            idx  = cs.get('queue_idx', -1) + 1
            tot  = cs.get('queue_total', 0)
            self._conveyor_var.set(f'{has} {obj}  [{idx}/{tot}]')

        xs = self._node._cell_state
        if xs:
            state  = xs.get('state', '—')
            busy   = '⏳' if xs.get('busy') else '✔'
            last   = xs.get('last_obj') or '—'
            self._cell_state_var.set(f'{busy} {state}  (objeto: {last})')

            # Habilita/desabilita botão Agarrar conforme disponibilidade
            if hasattr(self, '_btn_grasp'):
                busy_flag = xs.get('busy', False)
                self._btn_grasp.configure(
                    state='disabled' if busy_flag else 'normal')

        self._root.after(500, self._poll_status)


def main(args=None):
    rclpy.init(args=args)
    node = GUIControlNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    # ROS 2 spin em thread de fundo (Tkinter exige a thread principal)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_gui()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
