"""
mirror_node.py — Espelhamento sim → CR10 real SEM a GUI.

Antes, o modo MIRROR morava inteiro na palpation_gui (poll loop +
debounce MovJ), então `no_gui:=true` quebrava o espelhamento. Este nó
standalone reproduz o núcleo daquele comportamento:

  • Fase de palpação ATIVA (HOME/DESCENDING/HOLD/SLIDING/RETRACT):
    ServoJ a 33 Hz com a posição lida de /joint_states — latência mínima
    para o controle de força.
  • Fase inativa (IDLE/DONE/ABORTED): MovJ com debounce de 80 ms a partir
    do ÚLTIMO ponto publicado em /cr10_group_controller/joint_trajectory
    (cobre jog de outros publishers), idêntico ao padrão da DobotAPI.

Recursos exclusivos da GUI (drag teach, execução de movimentos salvos,
bridge de força) NÃO são replicados aqui.

Uso (launch): sobe automaticamente com control_mode:=mirror no_gui:=true.
  ros2 launch touch_pack tactile_cell.launch.py \
      end_effector:=touch_tool control_mode:=mirror no_gui:=true

Parâmetros ROS:
  robot_ip   ''     IP do CR10; vazio → ~/.config/touch_pack/robot.json
"""
from __future__ import annotations

import json
import os
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from touch_pack_msgs.msg import PalpationStatus

from .constants import ARM_JOINTS, ROBOT_CONFIG_FILE
from .kinematics import urdf_to_dobot
from .real_driver import (
    CR10RealDriver, CR10RealDriverConfig, CR10RealDriverError,
)

_PERIOD_S = 0.030          # 33 Hz — mesmo período do streaming do explorer
_MOVJ_DEBOUNCE_S = 0.08    # coalesce de publicações em rajada no jog
_RECONNECT_BACKOFF_S = (2.0, 5.0, 10.0, 30.0)

_ACTIVE_PHASES = ('HOME', 'DESCENDING', 'HOLD', 'SLIDING', 'RETRACT')


class MirrorNode(Node):

    def __init__(self):
        super().__init__('mirror_node')
        self.declare_parameter('robot_ip', '')

        self._lock = threading.Lock()
        self._phase: str = 'IDLE'
        self._latest_q: list[float] | None = None
        self._driver: CR10RealDriver | None = None
        self._connected = False
        self._stop = threading.Event()
        self._last_servoj_q: np.ndarray | None = None
        self._movj_timer: threading.Timer | None = None
        self._movj_lock = threading.Lock()

        self.create_subscription(
            JointState, '/joint_states', self._cb_joints, 50)
        self.create_subscription(
            PalpationStatus, '/palpation/status', self._cb_status, 10)
        self.create_subscription(
            JointTrajectory, '/cr10_group_controller/joint_trajectory',
            self._cb_trajectory, 1)

        threading.Thread(target=self._connect_loop, daemon=True,
                         name='mirror-connect').start()
        threading.Thread(target=self._servoj_loop, daemon=True,
                         name='mirror-servoj').start()
        self.get_logger().info('mirror_node ativo — aguardando CR10 real.')

    # ── conexão ──────────────────────────────────────────────────────
    def _robot_ip(self) -> str:
        ip = str(self.get_parameter('robot_ip').value or '').strip()
        if ip:
            return ip
        try:
            with open(ROBOT_CONFIG_FILE) as fh:
                ip = str(json.load(fh).get('robot_ip', '')).strip()
        except (OSError, json.JSONDecodeError, AttributeError):
            ip = ''
        return ip or '192.168.5.2'

    def _connect_loop(self) -> None:
        """Conecta (e reconecta com backoff) ao controlador CR10."""
        attempt = 0
        while not self._stop.is_set():
            if self._connected:
                time.sleep(1.0)
                continue
            ip = self._robot_ip()
            try:
                drv = CR10RealDriver(ip=ip, config=CR10RealDriverConfig())
                drv.connect()
                drv.enable()
                with self._lock:
                    self._driver = drv
                    self._connected = True
                attempt = 0
                self.get_logger().info(f'CR10 real conectado em {ip}.')
            except Exception as exc:
                wait = _RECONNECT_BACKOFF_S[
                    min(attempt, len(_RECONNECT_BACKOFF_S) - 1)]
                attempt += 1
                self.get_logger().warning(
                    f'CR10 em {ip} indisponível ({exc}) — '
                    f'nova tentativa em {wait:.0f}s.')
                self._stop.wait(wait)

    def _drop_connection(self, exc: Exception) -> None:
        self.get_logger().warning(
            f'Conexão com o CR10 perdida ({exc}) — reconectando.')
        with self._lock:
            drv = self._driver
            self._driver = None
            self._connected = False
        if drv is not None:
            try:
                drv.close()
            except Exception:
                pass

    # ── callbacks ────────────────────────────────────────────────────
    def _cb_status(self, msg: PalpationStatus) -> None:
        with self._lock:
            self._phase = msg.phase

    def _cb_joints(self, msg: JointState) -> None:
        pos = dict(zip(msg.name, msg.position))
        try:
            q = [float(pos[j]) for j in ARM_JOINTS]
        except KeyError:
            return   # mensagem parcial (mão) — ignorar
        with self._lock:
            self._latest_q = q

    def _cb_trajectory(self, msg: JointTrajectory) -> None:
        """Jog (fase inativa): MovJ debounced para o último alvo."""
        with self._lock:
            phase = self._phase
            connected = self._connected
        if not connected or phase in _ACTIVE_PHASES:
            return   # palpação ativa → ServoJ loop assume
        if not msg.points:
            return
        target = list(msg.points[-1].positions)
        if len(target) < 6:
            return
        with self._movj_lock:
            if self._movj_timer is not None:
                self._movj_timer.cancel()
            self._movj_timer = threading.Timer(
                _MOVJ_DEBOUNCE_S, self._movj_send, args=[target[:6]])
            self._movj_timer.daemon = True
            self._movj_timer.start()

    def _movj_send(self, q_urdf: list[float]) -> None:
        with self._lock:
            drv = self._driver
            phase = self._phase
        if drv is None or phase in _ACTIVE_PHASES:
            return   # race guard: fase mudou durante o debounce
        try:
            q_dobot_deg = list(np.degrees(
                urdf_to_dobot(np.asarray(q_urdf, dtype=np.float64))))
            drv.mov_j_joint_deg(q_dobot_deg)
        except CR10RealDriverError as exc:
            self._drop_connection(exc)

    # ── ServoJ loop (palpação ativa) ─────────────────────────────────
    def _servoj_loop(self) -> None:
        t_next = time.monotonic() + _PERIOD_S
        while not self._stop.is_set():
            now = time.monotonic()
            self._stop.wait(max(0.0, t_next - now))
            t_next += _PERIOD_S
            if t_next < time.monotonic():
                t_next = time.monotonic() + _PERIOD_S

            with self._lock:
                drv = self._driver
                connected = self._connected
                phase = self._phase
                q = self._latest_q
            if not connected or drv is None or q is None:
                continue
            if phase not in _ACTIVE_PHASES:
                self._last_servoj_q = None
                continue
            q_new = np.asarray(q, dtype=np.float64)
            last = self._last_servoj_q
            if last is not None and float(np.max(np.abs(q_new - last))) < 1e-4:
                continue   # estacionário — sem ServoJ redundante
            try:
                try:
                    drv.servo_j_urdf(q)
                except CR10RealDriverError:
                    drv.prepare_servoj()
                    drv.servo_j_urdf(q)
                self._last_servoj_q = q_new
            except CR10RealDriverError as exc:
                self._drop_connection(exc)

    def destroy_node(self):
        self._stop.set()
        with self._lock:
            drv = self._driver
            self._driver = None
        if drv is not None:
            try:
                drv.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MirrorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
