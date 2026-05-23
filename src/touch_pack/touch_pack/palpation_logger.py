"""
palpation_logger.py — Nó ROS 2 que grava cada execução de palpação em disco.

Lê três tópicos:
  sub /palpation/start    std_msgs/String   JSON com os parâmetros (force_n,
                                              speed_mms, distance_mm, etc.)
                                              Marca o início de um "run".
  sub /palpation/status   std_msgs/String   JSON {phase, ...} — usado para
                                              detectar transições de fase e
                                              encerrar o run em DONE/ABORTED.
  sub /ft_sensor/wrench   geometry_msgs/WrenchStamped — amostras de força.

Saída:
  ~/touch_pack_runs/<timestamp>__<phase>.csv  (um arquivo por run)
  ~/touch_pack_runs/<timestamp>__params.json   (parâmetros do start)

Cada linha do CSV: t_rel_s, phase, fx, fy, fz, tx, ty, tz.

Encerramento: o run fecha automaticamente quando recebe status DONE ou
ABORTED, ou após 5 min de inatividade do /ft_sensor/wrench (timeout de
segurança caso o explorer caia e nunca emita DONE).
"""
from __future__ import annotations

import csv
import json
import logging
import os
import threading
import time
from datetime import datetime
from typing import IO

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy,
)

from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped


log = logging.getLogger('touch_pack.palpation_logger')

OUTPUT_DIR = os.path.expanduser('~/touch_pack_runs')
RUN_IDLE_TIMEOUT_S = 300.0   # 5 min sem wrench → fecha run "perdido"

_QOS_COMMAND = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)
_QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class PalpationLogger(Node):

    def __init__(self):
        super().__init__('palpation_logger')

        os.makedirs(OUTPUT_DIR, exist_ok=True)

        self._lock = threading.Lock()
        self._csv_fh: IO | None = None
        self._csv_writer: csv.writer | None = None
        self._run_path: str | None = None
        self._run_t0: float | None = None
        self._phase: str = 'IDLE'
        self._last_wrench_t: float = 0.0
        self._sample_count: int = 0

        self.create_subscription(
            String, '/palpation/start', self._cb_start, _QOS_COMMAND)
        self.create_subscription(
            String, '/palpation/status', self._cb_status, 10)
        self.create_subscription(
            WrenchStamped, '/ft_sensor/wrench', self._cb_wrench, _QOS_SENSOR)

        # Watchdog @1 Hz para fechar runs órfãos.
        self.create_timer(1.0, self._watchdog)

        self.get_logger().info(
            f'palpation_logger ativo — gravando em {OUTPUT_DIR}/')

    # ── Callbacks ────────────────────────────────────────────────────
    def _cb_start(self, msg: String) -> None:
        """Início de um novo run: cria CSV e dump dos parâmetros em JSON."""
        try:
            params = json.loads(msg.data)
            if not isinstance(params, dict):
                params = {'raw': msg.data}
        except json.JSONDecodeError:
            params = {'raw': msg.data}

        with self._lock:
            self._close_run_locked('superseded')
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            csv_path = os.path.join(OUTPUT_DIR, f'{ts}__samples.csv')
            json_path = os.path.join(OUTPUT_DIR, f'{ts}__params.json')
            try:
                fh = open(csv_path, 'w', newline='')
                writer = csv.writer(fh)
                writer.writerow(['t_rel_s', 'phase',
                                 'fx', 'fy', 'fz', 'tx', 'ty', 'tz'])
                with open(json_path, 'w') as pf:
                    json.dump(params, pf, indent=2, sort_keys=True)
            except OSError as exc:
                self.get_logger().error(
                    f'Falha ao criar arquivos de run: {exc}')
                return
            self._csv_fh = fh
            self._csv_writer = writer
            self._run_path = csv_path
            self._run_t0 = time.time()
            self._last_wrench_t = self._run_t0
            self._sample_count = 0
            self._phase = 'IDLE'
            self.get_logger().info(
                f'Run iniciado → {os.path.basename(csv_path)}')

    def _cb_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            phase = str(data.get('phase', 'IDLE'))
        except (json.JSONDecodeError, AttributeError):
            return
        with self._lock:
            self._phase = phase
            if phase in ('DONE', 'ABORTED') and self._csv_fh is not None:
                self._close_run_locked(phase)

    def _cb_wrench(self, msg: WrenchStamped) -> None:
        now = time.time()
        with self._lock:
            self._last_wrench_t = now
            if self._csv_writer is None or self._run_t0 is None:
                return
            try:
                self._csv_writer.writerow([
                    f'{now - self._run_t0:.4f}',
                    self._phase,
                    f'{msg.wrench.force.x:.4f}',
                    f'{msg.wrench.force.y:.4f}',
                    f'{msg.wrench.force.z:.4f}',
                    f'{msg.wrench.torque.x:.4f}',
                    f'{msg.wrench.torque.y:.4f}',
                    f'{msg.wrench.torque.z:.4f}',
                ])
                self._sample_count += 1
                # Flush a cada 50 amostras (~1 s @ 50 Hz) para não perder
                # dados se o nó for morto sem encerrar limpo.
                if self._sample_count % 50 == 0 and self._csv_fh is not None:
                    self._csv_fh.flush()
            except (ValueError, OSError) as exc:
                self.get_logger().warn(f'Falha ao gravar amostra: {exc}')

    def _watchdog(self) -> None:
        with self._lock:
            if self._csv_fh is None or self._run_t0 is None:
                return
            if time.time() - self._last_wrench_t > RUN_IDLE_TIMEOUT_S:
                self.get_logger().warn(
                    f'Run sem wrench há {RUN_IDLE_TIMEOUT_S:.0f}s — '
                    'encerrando por timeout.')
                self._close_run_locked('timeout')

    # ── Encerramento ─────────────────────────────────────────────────
    def _close_run_locked(self, reason: str) -> None:
        """Fecha o run atual. Deve ser chamado com `self._lock`."""
        if self._csv_fh is None:
            return
        try:
            self._csv_fh.flush()
            self._csv_fh.close()
        except OSError:
            pass
        duration = (time.time() - self._run_t0
                     if self._run_t0 else 0.0)
        self.get_logger().info(
            f'Run encerrado ({reason}): {self._sample_count} amostras '
            f'em {duration:.1f}s → {os.path.basename(self._run_path or "?")}')
        self._csv_fh = None
        self._csv_writer = None
        self._run_path = None
        self._run_t0 = None
        self._sample_count = 0

    def close(self) -> None:
        with self._lock:
            self._close_run_locked('shutdown')


def main(args=None):
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s.%(msecs)03d [%(name)s] %(levelname)s  %(message)s',
        datefmt='%H:%M:%S')
    rclpy.init(args=args)
    node = PalpationLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
