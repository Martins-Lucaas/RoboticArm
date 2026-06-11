"""
palpation_logger.py — Nó ROS 2 que grava cada execução de palpação em disco.

Lê cinco tópicos:
  sub /palpation/start     touch_pack_msgs/PalpationStart — parâmetros do
                                               experimento; marca o início de
                                               um "run".
  sub /palpation/status    touch_pack_msgs/PalpationStatus — fase e ciclo
                                               atuais; encerra o run em
                                               DONE/ABORTED.
  sub /load_cell/force_net std_msgs/Float32  força tare-compensada (N,
                                               compressão positiva) — é o sinal
                                               CANÔNICO do experimento e o
                                               gatilho de amostragem (~50 Hz).
  sub /touch_sensor/value  std_msgs/Float32  leitura do touch sensor (STM32);
                                               pareada por chegada com a
                                               amostra de força (mesma regra
                                               do force_sync: idade máxima
                                               SYNC_MAX_AGE_S — estale e as
                                               colunas saem vazias).
  sub /joint_states        sensor_msgs/JointState — juntas do braço; a pose do
                                               TCP é calculada via FK
                                               (kinematics + T_TOUCH_TOOL_ATTACH).

Saída (em ~/touch_pack_runs/):
  <timestamp>__samples.csv   uma linha por amostra de força:
      t_rel_s, cycle, phase, force_net_n, q1..q6, tcp_x, tcp_y, tcp_z,
      touch_value, touch_age_ms
  <timestamp>__params.json   parâmetros do start
  <timestamp>__summary.json  métricas pós-run (gerado pelo palpation_report)
  <timestamp>__plot.png      força×tempo por fase (se matplotlib disponível)

Encerramento: o run fecha quando recebe status DONE ou ABORTED, ou após
5 min sem amostras de força (timeout de segurança caso o explorer caia).
Ao fechar um run com amostras, o relatório é gerado automaticamente em
background (ver palpation_report.generate_report).
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

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy,
)

from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from rosidl_runtime_py.convert import message_to_ordereddict
from touch_pack_msgs.msg import PalpationStart, PalpationStatus

from .kinematics import forward_kinematics, T_TOUCH_TOOL_ATTACH
from .constants import (
    ARM_JOINTS as _ARM_JOINTS,
    RUNS_DIR as OUTPUT_DIR,
    SYNC_MAX_AGE_S,
)


log = logging.getLogger('touch_pack.palpation_logger')

RUN_IDLE_TIMEOUT_S = 300.0   # 5 min sem força → fecha run "perdido"

CSV_HEADER = ['t_rel_s', 'cycle', 'phase', 'force_net_n',
              'q1', 'q2', 'q3', 'q4', 'q5', 'q6',
              'tcp_x', 'tcp_y', 'tcp_z',
              'touch_value', 'touch_age_ms']

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
        self._cycle: int = 0
        self._last_sample_t: float = 0.0
        self._sample_count: int = 0
        # Últimas juntas do braço (rad, ordem _ARM_JOINTS); None até o
        # primeiro /joint_states.
        self._q: np.ndarray | None = None
        # Última leitura do touch sensor: (valor, instante de chegada).
        # None até o primeiro /touch_sensor/value — sensor é opcional.
        self._touch: tuple[float, float] | None = None

        self.create_subscription(
            PalpationStart, '/palpation/start', self._cb_start, _QOS_COMMAND)
        self.create_subscription(
            PalpationStatus, '/palpation/status', self._cb_status, 10)
        self.create_subscription(
            Float32, '/load_cell/force_net', self._cb_force, _QOS_SENSOR)
        self.create_subscription(
            Float32, '/touch_sensor/value', self._cb_touch, _QOS_SENSOR)
        self.create_subscription(
            JointState, '/joint_states', self._cb_joints, 50)

        # Watchdog @1 Hz para fechar runs órfãos.
        self.create_timer(1.0, self._watchdog)

        self.get_logger().info(
            f'palpation_logger ativo — gravando em {OUTPUT_DIR}/')

    # ── Callbacks ────────────────────────────────────────────────────
    def _cb_start(self, msg: PalpationStart) -> None:
        """Início de um novo run: cria CSV e dump dos parâmetros em JSON.
        O msg tipado vira dict (mesmas chaves dos campos) para o
        __params.json — o palpation_report lê 'force_n' etc. de lá."""
        try:
            params = dict(message_to_ordereddict(msg))
            params['home_deg'] = list(params.get('home_deg', []))
        except Exception:
            params = {}

        with self._lock:
            self._close_run_locked('superseded')
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            csv_path = os.path.join(OUTPUT_DIR, f'{ts}__samples.csv')
            json_path = os.path.join(OUTPUT_DIR, f'{ts}__params.json')
            fh = None
            try:
                fh = open(csv_path, 'w', newline='')
                writer = csv.writer(fh)
                writer.writerow(CSV_HEADER)
                with open(json_path, 'w') as pf:
                    json.dump(params, pf, indent=2, sort_keys=True)
            except OSError as exc:
                self.get_logger().error(
                    f'Falha ao criar arquivos de run: {exc}')
                if fh is not None:
                    fh.close()
                return
            self._csv_fh = fh
            self._csv_writer = writer
            self._run_path = csv_path
            self._run_t0 = time.time()
            self._last_sample_t = self._run_t0
            self._sample_count = 0
            self._phase = 'IDLE'
            self._cycle = 0
            self.get_logger().info(
                f'Run iniciado → {os.path.basename(csv_path)}')

    def _cb_status(self, msg: PalpationStatus) -> None:
        with self._lock:
            self._phase = msg.phase
            self._cycle = int(msg.cycle)
            if msg.phase in ('DONE', 'ABORTED') and self._csv_fh is not None:
                self._close_run_locked(msg.phase)

    def _cb_touch(self, msg: Float32) -> None:
        with self._lock:
            self._touch = (float(msg.data), time.time())

    def _cb_joints(self, msg: JointState) -> None:
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in _ARM_JOINTS):
            return   # mensagem só com juntas da mão
        q = np.array([float(msg.position[idx[j]]) for j in _ARM_JOINTS])
        with self._lock:
            self._q = q

    def _cb_force(self, msg: Float32) -> None:
        """Uma amostra por leitura de força (~50 Hz) — sinal canônico."""
        now = time.time()
        with self._lock:
            self._last_sample_t = now
            if self._csv_writer is None or self._run_t0 is None:
                return
            q = self._q
            if q is not None:
                try:
                    tcp = forward_kinematics(
                        q, T_end=T_TOUCH_TOOL_ATTACH)[:3, 3]
                    tcp_cols = [f'{v:.5f}' for v in tcp]
                except Exception:
                    tcp_cols = ['', '', '']
                q_cols = [f'{v:.5f}' for v in q]
            else:
                q_cols = [''] * 6
                tcp_cols = [''] * 3
            # Touch pareado por chegada (mesma regra do force_sync):
            # fresco entra com valor + idade; estale/ausente sai vazio.
            touch_cols = ['', '']
            if self._touch is not None:
                age_s = now - self._touch[1]
                if age_s <= SYNC_MAX_AGE_S:
                    touch_cols = [f'{self._touch[0]:.4f}',
                                  f'{age_s * 1e3:.1f}']
            try:
                self._csv_writer.writerow([
                    f'{now - self._run_t0:.4f}',
                    self._cycle,
                    self._phase,
                    f'{float(msg.data):.4f}',
                    *q_cols,
                    *tcp_cols,
                    *touch_cols,
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
            if time.time() - self._last_sample_t > RUN_IDLE_TIMEOUT_S:
                self.get_logger().warn(
                    f'Run sem amostras de força há {RUN_IDLE_TIMEOUT_S:.0f}s '
                    '— encerrando por timeout.')
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
        run_path = self._run_path
        n_samples = self._sample_count
        self.get_logger().info(
            f'Run encerrado ({reason}): {n_samples} amostras '
            f'em {duration:.1f}s → {os.path.basename(run_path or "?")}')
        self._csv_fh = None
        self._csv_writer = None
        self._run_path = None
        self._run_t0 = None
        self._sample_count = 0
        # Relatório pós-run (summary JSON + gráfico) em background — só
        # para runs concluídos com dados; 'superseded' é um run substituído.
        if run_path and n_samples > 0 and reason != 'superseded':
            threading.Thread(
                target=self._generate_report, args=(run_path,),
                daemon=True, name='palpation-report').start()

    def _generate_report(self, csv_path: str) -> None:
        try:
            from .palpation_report import generate_report
            summary = generate_report(csv_path)
            self.get_logger().info(
                'Relatório gerado: '
                f'{os.path.basename(summary.get("summary_path", "?"))}')
        except Exception as exc:   # nunca derruba o logger
            self.get_logger().warn(f'Falha ao gerar relatório: {exc}')

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
