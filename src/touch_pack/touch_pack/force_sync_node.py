"""
force_sync_node.py — Pareia em tempo real a célula de carga (ESP32, UDP 8080)
com o touch sensor (STM32 → PC plotter, UDP 8081).

Tópico publicado:
  /touch_sync/data   touch_pack_msgs/SyncedTouch   par sincronizado, 100 Hz

Estratégia de sincronização: as duas fontes publicam Float32 sem stamp,
mas ambas chegam por UDP e são carimbadas AQUI, no mesmo relógio do PC
do ROS — os instantes de chegada são diretamente comparáveis (latência
de LAN ≪ período de 10 ms da célula). Um timer a 100 Hz emite o par
somente quando as duas amostras são frescas (< SYNC_MAX_AGE_S); as
idades vão na mensagem para auditoria posterior.
"""

from __future__ import annotations

import threading
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32

from touch_pack_msgs.msg import SyncedTouch

from .constants import SYNC_MAX_AGE_S

SYNC_RATE_HZ = 1000.0   # mesma taxa da célula de carga (ESP a 1 kHz) e do toque
# NOTA: o par só é tão fresco quanto a fonte MAIS LENTA. Com a célula a 1 kHz e
# o STM32 do toque a 1 kHz ambas as fontes acompanham; se uma cair abaixo disso,
# o último valor é repetido (segurado dentro de SYNC_MAX_AGE_S) — sobe a taxa de
# emissão, não a informação.

QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)

# Amostra carimbada: (valor, instante de chegada em segundos).
Sample = Tuple[float, float]


def pair_if_fresh(now_s: float,
                  load_cell: Optional[Sample],
                  touch: Optional[Sample],
                  max_age_s: float = SYNC_MAX_AGE_S,
                  ) -> Optional[Tuple[float, float, float, float]]:
    """Retorna (load_cell_n, touch_value, lc_age_ms, touch_age_ms) se as
    duas amostras existem e são frescas; None caso contrário. Puro, para
    ser testável sem rclpy."""
    if load_cell is None or touch is None:
        return None
    lc_age = now_s - load_cell[1]
    th_age = now_s - touch[1]
    if lc_age > max_age_s or th_age > max_age_s:
        return None
    return load_cell[0], touch[0], lc_age * 1e3, th_age * 1e3


class ForceSyncNode(Node):

    def __init__(self):
        super().__init__('force_sync')

        self._lock = threading.Lock()
        self._load_cell: Optional[Sample] = None
        self._touch:     Optional[Sample] = None
        self._was_synced = False

        # Usa /load_cell/force_net (tare-compensada, +compressão) — a MESMA
        # grandeza que o explorer (PID + limite de 15 N) e o display da GUI
        # consomem. Antes assinava /load_cell/force, referenciada ao zero da
        # CALIBRAÇÃO (semanas atrás); o dataset gravado divergia da força
        # efetivamente controlada pela deriva entre zero de calibração e
        # tare do dia. Requer GUI aberta + Tare feito para publicar; sem
        # isso o par fica 'sem amostra fresca' (warn), o que é correto.
        self.create_subscription(Float32, '/load_cell/force_net',
                                 self._on_load_cell, QOS_SENSOR)
        self.create_subscription(Float32, '/touch_sensor/value',
                                 self._on_touch, QOS_SENSOR)

        self._sync_pub = self.create_publisher(SyncedTouch, '/touch_sync/data', 10)
        self.create_timer(1.0 / SYNC_RATE_HZ, self._on_timer)

        self.get_logger().info(
            f'ForceSync: /load_cell/force_net + /touch_sensor/value → '
            f'/touch_sync/data @ {SYNC_RATE_HZ:.0f} Hz '
            f'(idade máx. {SYNC_MAX_AGE_S * 1e3:.0f} ms)')

    # ──────────────────────────────────────────────────────────────────
    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_load_cell(self, msg: Float32) -> None:
        with self._lock:
            self._load_cell = (msg.data, self._now_s())

    def _on_touch(self, msg: Float32) -> None:
        with self._lock:
            self._touch = (msg.data, self._now_s())

    # ──────────────────────────────────────────────────────────────────
    def _on_timer(self) -> None:
        now = self._now_s()
        with self._lock:
            pair = pair_if_fresh(now, self._load_cell, self._touch)

        if pair is None:
            if self._was_synced:
                self._was_synced = False
                self.get_logger().warn('Sincronização perdida — fonte sem '
                                       'amostra fresca')
            return

        if not self._was_synced:
            self._was_synced = True
            self.get_logger().info('Fontes sincronizadas')

        msg = SyncedTouch()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.load_cell_n, msg.touch_value, \
            msg.load_cell_age_ms, msg.touch_age_ms = (
                float(pair[0]), float(pair[1]), float(pair[2]), float(pair[3]))
        self._sync_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ForceSyncNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
