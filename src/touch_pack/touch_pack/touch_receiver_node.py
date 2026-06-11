"""
touch_receiver_node.py — Recebe pacotes UDP do PC plotter do touch sensor
(STM32) e publica os dados como tópico ROS2.

Tópico publicado:
  /touch_sensor/value   std_msgs/Float32   leitura retransmitida pelo plotter

Porta UDP separada da célula de carga (8081 vs 8080): nenhum dos
receptores filtra remetente, então porta compartilhada misturaria os
fluxos silenciosamente.

Payload UDP (little-endian, 8 bytes — TOUCH_PAYLOAD_FMT):
  uint32 seq    — contador do plotter; salto = pacote perdido na rede
  float  value  — leitura do sensor (unidade definida no firmware STM32)
"""

import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32

from .constants import (
    TOUCH_SENSOR_UDP_PORT as UDP_PORT,
    TOUCH_PAYLOAD_FMT as PAYLOAD_FMT,
)

PAYLOAD_SZ = struct.calcsize(PAYLOAD_FMT)   # 8 bytes

QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class TouchReceiverNode(Node):

    def __init__(self):
        super().__init__('touch_receiver')

        self._value_pub = self.create_publisher(Float32, '/touch_sensor/value', QOS_SENSOR)

        self._last_seq: int | None = None
        self._drops = 0

        self._running = True
        self._udp_thr = threading.Thread(
            target=self._udp_loop, daemon=True, name='udp-touch-rx')
        self._udp_thr.start()

        self.get_logger().info(f'TouchReceiver: UDP :{UDP_PORT}')

    # ──────────────────────────────────────────────────────────────────
    def _udp_loop(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.settimeout(1.0)
        try:
            sock.bind(('', UDP_PORT))
        except OSError as exc:
            self.get_logger().error(f'Bind UDP :{UDP_PORT} falhou: {exc}')
            return
        self.get_logger().info(f'UDP bind OK em 0.0.0.0:{UDP_PORT} (broadcast)')

        while self._running and rclpy.ok():
            try:
                raw, _ = sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                break

            if len(raw) < PAYLOAD_SZ:
                continue

            seq, value = struct.unpack(PAYLOAD_FMT, raw[:PAYLOAD_SZ])

            if self._last_seq is not None:
                gap = (seq - self._last_seq - 1) & 0xFFFFFFFF
                if 0 < gap < 1000:          # salto enorme = plotter reiniciou
                    self._drops += gap
                    self.get_logger().warn(
                        f'{gap} pacote(s) perdido(s) (total {self._drops})',
                        throttle_duration_sec=5.0)
            self._last_seq = seq

            msg = Float32(); msg.data = float(value)
            self._value_pub.publish(msg)

        sock.close()

    # ──────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TouchReceiverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
