"""
force_receiver_node.py — Recebe pacotes UDP da ESP32 (célula de carga)
e publica os dados como tópicos ROS2.

Tópicos publicados:
  /load_cell/voltage    std_msgs/Float32   tensão do sensor (V)
  /load_cell/force      std_msgs/Float32   força calibrada (N)
  /load_cell/calibrated std_msgs/Bool      True quando calibração existe

A calibração é lida de ~/.config/touch_pack/load_cell_calib.json
e recarregada automaticamente a cada 10 s (após a GUI salvar nova calib).

Payload UDP (little-endian, 8 bytes):
  float v_sensor       — tensão reconstruída após divisor (V)
  float force_filtered — força com calibração padrão da ESP32 (N)
"""

import json
import os
import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32, Bool

UDP_PORT    = 8080
CALIB_FILE  = os.path.expanduser('~/.config/touch_pack/load_cell_calib.json')
PAYLOAD_FMT = '<ff'
PAYLOAD_SZ  = struct.calcsize(PAYLOAD_FMT)   # 8 bytes

QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class ForceReceiverNode(Node):

    def __init__(self):
        super().__init__('force_receiver')

        self._voltage_pub = self.create_publisher(Float32, '/load_cell/voltage',    QOS_SENSOR)
        self._force_pub   = self.create_publisher(Float32, '/load_cell/force',      QOS_SENSOR)
        self._calib_pub   = self.create_publisher(Bool,    '/load_cell/calibrated', 10)

        self._slope:      float = 0.4490
        self._intercept:  float = 0.0017
        self._calibrated: bool  = False
        self._lock = threading.Lock()

        self._load_calib()
        self.create_timer(10.0, self._load_calib)

        self._running = True
        self._udp_thr = threading.Thread(
            target=self._udp_loop, daemon=True, name='udp-force-rx')
        self._udp_thr.start()

        self.get_logger().info(
            f'ForceReceiver: UDP :{UDP_PORT} | calibrado={self._calibrated}')

    # ──────────────────────────────────────────────────────────────────
    def _load_calib(self) -> None:
        try:
            with open(CALIB_FILE) as f:
                data = json.load(f)
            sl = float(data['slope'])
            ic = float(data['intercept'])
            with self._lock:
                changed = (sl != self._slope or ic != self._intercept
                           or not self._calibrated)
                self._slope      = sl
                self._intercept  = ic
                self._calibrated = True
            if changed:
                self.get_logger().info(
                    f'Calibração carregada: slope={sl:.4f} intercept={ic:.6f}')
        except FileNotFoundError:
            pass
        except Exception as exc:
            self.get_logger().warn(f'Falha ao carregar calibração: {exc}')

    # ──────────────────────────────────────────────────────────────────
    def _udp_loop(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)  # múltiplos nós no mesmo PC
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # aceitar broadcasts
        sock.settimeout(1.0)
        try:
            sock.bind(('', UDP_PORT))
        except OSError as exc:
            self.get_logger().error(
                f'Bind UDP :{UDP_PORT} falhou: {exc}')
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

            v_sensor, _ = struct.unpack(PAYLOAD_FMT, raw[:PAYLOAD_SZ])

            v_msg = Float32(); v_msg.data = float(v_sensor)
            self._voltage_pub.publish(v_msg)

            with self._lock:
                is_cal = self._calibrated
                sl     = self._slope
                ic     = self._intercept

            cb_msg = Bool(); cb_msg.data = is_cal
            self._calib_pub.publish(cb_msg)

            if abs(sl) > 1e-9:
                force = (v_sensor - ic) / sl
                f_msg = Float32(); f_msg.data = float(force)
                self._force_pub.publish(f_msg)

        sock.close()

    # ──────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ForceReceiverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
