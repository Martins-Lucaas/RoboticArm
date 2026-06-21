"""
force_receiver_node.py — Recebe pacotes UDP da ESP32 (célula de carga)
e publica os dados como tópicos ROS2.

Tópicos publicados:
  /load_cell/voltage    std_msgs/Float32   tensão do sensor (V)
  /load_cell/force      std_msgs/Float32   força calibrada (N, compressão = positivo)
  /load_cell/calibrated std_msgs/Bool      True quando calibração existe

A calibração é lida de ~/.config/touch_pack/load_cell_calib.json
e recarregada automaticamente a cada 10 s (após a GUI salvar nova calib).

Payload UDP (little-endian, 8 bytes — LOAD_CELL_PAYLOAD_FMT '<If'):
  uint32 seq     — contador incremental da ESP32; o salto do seq revela
                   pacotes perdidos na rede (logado periodicamente).
  float  v_sensor — tensão do sensor já filtrada na ESP32 (V).
                   A conversão tensão→força e a calibração são feitas
                   AQUI, a partir de load_cell_calib.json — nada é
                   hardcoded na ESP.
"""

import json
import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32, Bool

from .constants import (
    LOAD_CELL_UDP_PORT as UDP_PORT,
    LC_CALIB_FILE as CALIB_FILE,
    LOAD_CELL_PAYLOAD_FMT as PAYLOAD_FMT,
    LOAD_CELL_ESP_IP as ESP_IP,
    LOAD_CELL_DISCOVERY_PORT as DISCOVERY_PORT,
    LOAD_CELL_DISCOVERY_MAGIC as DISCOVERY_MAGIC,
)

PAYLOAD_SZ  = struct.calcsize(PAYLOAD_FMT)   # 8 bytes: uint32 seq + float v

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

        # Detecção de perda de pacotes via seq da ESP32.
        self._last_seq:  int | None = None
        self._lost_pkts: int = 0
        self._rx_pkts:   int = 0
        self._seq_resets: int = 0
        self.create_timer(10.0, self._report_packet_loss)

        # Auto-descoberta: anuncia este host ao ESP (IP fixo) para receber a
        # telemetria por UNICAST em vez de broadcast (que perde ~30% no WiFi).
        # O ESP grava nosso IP e responde unicast; renovamos a cada 2 s para
        # que, se o ESP reiniciar (OTA/boot), ele reaprenda o destino. Fallback:
        # se o hello não chega, o firmware volta sozinho ao broadcast.
        self._disc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._send_discovery()
        self.create_timer(2.0, self._send_discovery)

        self._load_calib()
        self.create_timer(10.0, self._load_calib)
        # Flag de calibração a 1 Hz (antes era publicada a cada pacote UDP,
        # ~50 Hz — desperdício para um Bool que quase nunca muda).
        self.create_timer(1.0, self._publish_calibrated)

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
    def _send_discovery(self) -> None:
        """Envia o 'hello' ao ESP (IP fixo) para que ele nos mande unicast."""
        try:
            self._disc_sock.sendto(DISCOVERY_MAGIC, (ESP_IP, DISCOVERY_PORT))
        except OSError:
            pass  # rede fora do ar: o firmware mantém broadcast no fallback

    # ──────────────────────────────────────────────────────────────────
    def _publish_calibrated(self) -> None:
        with self._lock:
            is_cal = self._calibrated
        msg = Bool(); msg.data = is_cal
        self._calib_pub.publish(msg)

    # ──────────────────────────────────────────────────────────────────
    def _udp_loop(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)  # múltiplos nós no mesmo PC
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # aceitar broadcasts
        # Buffer de recepção generoso: a 100 Hz × 8 B o tráfego é minúsculo, mas
        # se a thread ROS engasgar por alguns ms o datagrama não é descartado
        # pelo kernel (uma das fontes de "perda" que não é da rede).
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        except OSError:
            pass
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

            (seq, v_sensor) = struct.unpack(PAYLOAD_FMT, raw[:PAYLOAD_SZ])
            self._track_seq(seq)

            v_msg = Float32(); v_msg.data = float(v_sensor)
            self._voltage_pub.publish(v_msg)

            with self._lock:
                sl = self._slope
                ic = self._intercept

            if abs(sl) > 1e-9:
                # Calibração feita em tração → invertido para a convenção do
                # sistema: compressão = positivo, tração = negativo.
                force = (ic - v_sensor) / sl
                f_msg = Float32(); f_msg.data = float(force)
                self._force_pub.publish(f_msg)

        sock.close()

    # ──────────────────────────────────────────────────────────────────
    # Salto de seq acima disto numa janela de 10 s (~1000 pkt a 100 Hz) não é
    # perda de rede plausível: trata como descontinuidade e re-ancora sem contar.
    _MAX_PLAUSIBLE_GAP = 5000

    def _track_seq(self, seq: int) -> None:
        """Contabiliza pacotes perdidos pelo salto do seq.

        ``struct.unpack('<I')`` já devolve um int Python 0..2³²-1, então o
        delta é calculado COM SINAL, sem máscara. Um reset do ESP (seq volta a
        0 após boot/OTA) dá delta negativo → re-ancora, NÃO conta como perda.
        O código antigo mascarava com ``& 0xFFFFFFFF`` e o reset virava um salto
        de ~4 bilhões: era a origem do '100% / 4294714580' espúrio no log.

        Roda na thread UDP; os contadores são lidos/zerados pelo timer em outra
        thread, então mexe neles sob o mesmo lock."""
        with self._lock:
            self._rx_pkts += 1
            if self._last_seq is not None:
                delta = seq - self._last_seq
                if delta <= 0:
                    # Reset (boot/OTA) ou pacote duplicado/fora de ordem.
                    self._seq_resets += 1
                elif delta <= self._MAX_PLAUSIBLE_GAP:
                    self._lost_pkts += delta - 1
                # delta enorme e positivo: descontinuidade — ignora p/ não inflar.
            self._last_seq = seq

    def _report_packet_loss(self) -> None:
        with self._lock:
            rx, lost, resets = self._rx_pkts, self._lost_pkts, self._seq_resets
            self._rx_pkts = 0
            self._lost_pkts = 0
            self._seq_resets = 0
        if resets:
            self.get_logger().info(
                f'ESP32 reiniciou {resets}× nos últimos 10 s (seq reancorado) — '
                'provável boot/OTA, não é perda de rede.')
        if rx == 0 or lost == 0:
            return
        total = rx + lost
        pct = 100.0 * lost / total if total else 0.0
        self.get_logger().warn(
            f'Perda de pacotes UDP: {lost}/{total} '
            f'({pct:.1f}%) nos últimos 10 s')

    # ──────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self._running = False
        try:
            self._disc_sock.close()
        except Exception:
            pass
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
