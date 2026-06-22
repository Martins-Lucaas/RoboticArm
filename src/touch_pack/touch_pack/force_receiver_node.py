"""
force_receiver_node.py — Recebe pacotes UDP da ESP32 (célula de carga)
e publica os dados como tópicos ROS2.

Tópicos publicados:
  /load_cell/voltage    std_msgs/Float32   tensão do sensor (V)
  /load_cell/force      std_msgs/Float32   força calibrada (N, compressão = positivo)
  /load_cell/calibrated std_msgs/Bool      True quando calibração existe

A calibração é lida de ~/.config/touch_pack/load_cell_calib.json
e recarregada automaticamente a cada 10 s (após a GUI salvar nova calib).

A ESP32 amostra a 1 kHz e envia EM LOTE (LOAD_CELL_BATCH_N amostras por
datagrama, ~100 pacotes/s). Cada amostra (LOAD_CELL_SAMPLE_FMT '<IIf', 12 B):
  uint32 seq      — contador incremental por AMOSTRA; o salto revela amostras
                    perdidas (logado periodicamente).
  uint32 t_us     — micros() da ESP32 no instante da amostra (relógio de sync).
  float  v_sensor — tensão do sensor com filtro LEVE na ESP (só a média do
                    oversampling). O filtro PESADO (mediana + EMA dupla) e a
                    conversão tensão→força/calibração são feitos AQUI — nada é
                    hardcoded na ESP, e reajustar o filtro não exige reflashar.
"""

import json
import math
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
    LOAD_CELL_SAMPLE_FMT as SAMPLE_FMT,
    LOAD_CELL_BATCH_N as BATCH_N,
    LOAD_CELL_ESP_IP as ESP_IP,
    LOAD_CELL_DISCOVERY_PORT as DISCOVERY_PORT,
    LOAD_CELL_DISCOVERY_MAGIC as DISCOVERY_MAGIC,
)

SAMPLE_SZ = struct.calcsize(SAMPLE_FMT)   # 12 bytes: uint32 seq + uint32 t_us + float v

# ── Filtro PESADO (movido do firmware para cá) ───────────────────────────────
# A 1 kHz, parâmetros em AMOSTRAS escalam diferente do firmware antigo (100 Hz).
# A EMA dupla de cutoff FIXO (α=0.024) custava ~80 ms de atraso de grupo SEMPRE
# — inclusive numa excitação rápida, que chegava atrasada E atenuada. Trocada
# pelo filtro One-Euro (Casiez et al. 2012): cutoff ADAPTATIVO. Parado, o cutoff
# cai a ONE_EURO_MINCUTOFF (zero firme, sem jitter); quando o sinal se move
# rápido, o cutoff sobe e a latência despenca. Tudo ajustável aqui, sem
# reflashar a ESP (que só manda a tensão crua).
MEDIAN_N = 5                  # rejeita glitches isolados de 1–2 amostras (1–2 ms)
ONE_EURO_FREQ      = 1000.0   # Hz — taxa de amostragem efetiva (uma por elemento do lote)
ONE_EURO_MINCUTOFF = 1.0      # Hz — cutoff em repouso (↓ = zero mais firme, +lag parado)
ONE_EURO_BETA      = 0.05     # responsividade ao movimento (↑ = menos lag, +ruído)
ONE_EURO_DCUTOFF   = 1.0      # Hz — cutoff do estimador de derivada


class _LoadCellFilter:
    """Mediana de MEDIAN_N (mata spikes) seguida do filtro One-Euro — passa-baixa
    de cutoff adaptativo. Mantém a mesma rejeição de ruído em repouso que a EMA
    dupla antiga, mas com muito menos atraso quando há excitação rápida."""

    def __init__(self, freq: float = ONE_EURO_FREQ,
                 mincutoff: float = ONE_EURO_MINCUTOFF,
                 beta: float = ONE_EURO_BETA,
                 dcutoff: float = ONE_EURO_DCUTOFF,
                 median_n: int = MEDIAN_N):
        self._freq = freq
        self._mincutoff = mincutoff
        self._beta = beta
        self._dcutoff = dcutoff
        self._median_n = median_n
        self._median_buf: list[float] = []
        self._mi = 0
        self._x_prev = 0.0
        self._dx_prev = 0.0
        self._seeded = False

    @staticmethod
    def _alpha(cutoff: float, freq: float) -> float:
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau * freq)

    def update(self, v: float) -> float:
        if not self._seeded:
            self._median_buf = [v] * self._median_n
            self._x_prev = v
            self._dx_prev = 0.0
            self._seeded = True
            return v
        # Estágio 1: mediana — spikes isolados de 1–2 amostras não passam.
        self._median_buf[self._mi] = v
        self._mi = (self._mi + 1) % self._median_n
        v_med = sorted(self._median_buf)[self._median_n // 2]
        # Estágio 2: One-Euro — cutoff sobe com a velocidade estimada do sinal.
        dx = (v_med - self._x_prev) * self._freq
        a_d = self._alpha(self._dcutoff, self._freq)
        dx_hat = a_d * dx + (1.0 - a_d) * self._dx_prev
        cutoff = self._mincutoff + self._beta * abs(dx_hat)
        a = self._alpha(cutoff, self._freq)
        x_hat = a * v_med + (1.0 - a) * self._x_prev
        self._x_prev = x_hat
        self._dx_prev = dx_hat
        return x_hat

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

        # Filtro pesado em software (a ESP só manda o oversample leve agora).
        # Só a thread UDP o toca, então não precisa de lock.
        self._filter = _LoadCellFilter()

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
        # Buffer de recepção generoso: ~100 pacotes/s de lotes são minúsculos,
        # mas se a thread ROS engasgar por alguns ms o datagrama não é descartado
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

        rcvbuf = SAMPLE_SZ * BATCH_N + 64   # 1 lote cabe folgado
        while self._running and rclpy.ok():
            try:
                raw, _ = sock.recvfrom(max(256, rcvbuf))
            except socket.timeout:
                continue
            except OSError:
                break

            # Datagrama = N amostras concatenadas (12 B cada). Ignora um rabo
            # truncado se o tamanho não for múltiplo exato de SAMPLE_SZ.
            n_samples = len(raw) // SAMPLE_SZ
            if n_samples == 0:
                continue

            with self._lock:
                sl = self._slope
                ic = self._intercept

            for k in range(n_samples):
                off = k * SAMPLE_SZ
                (seq, _t_us, v_raw) = struct.unpack_from(SAMPLE_FMT, raw, off)
                self._track_seq(seq)

                # Filtro pesado no PC (a ESP só mandou o oversample leve).
                v_sensor = self._filter.update(float(v_raw))

                v_msg = Float32(); v_msg.data = v_sensor
                self._voltage_pub.publish(v_msg)

                if abs(sl) > 1e-9:
                    # Calibração feita em tração → invertido para a convenção do
                    # sistema: compressão = positivo, tração = negativo.
                    force = (ic - v_sensor) / sl
                    f_msg = Float32(); f_msg.data = float(force)
                    self._force_pub.publish(f_msg)

        sock.close()

    # ──────────────────────────────────────────────────────────────────
    # Salto de seq acima disto numa janela de 10 s (~10000 amostras a 1 kHz) não
    # é perda de rede plausível: trata como descontinuidade e re-ancora sem contar.
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
