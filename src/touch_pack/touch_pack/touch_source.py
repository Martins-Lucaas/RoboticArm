"""touch_source.py — Fonte de dados do sensor de toque (STM32) + figura
matplotlib reaproveitável.

Portado de ``sensors/Touch_sensor/touch_sensor.py`` para dentro do pacote
``touch_pack`` de modo que a GUI possa, no MESMO PC do STM32, ler a serial
diretamente e EMBUTIR os mesmos quatro gráficos (heatmap de tensão, raster
RA/SA, I_final e neurônio pós) sem precisar do plotter standalone.

Este módulo NÃO depende de ROS nem de Tkinter:
  • ``TouchSensorSource``  — abre a serial, parseia em uma thread de fundo e
    mantém o estado compartilhado sob lock. O callback ``on_sample`` é
    chamado a cada I_final (a GUI o usa para publicar /touch_sensor/value).
  • ``TouchFigure``        — constrói a ``matplotlib.figure.Figure`` com os
    quatro eixos e a atualiza a partir de um snapshot da fonte. A GUI a
    embute via ``FigureCanvasTkAgg`` e dispara ``canvas.draw_idle()``.
"""
from __future__ import annotations

import re
import socket
import struct
import threading
import time
from collections import deque
from typing import Callable, Optional

import numpy as np

from .constants import (
    TOUCH_SENSOR_UDP_PORT,
    TOUCH_PAYLOAD_FMT,
    TOUCH_UDP_BROADCAST_IP,
    TOUCH_FRAME_UDP_PORT,
)

try:
    import serial
    from serial.tools import list_ports
    _SERIAL_OK = True
except Exception:  # pragma: no cover - pyserial ausente
    serial = None
    list_ports = None
    _SERIAL_OK = False

# ── Configuração (espelha o firmware STM32) ──────────────────────────────
BAUD = 115200
# Grade DEFAULT (sensor 4×4). É o tamanho que o firmware 4×4 emite e o que a
# GUI assume quando o argumento de launch `sensor` não é informado. Cada
# TouchSensorSource/TouchFigure carrega a SUA própria grade (rows/cols), então
# o sensor 5×5 é só `TouchSensorSource(rows=5, cols=5, has_total=False)`; estes
# módulos-constantes seguem como fallback de import na palpation_gui.
ROWS = 4
COLS = 4
NUM_TAXELS = ROWS * COLS
VREF = 3.3
# Janela de tempo (s) do raster RA/SA e do neurônio pós — IGUAL ao plotter
# standalone (touch_sensor4x4.py/5x5.py): o eixo X mostra TEMPO ABSOLUTO e
# desliza a cada frame (xlim = agora-RASTER_WINDOW .. agora). 5 s reproduz o
# mesmo "look" do standalone, que o usuário considera a plotagem correta.
RASTER_WINDOW = 5.0
# Nº de amostras mostradas no painel escalar (I_final 4×4 / ativação média 5×5),
# plotadas sobre ÍNDICE DE AMOSTRA (arange) como no standalone — não sobre tempo.
WINDOW_SIZE = 50
# Backstop de memória do buffer de I_final (a poda real é por TEMPO, em
# _note_time); evita crescimento ilimitado caso só cheguem linhas TOTAL.
I_FINAL_MAXLEN = 20000

# Regex pré-compiladas: o parser roda em cada linha da serial.
RE_DATA = re.compile(r"idx=(\d+),adc=(\d+),t=(\d+)")
RE_IDX_T = re.compile(r"idx=(\d+),adc=\d+,t=(\d+)")
RE_POST = re.compile(r"t=(\d+)")
RE_TOTAL = re.compile(
    r"Iexc=([-+0-9.eE]+),Iinh=([-+0-9.eE]+),Ifinal=([-+0-9.eE]+)")


def detect_serial_port() -> Optional[str]:
    """Primeira porta USB/ACM disponível (STM32 via USB-CDC)."""
    if not _SERIAL_OK:
        return None
    candidates = [
        p.device for p in list_ports.comports()
        if "ACM" in p.device or "USB" in p.device
    ]
    return candidates[0] if candidates else None


class TouchSensorSource:
    """Leitor serial do touch sensor com estado compartilhado thread-safe."""

    def __init__(self, port: Optional[str] = None,
                 on_sample: Optional[Callable[[float], None]] = None,
                 udp_broadcast: bool = True,
                 udp_ip: str = TOUCH_UDP_BROADCAST_IP,
                 udp_port: int = TOUCH_SENSOR_UDP_PORT,
                 rows: int = ROWS, cols: int = COLS,
                 has_total: bool = True,
                 frame_relay: bool = False,
                 frame_port: int = TOUCH_FRAME_UDP_PORT,
                 on_raw_lines: Optional[Callable[[list], None]] = None):
        # port=None → auto-detect no start().
        self._port_req = port
        self.port: Optional[str] = None
        self._on_sample = on_sample
        # Tap das LINHAS BRUTAS do firmware (já fatiadas, sem o "\n"), chamado a
        # cada chunk FORA do lock. A GUI o usa para gravar os CSVs cru (ADC,
        # spikes RA/SA, cuneiformes) idênticos ao plotter standalone, sem
        # reparsing do estado agregado. Funciona no modo serial e no modo rede
        # com relay de frame (:8082), onde as mesmas linhas chegam.
        self._on_raw_lines = on_raw_lines

        # ── Modo de ingestão ──────────────────────────────────────────────
        # 'serial'  → lê o STM32 pela USB (start());
        # 'network' → recebe as linhas retransmitidas por UDP (start_network()),
        #             para um PC SEM USB exibir os mesmos gráficos.
        # None      → ainda não iniciado / parado.
        self.mode: Optional[str] = None
        # Relógio monotônico do PC no último dado REALMENTE parseado — usado pela
        # GUI para diferenciar "conectado e recebendo" de "conectado sem dados".
        self.last_rx: float = 0.0

        # ── Relay do frame completo (linhas brutas) p/ PCs remotos ─────────
        # No modo serial, além do escalar em :8081, retransmitimos as linhas do
        # firmware em :frame_port para que um PC sem USB reconstrua tudo.
        self._frame_relay = bool(frame_relay)
        self._frame_addr = (udp_ip, int(frame_port))
        self._frame_port = int(frame_port)
        self._frame_sock: Optional[socket.socket] = None
        self._net_sock: Optional[socket.socket] = None
        # Escuta do ESCALAR (:8081) no modo rede. Quando ESTE PC não tem a serial
        # mas um plotter standalone (ex.: touch_sensor5x5.py num PC Windows) só
        # transmite o escalar '<If' em :8081 — SEM o relay de frame em :8082 —, é
        # por aqui que a GUI recebe o sinal de toque. None fora do modo rede.
        self._scalar_sock: Optional[socket.socket] = None
        self._scalar_thread: Optional[threading.Thread] = None
        # Instante (monotônico) do último FRAME recebido em :8082. Serve para NÃO
        # duplicar o escalar: se o relay de frame está fresco, o :8082 já entrega
        # o I_final reconstruído e ignoramos o :8081 (evita publicar em dobro).
        self._last_frame_rx = 0.0

        # ── Grade do sensor (4×4 ou 5×5) ──────────────────────────────────
        # Cada instância carrega a sua grade; TODO o estado (matriz de tensões,
        # deques de spikes) e o parser usam self.rows/cols/num_taxels — nunca
        # mais os módulos-constantes. Assim o mesmo código serve 4×4 e 5×5.
        self.rows = int(rows)
        self.cols = int(cols)
        self.num_taxels = self.rows * self.cols
        # has_total: o firmware 4×4 envia a linha `TOTAL ... Ifinal=` (corrente
        # final do neurônio Izhikevich) a ~1 kHz, e é ESSE valor que vai pelo
        # UDP/ROS. O firmware 5×5 NÃO envia TOTAL; então, quando has_total=False,
        # sintetizamos um sinal de 1 kHz por FRAME do heatmap (média das tensões
        # dos taxels), emitido ao receber o último taxel de cada varredura. Ver
        # _parse_line/DATA e _frame_aggregate.
        self.has_total = bool(has_total)

        # Reemissão UDP do I_final a cada TOTAL — papel do plotter original.
        # O destino é o touch_receiver_node (broadcast :8081, formato '<If').
        self._udp_broadcast = udp_broadcast
        self._udp_addr = (udp_ip, udp_port)
        self._udp_sock: Optional[socket.socket] = None
        self._tx_seq = 0

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._ser = None
        self._thread: Optional[threading.Thread] = None
        self._buffer = ""

        self.connected = False
        self.error: Optional[str] = None

        # Estado compartilhado (protegido por self._lock).
        # Spikes em deques (não listas): a poda da janela é feita pela FRENTE,
        # O(1) por descarte, na própria thread serial — ver _note_time. Antes
        # a poda era O(N) DENTRO do snapshot(), na thread da GUI, e travava o
        # desenho sob carga (muitos spikes).
        # voltage_matrix = frame sendo PREENCHIDO (taxel a taxel, pode estar
        # parcial). voltage_frame = último frame COMPLETO (anti-tearing): só é
        # atualizado quando a varredura fecha (idx == num_taxels-1). A figura, a
        # gravação e o snapshot leem voltage_frame, então o heatmap nunca mostra
        # meia varredura (metade nova, metade antiga).
        self.voltage_matrix = np.zeros((self.rows, self.cols))
        self.voltage_frame = np.zeros((self.rows, self.cols))
        self.spike_times_RA = [deque() for _ in range(self.num_taxels)]
        self.spike_times_SA = [deque() for _ in range(self.num_taxels)]
        # Neurônio pós: o 4×4 manda UMA linha "POST" (neurônio único). O 5×5 manda
        # três cuneiformes (CN_SA/CN_RA/CN_MM) plotadas em LINHAS SEPARADAS, como
        # no standalone. Mantemos os dois: POST para o 4×4, as três para o 5×5.
        self.spike_times_POST: deque = deque()
        self.spike_times_CN_SA: deque = deque()
        self.spike_times_CN_RA: deque = deque()
        self.spike_times_CN_MM: deque = deque()
        # I_final agora é série TEMPORAL: guarda (t, valor) e desliza pela mesma
        # janela RASTER_WINDOW que os rasters (antes eram só os últimos N
        # valores, indexados por amostra, sem noção de tempo). t vem de
        # current_time no instante do TOTAL — o firmware não envia t na linha
        # TOTAL, então ancoramos no relógio mais recente do STM32.
        self.I_final_data: deque = deque(maxlen=I_FINAL_MAXLEN)
        self.current_time = 0.0
        self._last_evict = 0.0
        # Âncora p/ o relógio de DISPLAY: mapeia o tempo do STM32 (current_time)
        # para o relógio monotônico do PC. A GUI usa isso para deslizar a janela
        # SUAVEMENTE a cada frame (33 fps), mesmo quando os dados da serial
        # chegam em rajadas — sem isso a janela dava saltos a cada lote.
        self._t_ref_stm = 0.0
        self._t_ref_pc: Optional[float] = None

    # ──────────────────────────────────────────────────────────────────
    def start(self) -> bool:
        """Tenta abrir a serial e iniciar a thread de leitura.

        Retorna True em sucesso; em falha registra ``self.error`` e devolve
        False (a GUI continua funcionando em modo degradado, plotando zeros
        ou caindo para a subscrição ROS /touch_sensor/value)."""
        if not _SERIAL_OK:
            self.error = "pyserial não instalado"
            return False
        port = self._port_req or detect_serial_port()
        if not port:
            self.error = "nenhuma porta serial USB/ACM encontrada"
            return False
        try:
            self._ser = serial.Serial(port, BAUD, timeout=0.1)
        except Exception as exc:  # serial.SerialException e afins
            self.error = f"não foi possível abrir {port}: {exc}"
            return False
        self.port = port
        self.connected = True
        self.error = None
        self.mode = 'serial'
        # Socket de broadcast (best-effort): falha de rede aqui NÃO impede a
        # leitura serial nem a publicação ROS local — só desliga o reenvio UDP.
        if self._udp_broadcast:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                self._udp_sock = sock
            except Exception:
                self._udp_sock = None
        # Socket do relay do frame completo (linhas brutas) → PCs remotos.
        if self._frame_relay:
            try:
                fsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                fsock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                self._frame_sock = fsock
            except Exception:
                self._frame_sock = None
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._worker, daemon=True, name="touch-serial")
        self._thread.start()
        return True

    def start_network(self) -> bool:
        """Modo rede: ingere as linhas retransmitidas por UDP (PC SEM USB).

        Faz bind na porta do frame (:frame_port) e alimenta o MESMO parser que a
        serial, então a TouchFigure renderiza heatmap/rasters/pós idênticos sem
        nenhuma mudança. Best-effort: em falha registra ``self.error`` e devolve
        False (a GUI segue mostrando zeros / o escalar de /touch_sensor/value)."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except (AttributeError, OSError):
                pass
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.settimeout(1.0)
            sock.bind(('', self._frame_port))
        except OSError as exc:
            self.error = f"bind UDP :{self._frame_port} falhou: {exc}"
            return False
        self._net_sock = sock
        self.connected = True
        self.error = None
        self.mode = 'network'
        self._buffer = ""
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._net_worker, daemon=True, name="touch-net")
        self._thread.start()
        # Escuta best-effort do escalar em :8081 — cobre o plotter standalone que
        # só transmite o escalar (sem relay de frame em :8082).
        self._start_scalar_listener()
        return True

    def is_fresh(self, max_age: float = 1.0) -> bool:
        """True se chegou dado parseado nos últimos ``max_age`` s. Distingue
        'conectado e recebendo' de 'conectado/ligado mas sem dados' (porta serial
        errada, STM mudo, ou rede ligada sem ninguém transmitindo)."""
        return self.last_rx > 0.0 and (time.monotonic() - self.last_rx) < max_age

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._scalar_thread is not None:
            self._scalar_thread.join(timeout=1.5)
            self._scalar_thread = None
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        for attr in ('_udp_sock', '_frame_sock', '_net_sock', '_scalar_sock'):
            sk = getattr(self, attr)
            if sk is not None:
                try:
                    sk.close()
                except Exception:
                    pass
                setattr(self, attr, None)
        self.connected = False
        self.mode = None

    # ──────────────────────────────────────────────────────────────────
    def _ingest(self, text: str) -> tuple[list, list]:
        """Acumula ``text`` no buffer, fatia em linhas e parseia as COMPLETAS
        sob um único lock (batelado). Retorna (linhas_completas, pending), onde
        pending são os I_final a reenviar/entregar FORA do lock. Compartilhado
        pelas duas fontes (serial e rede) — ambas falam o mesmo protocolo."""
        self._buffer += text
        lines = self._buffer.split("\n")
        self._buffer = lines[-1]
        complete = lines[:-1]
        pending: list = []
        # Lock BATELADO: pega self._lock UMA vez por chunk em vez de por linha.
        # Com o firmware a ~1 kHz são ~16 mil linhas/s; travar/soltar o lock por
        # linha gerava contenção massiva com o snapshot() da GUI. Aqui o parse do
        # chunk inteiro roda sob um único lock; broadcast/ROS ficam para depois.
        with self._lock:
            for line in complete:
                # Uma linha malformada NUNCA pode derrubar a thread: engole o
                # erro da linha individual e segue.
                try:
                    self._parse_line(line.strip(), pending)
                except Exception:
                    pass
            if complete:
                self.last_rx = time.monotonic()
        return complete, pending

    def _worker(self) -> None:
        """Lê e parseia a serial continuamente até stop()."""
        ser = self._ser
        while not self._stop.is_set():
            try:
                chunk = ser.read(ser.in_waiting or 1)
            except Exception:
                self.connected = False
                self.error = "serial desconectada"
                break

            if not chunk:
                continue

            complete, pending = self._ingest(chunk.decode(errors="ignore"))
            # Tap das linhas brutas (gravador de CSV cru da GUI) — FORA do lock.
            if self._on_raw_lines is not None and complete:
                try:
                    self._on_raw_lines(complete)
                except Exception:
                    pass
            # Relay do frame completo (linhas brutas) p/ PCs remotos sem USB —
            # FORA do lock, e antes do escalar para minimizar latência do frame.
            if self._frame_relay:
                self._relay_lines(complete)
            # Reenvio UDP escalar (:8081) e callback ROS FORA do lock — rede/ROS
            # não podem segurar o lock de dados nem bloquear a leitura da serial.
            for i_final in pending:
                self._broadcast(i_final)
                if self._on_sample is not None:
                    try:
                        self._on_sample(i_final)
                    except Exception:
                        pass

    def _net_worker(self) -> None:
        """Modo rede: recebe os datagramas do relay e os injeta no parser.

        Não reenvia escalar nem retransmite (evita laços): só atualiza o estado
        para a figura e entrega o I_final via on_sample (publica ROS local)."""
        sock = self._net_sock
        while not self._stop.is_set():
            try:
                raw, _addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            if not raw:
                continue
            _complete, pending = self._ingest(raw.decode(errors="ignore"))
            if _complete:
                # Frame de :8082 chegou → marca fresco para o listener do escalar
                # (:8081) se calar e não duplicar o I_final.
                self._last_frame_rx = time.monotonic()
                # Tap das linhas brutas (gravador de CSV cru da GUI) — FORA do lock.
                if self._on_raw_lines is not None:
                    try:
                        self._on_raw_lines(_complete)
                    except Exception:
                        pass
            for i_final in pending:
                if self._on_sample is not None:
                    try:
                        self._on_sample(i_final)
                    except Exception:
                        pass

    def _start_scalar_listener(self) -> None:
        """Abre (best-effort) o socket do escalar em :8081 e dispara o worker.

        Só faz sentido no modo rede: quando o PC NÃO tem a serial, um plotter
        standalone (ex.: touch_sensor5x5.py) pode estar transmitindo apenas o
        escalar '<If' em :8081, sem o relay de frame em :8082. Falha de bind NÃO
        derruba o modo rede — apenas desliga esta fonte extra."""
        port = self._udp_addr[1]
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except (AttributeError, OSError):
                pass
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.settimeout(1.0)
            sock.bind(('', port))
        except OSError:
            self._scalar_sock = None
            return
        self._scalar_sock = sock
        self._scalar_thread = threading.Thread(
            target=self._net_scalar_worker, daemon=True, name="touch-net-scalar")
        self._scalar_thread.start()

    def _net_scalar_worker(self) -> None:
        """Modo rede: recebe o escalar '<If' em :8081 e o entrega via on_sample.

        Caminho usado quando um plotter standalone transmite SÓ o escalar (sem o
        relay de frame em :8082). Quando o relay de frame ESTÁ fresco, o :8082 já
        reconstrói e entrega o I_final — então ignoramos o :8081 para não publicar
        em dobro."""
        sock = self._scalar_sock
        if sock is None:
            return
        sz = struct.calcsize(TOUCH_PAYLOAD_FMT)
        while not self._stop.is_set():
            try:
                raw, _addr = sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(raw) < sz:
                continue
            # Relay de frame chegando há < 1 s → o :8082 já entrega o escalar.
            if (time.monotonic() - self._last_frame_rx) < 1.0:
                continue
            try:
                _seq, value = struct.unpack(TOUCH_PAYLOAD_FMT, raw[:sz])
            except struct.error:
                continue
            self.last_rx = time.monotonic()
            if self._on_sample is not None:
                try:
                    self._on_sample(float(value))
                except Exception:
                    pass

    def _relay_lines(self, lines: list) -> None:
        """Retransmite as linhas brutas do firmware por UDP (:frame_port).

        Empacota em datagramas de até ~1400 B (abaixo da MTU típica) cortando em
        fronteiras de linha, para evitar fragmentação IP. Best-effort: erro de
        rede é engolido para não derrubar a leitura serial."""
        sock = self._frame_sock
        if sock is None or not lines:
            return
        buf: list = []
        size = 0
        for ln in lines:
            b = (ln + "\n").encode("ascii", "ignore")
            if size + len(b) > 1400 and buf:
                try:
                    sock.sendto(b"".join(buf), self._frame_addr)
                except Exception:
                    pass
                buf, size = [], 0
            buf.append(b)
            size += len(b)
        if buf:
            try:
                sock.sendto(b"".join(buf), self._frame_addr)
            except Exception:
                pass

    def _note_time(self, t: float) -> None:
        """Registra o timestamp do firmware (só p/ o CSV) e PODA a janela por
        TEMPO REAL de chegada. Chamado SEMPRE sob ``self._lock``.

        Os instantes guardados nos buffers de spike/I_final são o relógio
        monotônico do PC no momento da chegada (ver _parse_line), NÃO o
        ``t`` do firmware. Por isso a janela desliza pelo relógio real e
        NUNCA precisamos limpar tudo: cada spike simplesmente envelhece e sai
        pela frente após RASTER_WINDOW s. Isso elimina o "reset" que ocorria
        quando o relógio do ADC e o dos spikes tinham bases diferentes e a
        antiga heurística ``t + RASTER_WINDOW < current_time`` zerava os
        buffers a cada frame do heatmap.

        ``current_time`` segue sendo o timestamp do STM32 mais recente, usado
        apenas por latest_voltages_and_time() para datar o CSV."""
        if t > self.current_time:
            self.current_time = t
        # Poda no máximo a ~50 Hz, por tempo real de chegada.
        now = time.monotonic()
        if now - self._last_evict < 0.02:
            return
        self._last_evict = now
        cutoff = now - RASTER_WINDOW
        for n in range(self.num_taxels):
            ra = self.spike_times_RA[n]
            while ra and ra[0] < cutoff:
                ra.popleft()
            sa = self.spike_times_SA[n]
            while sa and sa[0] < cutoff:
                sa.popleft()
        for cn in (self.spike_times_POST, self.spike_times_CN_SA,
                   self.spike_times_CN_RA, self.spike_times_CN_MM):
            while cn and cn[0] < cutoff:
                cn.popleft()
        ifd = self.I_final_data
        while ifd and ifd[0][0] < cutoff:
            ifd.popleft()

    def _parse_line(self, line: str, pending: list) -> None:
        """Parseia UMA linha e aplica ao estado. O CALLER já segura self._lock
        (lock batelado por chunk — ver _worker). I_final que chegar via TOTAL é
        empilhado em ``pending`` para broadcast/ROS fora do lock."""
        # ── ADC (protocolo do 5x5_base: frame inteiro numa linha) ────────
        # "ADC,v0,v1,...,vN-1,t=micros". Alimenta o heatmap e, no sensor sem
        # TOTAL (5×5), sintetiza o escalar por frame (média das tensões).
        if line.startswith("ADC"):
            parts = line.split(",")
            try:
                tstamp = int(parts[-1].replace("t=", "").strip()) / 1e6
            except (ValueError, IndexError):
                return
            vals = []
            for v in parts[1:-1]:
                v = v.strip()
                if v.isdigit():
                    vals.append(int(v))
            if len(vals) != self.num_taxels:
                return
            self._note_time(tstamp)
            self.voltage_matrix[:] = (
                np.array(vals).reshape(self.rows, self.cols) * (VREF / 4095.0))
            self.voltage_frame = self.voltage_matrix.copy()
            if not self.has_total:
                agg = self._frame_aggregate()
                self.I_final_data.append((time.monotonic(), agg))
                pending.append(agg)
            return

        if line.startswith("DATA"):
            m = RE_DATA.search(line)
            if m:
                idx = int(m.group(1))
                if not 0 <= idx < self.num_taxels:
                    return
                adc = int(m.group(2))
                tstamp = int(m.group(3)) / 1e6
                row, col = divmod(idx, self.cols)
                self._note_time(tstamp)
                self.voltage_matrix[row, col] = adc * (VREF / 4095.0)
                # Fim da varredura (último taxel): o frame está completo.
                if idx == self.num_taxels - 1:
                    # Publica o frame ESTÁVEL (anti-tearing): a figura/gravação
                    # leem voltage_frame, nunca a matriz parcial em preenchimento.
                    self.voltage_frame = self.voltage_matrix.copy()
                    # Sensor SEM linha TOTAL (5×5): o sinal de 1 kHz publicado em
                    # /touch_sensor/value é sintetizado por FRAME — média das
                    # tensões do frame recém-fechado, emitida aqui.
                    if not self.has_total:
                        agg = self._frame_aggregate()
                        self.I_final_data.append((time.monotonic(), agg))
                        pending.append(agg)

        # CN_* ANTES de RA/SA: "CN_RA"/"CN_SA" também começam com "CN", mas o
        # teste startswith("RA"/"SA") não os pega. Cada cuneiforme vai p/ sua
        # PRÓPRIA deque → 3 linhas no raster (CN_SA/CN_FA/CN_MM), como o standalone
        # (touch_sensor5x5_windows.py), em vez de colapsadas no neurônio pós.
        elif (line.startswith("CN_MM") or line.startswith("CN_RA")
              or line.startswith("CN_SA")):
            # Os plotters standalone registram o spike SÓ pelo prefixo da linha —
            # o firmware NÃO emite "t=" nas linhas CN. Exigir o regex aqui dropava
            # TODOS os spikes → raster preso em 0.00. Spike no prefixo (relógio do
            # PC); se houver "t=", usa só p/ datar o CSV/poda.
            m = RE_POST.search(line)
            if m:
                self._note_time(int(m.group(1)) / 1e6)
            now = time.monotonic()
            if line.startswith("CN_MM"):
                self.spike_times_CN_MM.append(now)
            elif line.startswith("CN_RA"):
                self.spike_times_CN_RA.append(now)
            else:  # CN_SA
                self.spike_times_CN_SA.append(now)

        elif line.startswith("RA"):
            m = RE_IDX_T.search(line)
            if m:
                idx = int(m.group(1))
                if not 0 <= idx < self.num_taxels:
                    return
                tstamp = int(m.group(2)) / 1e6
                self._note_time(tstamp)
                self.spike_times_RA[idx].append(time.monotonic())

        elif line.startswith("SA"):
            m = RE_IDX_T.search(line)
            if m:
                idx = int(m.group(1))
                if not 0 <= idx < self.num_taxels:
                    return
                tstamp = int(m.group(2)) / 1e6
                self._note_time(tstamp)
                self.spike_times_SA[idx].append(time.monotonic())

        elif line.startswith("POST"):
            # Igual ao 4×4 standalone: spike no prefixo, sem exigir "t=".
            m = RE_POST.search(line)
            if m:
                self._note_time(int(m.group(1)) / 1e6)
            self.spike_times_POST.append(time.monotonic())

        elif line.startswith("TOTAL"):
            m = RE_TOTAL.search(line)
            if m:
                try:
                    i_final = float(m.group(3))
                except ValueError:
                    return
                if not np.isfinite(i_final):
                    return
                self.I_final_data.append((time.monotonic(), i_final))
                pending.append(i_final)

    def _frame_aggregate(self) -> float:
        """Sinal escalar de 1 kHz para o sensor SEM linha TOTAL (5×5).

        Média das tensões dos taxels (V, faixa 0..VREF) — proxy da intensidade
        global de contato, derivado direto do heatmap que o firmware já manda.
        Bounded e estável; troque por np.sum se quiser a "carga" total em vez da
        média. Chamado SOB self._lock (a partir de _parse_line), logo após fechar
        o frame — usa voltage_frame (varredura completa)."""
        return float(self.voltage_frame.mean())

    def _broadcast(self, i_final: float) -> None:
        """Reemite o I_final por UDP a cada TOTAL (como o plotter original).

        Pacote '<If' = seq + valor, igual ao que o touch_receiver_node espera.
        Best-effort: qualquer erro de rede é engolido para não derrubar o
        parsing — UDP é não-confiável por natureza e o seq deixa o receptor
        detectar perdas."""
        sock = self._udp_sock
        if sock is None:
            return
        try:
            packet = struct.pack(TOUCH_PAYLOAD_FMT, self._tx_seq & 0xFFFFFFFF,
                                 float(i_final))
            self._tx_seq += 1
            sock.sendto(packet, self._udp_addr)
        except Exception:
            pass

    # ──────────────────────────────────────────────────────────────────
    def latest_voltages(self) -> np.ndarray:
        """Cópia barata só da matriz de tensões (rows×cols), sob lock. Para o
        gravador a 1 kHz, que não precisa do snapshot completo (spikes etc.) e
        não pode pagar a cópia O(N) das listas a cada amostra. Devolve o último
        frame COMPLETO (voltage_frame), não a matriz parcial em preenchimento."""
        with self._lock:
            return self.voltage_frame.copy()

    def latest_voltages_and_time(self) -> tuple[np.ndarray, float]:
        """Tensões (rows×cols) + o timestamp do STM32 (current_time, s) da
        última amostra, ambos sob o MESMO lock. O gravador usa o relógio do
        firmware (micros() a 1 kHz) na planilha, em vez do relógio do PC, para
        que cada linha do CSV carregue o instante REAL da amostra de 1 ms.
        Usa voltage_frame (último frame completo) — sem tearing no CSV."""
        with self._lock:
            return self.voltage_frame.copy(), float(self.current_time)

    def snapshot(self) -> dict:
        """Retrato consistente do estado para renderização. As listas já vêm
        podadas à janela pela thread serial (_note_time); aqui só COPIAMOS sob
        o lock — sem custo O(N) de filtragem na thread da GUI (era o que
        travava o desenho sob carga). O desenho em si roda fora do lock."""
        with self._lock:
            # Base de tempo do desenho = relógio REAL do PC (monotônico), o mesmo
            # carimbado em cada spike/I_final na chegada. A janela desliza suave
            # a cada frame mesmo sem dado novo, e os spikes saem por idade real.
            t_now = time.monotonic()
            t_disp = t_now
            volt = self.voltage_frame.copy()
            ra = [list(lst) for lst in self.spike_times_RA]
            sa = [list(lst) for lst in self.spike_times_SA]
            post = list(self.spike_times_POST)
            cn_sa = list(self.spike_times_CN_SA)
            cn_ra = list(self.spike_times_CN_RA)
            cn_mm = list(self.spike_times_CN_MM)
            i_final = list(self.I_final_data)  # lista de (t, valor)
            latest_v = float(self.I_final_data[-1][1]) if self.I_final_data else 0.0
        return {
            "t_now": t_now,
            "t_disp": t_disp,
            "volt": volt,
            "ra": ra,
            "sa": sa,
            "post": post,
            "cn_sa": cn_sa,
            "cn_ra": cn_ra,
            "cn_mm": cn_mm,
            "i_final": i_final,
            "latest_i_final": latest_v,
        }


def _offsets(xs, ys):
    """Empacota pares (x, y) no formato (N, 2) exigido por set_offsets."""
    if len(xs) == 0:
        return np.empty((0, 2))
    return np.column_stack((xs, ys))


class TouchFigure:
    """Figura matplotlib com os quatro gráficos do touch sensor, atualizada
    a partir de um snapshot da TouchSensorSource. Use ``self.fig`` com
    FigureCanvasTkAgg e chame ``update()`` seguido de ``canvas.draw_idle()``.
    """

    def __init__(self, source: TouchSensorSource, *, facecolor: str = "white"):
        from matplotlib.figure import Figure

        self.source = source
        # Grade vinda da fonte (4×4 ou 5×5) — a figura inteira (heatmap, raster,
        # textos) é construída a partir destes, não dos módulos-constantes.
        self.rows = source.rows
        self.cols = source.cols
        self.num_taxels = source.num_taxels
        self.has_total = source.has_total
        self.fig = Figure(figsize=(9.5, 7.0), dpi=100, facecolor=facecolor)
        axs = self.fig.subplots(2, 2)
        ax1, ax2 = axs[0, 0], axs[0, 1]
        ax5, ax6 = axs[1, 0], axs[1, 1]
        self.ax_heat, self.ax_raster = ax1, ax2
        self.ax_ifinal, self.ax_post = ax5, ax6

        # Estilo do plotter STANDALONE (touch_sensor4x4.py/5x5.py), que o
        # usuário considera a plotagem correta: o eixo X do raster/pós mostra
        # TEMPO ABSOLUTO em segundos (relógio do STM32) e DESLIZA a cada frame
        # (xlim = agora-RASTER_WINDOW .. agora). Isso exige redraw completo
        # (blit=False na FuncAnimation da GUI) — a poda dos buffers já foi
        # movida para a thread serial (_note_time), então o desenho é barato.

        # ── Heatmap ───────────────────────────────────────────────────
        # interpolation="bicubic": mesmo visual suave do plotter standalone.
        # Troque para "nearest" se quiser ver o valor REAL de cada taxel (sem
        # gradiente interpolado) e ganhar desempenho.
        self.im_volt = ax1.imshow(
            np.zeros((self.rows, self.cols)), cmap="jet",
            interpolation="bicubic", vmin=0, vmax=VREF)
        self.fig.colorbar(self.im_volt, ax=ax1)
        ax1.set_title(f"Tensão (0–3.3 V) — {self.rows}×{self.cols}")
        ax1.set_xticks(range(self.cols))
        ax1.set_yticks(range(self.rows))
        self.texts_volt = [[
            ax1.text(c, r, "0", ha="center", va="center",
                     fontsize=8, color="white")
            for c in range(self.cols)] for r in range(self.rows)]

        # ── Raster RA / SA (janela deslizante, eixo de tempo absoluto) ──
        ax2.set_title("Raster RA / SA")
        # Eixo de tempo RELATIVO e fixo (0 s = agora, -W = mais antigo): os
        # spikes deslizam da direita p/ a esquerda e saem por idade real.
        ax2.set_xlim(-RASTER_WINDOW, 0)
        ax2.set_ylim(-1, self.num_taxels * 2)
        ax2.set_xlabel("t - agora (s)")
        self.scatter_RA = ax2.scatter([], [], s=10, color="red", label="RA")
        self.scatter_SA = ax2.scatter([], [], s=10, color="blue", label="SA")
        ax2.legend(loc="upper right", fontsize=8)

        # ── Sinal escalar (últimas WINDOW_SIZE amostras sobre índice) ──
        # Como no standalone: o painel mostra as últimas WINDOW_SIZE amostras
        # sobre o ÍNDICE da amostra (eixo X fixo 0..WINDOW_SIZE), não sobre
        # tempo. 4×4: I_final (corrente do neurônio, ±1000). 5×5: sem TOTAL,
        # mostra o I_final sintetizado por frame (média das tensões, V) — ver
        # TouchSensorSource._frame_aggregate.
        self._x_fixed = np.arange(WINDOW_SIZE)
        (self.line_I_final,) = ax5.plot(
            self._x_fixed, np.zeros(WINDOW_SIZE), lw=2)
        if self.has_total:
            ax5.set_title("I_final")
            ax5.set_ylim(-1000, 1000)
        else:
            # 5×5 não tem linha TOTAL: o escalar é a média das tensões do frame
            # (ver _frame_aggregate). Rotulamos "I_final" como no plotter
            # standalone (touch_sensor5x5_windows.py) — o usuário não quer o
            # nome "Ativação média".
            ax5.set_title("I_final")
            ax5.set_ylabel("Mean Voltage (V)")
            ax5.set_ylim(0, VREF)
        ax5.set_xlim(0, WINDOW_SIZE)

        # ── Neurônio pós / cuneiformes (janela deslizante) ──────────────
        # 4×4: neurônio pós único (linha "POST"). 5×5: três cuneiformes em LINHAS
        # SEPARADAS (CN_SA=1, CN_FA=2, CN_MM=3), iguais ao standalone.
        ax6.set_xlim(-RASTER_WINDOW, 0)
        ax6.set_xlabel("t - agora (s)")
        if self.has_total:
            ax6.set_title("Neurônio Pós")
            ax6.set_ylim(-1, 1)
            self.scatter_POST = ax6.scatter([], [], s=20, color="black")
            self.scatter_CN_SA = self.scatter_CN_RA = self.scatter_CN_MM = None
        else:
            ax6.set_title("Raster Cuneiforme")
            ax6.set_ylim(0.3, 3.7)
            ax6.set_yticks([1, 2, 3])
            ax6.set_yticklabels(["CN_SA", "CN_FA", "CN_MM"])
            self.scatter_POST = None
            self.scatter_CN_SA = ax6.scatter([], [], s=14, color="#006B6B")
            self.scatter_CN_RA = ax6.scatter([], [], s=14, color="#8C2D5D")
            self.scatter_CN_MM = ax6.scatter([], [], s=14, color="#3D4F8A")

        try:
            self.fig.tight_layout()
        except Exception:
            pass

        # Artistas que mudam a cada frame. Com blit=False (estilo standalone) a
        # figura inteira é redesenhada — então os limites de eixo podem deslizar
        # em tempo absoluto a cada frame. A lista ainda é devolvida por update()
        # para compatibilidade com a FuncAnimation.
        post_artists = ([self.scatter_POST] if self.has_total
                        else [self.scatter_CN_SA, self.scatter_CN_RA,
                              self.scatter_CN_MM])
        self.blit_artists = [
            self.im_volt,
            self.scatter_RA,
            self.scatter_SA,
            self.line_I_final,
        ] + post_artists + [t for row in self.texts_volt for t in row]

    # ──────────────────────────────────────────────────────────────────
    def init_blit(self) -> list:
        """Estado inicial da FuncAnimation (init_func)."""
        self.scatter_RA.set_offsets(np.empty((0, 2)))
        self.scatter_SA.set_offsets(np.empty((0, 2)))
        if self.has_total:
            self.scatter_POST.set_offsets(np.empty((0, 2)))
        else:
            self.scatter_CN_SA.set_offsets(np.empty((0, 2)))
            self.scatter_CN_RA.set_offsets(np.empty((0, 2)))
            self.scatter_CN_MM.set_offsets(np.empty((0, 2)))
        self.line_I_final.set_data(self._x_fixed, np.zeros(WINDOW_SIZE))
        return self.blit_artists

    # ──────────────────────────────────────────────────────────────────
    def update(self, *_frame) -> list:
        snap = self.source.snapshot()
        # Tempo RELATIVO ao agora (relógio real do PC): cada spike é plotado em
        # x = t_chegada - now_t, no intervalo [-RASTER_WINDOW, 0]. O xlim é fixo
        # (definido no __init__), então a janela "desliza" porque os dados se
        # movem, não o eixo — acumulam e saem pela esquerda por idade real.
        now_t = snap["t_now"]

        # Heatmap (rot90 ×2 = mesma orientação física do plotter standalone).
        # IMPORTANTE: os números sobrepostos usam a MESMA matriz rotacionada que
        # a imagem (vr), senão o número de cada célula mostra um taxel e a cor
        # mostra outro (bug do standalone: imagem rotacionada, texto não).
        volt = snap["volt"]
        vr = np.rot90(volt, 2)
        self.im_volt.set_data(vr)
        for r in range(self.rows):
            for c in range(self.cols):
                self.texts_volt[r][c].set_text(f"{vr[r, c]:.2f}")

        # Raster RA / SA — timestamps ABSOLUTOS; o xlim desliza com now_t.
        x_ra, y_ra = [], []
        ra = snap["ra"]
        for n in range(self.num_taxels):
            for t in ra[n]:
                x_ra.append(t - now_t)
                y_ra.append(n)
        x_sa, y_sa = [], []
        sa = snap["sa"]
        for n in range(self.num_taxels):
            for t in sa[n]:
                x_sa.append(t - now_t)
                y_sa.append(n + self.num_taxels)
        self.scatter_RA.set_offsets(_offsets(x_ra, y_ra))
        self.scatter_SA.set_offsets(_offsets(x_sa, y_sa))

        # Neurônio pós (4×4) ou cuneiformes em 3 linhas (5×5) — mesmo eixo
        # relativo do raster.
        if self.has_total:
            x_post = [t - now_t for t in snap["post"]]
            self.scatter_POST.set_offsets(_offsets(x_post, [0] * len(x_post)))
        else:
            for scat, key, y in (
                    (self.scatter_CN_SA, "cn_sa", 1),
                    (self.scatter_CN_RA, "cn_ra", 2),
                    (self.scatter_CN_MM, "cn_mm", 3)):
                xs = [t - now_t for t in snap[key]]
                scat.set_offsets(_offsets(xs, [y] * len(xs)))

        # Painel escalar — últimas WINDOW_SIZE amostras sobre ÍNDICE (como o
        # standalone): pega os valores mais recentes do buffer e os alinha à
        # DIREITA do eixo fixo 0..WINDOW_SIZE, preenchendo o início com 0 quando
        # ainda há menos de WINDOW_SIZE amostras.
        ys = [v for (_t, v) in snap["i_final"]][-WINDOW_SIZE:]
        if len(ys) < WINDOW_SIZE:
            ys = [0.0] * (WINDOW_SIZE - len(ys)) + ys
        self.line_I_final.set_data(self._x_fixed, ys)

        return self.blit_artists
