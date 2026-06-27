"""touch_sensor5x5_windows.py — Plotter do touch sensor 5x5 (Windows).

Mesmo padrao do 5x5_base.py:
  • protocolo ADC / RA / SA / CN_MM / CN_RA / CN_SA;
  • 4 graficos com estilo de artigo (heatmap, I_final, raster FA/SA,
    cuneiformes coloridas);
  • janela deslizante por TEMPO REAL de chegada (time.monotonic) — acumula e
    sai sozinha, sem o "reset" do antigo note_time.

Alem de plotar, RETRANSMITE por UDP para o PC do ROS:
  • 8082 (frame): as linhas BRUTAS do firmware, para a GUI (touch_pack,
    TouchSensorSource.start_network) reconstruir heatmap/rasters/pos identicos;
  • 8081 (escalar): o I_final '<If' por frame, para o touch_receiver_node.

A leitura serial + relay rodam numa thread separada do desenho, entao o envio
de rede nao trava a plotagem (relay loteado e best-effort).
"""
import argparse
import re
import sys
import threading
import time
import serial
from serial.tools import list_ports
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.ticker import AutoMinorLocator
from collections import deque
import socket
import struct

# =========================================================
# CONFIG
# =========================================================

# Porta serial default do STM32 no Windows. None = autodeteccao (primeiro COM).
DEFAULT_PORT = None
BAUD = 115200

ROWS = 5
COLS = 5
NUM_TAXELS = ROWS * COLS

VREF = 3.3

RASTER_WINDOW = 5.0     # janela (s) do raster/cuneiformes
WINDOW_SIZE = 50        # amostras mostradas no painel I_final

parser = argparse.ArgumentParser(
    description="Plotter STM32 5x5 (Windows) + relay UDP (8082 frame / 8081 escalar)"
)
parser.add_argument("--udp-ip", default="192.168.5.255",
                    help="destino broadcast dos pacotes UDP (rede do PC do ROS)")
parser.add_argument("--frame-port", type=int, default=8082,
                    help="porta do relay de FRAME/linhas brutas (TOUCH_FRAME_UDP_PORT)")
parser.add_argument("--scalar-port", type=int, default=8081,
                    help="porta do escalar I_final (TOUCH_SENSOR_UDP_PORT)")
parser.add_argument("--port", default=DEFAULT_PORT,
                    help="porta serial do STM32 (ex.: COM5). Se omitida, autodetecta.")
cli_args = parser.parse_args()

UDP_IP = cli_args.udp_ip
FRAME_PORT = cli_args.frame_port
SCALAR_PORT = cli_args.scalar_port

# Sockets de broadcast (best-effort). Um para o frame (8082) e um para o
# escalar (8081). Falha de rede NUNCA derruba a leitura/plotagem.
def _make_bcast_socket():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    return s

frame_sock = _make_bcast_socket()
scalar_sock = _make_bcast_socket()
udp_seq = 0

# =========================================================
# SERIAL
# =========================================================

def detect_serial_port():
    candidates = [p.device for p in list_ports.comports()
                  if p.device.upper().startswith("COM")]
    return candidates[0] if candidates else None


PORT = cli_args.port or detect_serial_port()

if PORT is None:
    disponiveis = ", ".join(p.device for p in list_ports.comports()) or "nenhuma"
    sys.exit(
        "Nenhuma porta serial COM encontrada.\n"
        f"Portas disponiveis: {disponiveis}\n"
        "Conecte o STM32 ou informe a porta com --port (ex.: --port COM5)."
    )

try:
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
except serial.SerialException as e:
    sys.exit(f"Nao foi possivel abrir a porta serial '{PORT}': {e}")

print(f"Serial conectada em {PORT} @ {BAUD} baud")
print(f"Relay UDP: frame -> {UDP_IP}:{FRAME_PORT} | escalar -> {UDP_IP}:{SCALAR_PORT}")

# =========================================================
# ESTADO COMPARTILHADO (thread serial <-> desenho)
# =========================================================

voltage_matrix = np.zeros((ROWS, COLS))
I_final_data = deque([0.0] * WINDOW_SIZE, maxlen=WINDOW_SIZE)
spike_times_RA = [deque() for _ in range(NUM_TAXELS)]
spike_times_SA = [deque() for _ in range(NUM_TAXELS)]
spike_times_CN_MM = deque()
spike_times_CN_RA = deque()
spike_times_CN_SA = deque()

data_lock = threading.Lock()
reader_running = True
last_rx_time = 0.0

# =========================================================
# ESTILO DE ARTIGO
# =========================================================

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Liberation Serif", "DejaVu Serif"],
    "font.size": 9,
    "axes.titlesize": 10,
    "axes.labelsize": 9,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "legend.fontsize": 8,
    "axes.linewidth": 0.8,
})

COR_RA = "#0B3954"
COR_SA = "#B24C3D"
COR_CNMM = "#3D4F8A"
COR_CNRA = "#8C2D5D"
COR_CNSA = "#006B6B"
COR_TENSAO_MEDIA = "#613c4c"


def configurar_eixo_artigo(ax):
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_linewidth(0.8)
    ax.spines["bottom"].set_linewidth(0.8)
    ax.tick_params(which="major", direction="in", length=3, width=0.8,
                   top=False, right=False)
    ax.tick_params(which="minor", direction="in", length=1.8, width=0.5,
                   top=False, right=False)
    ax.xaxis.set_minor_locator(AutoMinorLocator())
    ax.yaxis.set_minor_locator(AutoMinorLocator(2))
    ax.grid(False)


# =========================================================
# RELAY UDP (rodam na thread serial; best-effort)
# =========================================================

def relay_lines(lines):
    """Retransmite as linhas BRUTAS em 8082, loteadas (<=1400 B) p/ evitar
    fragmentacao. Erro de rede e engolido para nao travar a leitura/desenho."""
    buf, size = [], 0
    for ln in lines:
        b = (ln + "\n").encode("ascii", "ignore")
        if size + len(b) > 1400 and buf:
            try:
                frame_sock.sendto(b"".join(buf), (UDP_IP, FRAME_PORT))
            except OSError:
                pass
            buf, size = [], 0
        buf.append(b)
        size += len(b)
    if buf:
        try:
            frame_sock.sendto(b"".join(buf), (UDP_IP, FRAME_PORT))
        except OSError:
            pass


def send_scalar(value):
    global udp_seq
    try:
        pkt = struct.pack("<If", udp_seq & 0xFFFFFFFF, float(value))
        udp_seq += 1
        scalar_sock.sendto(pkt, (UDP_IP, SCALAR_PORT))
    except OSError:
        pass


# =========================================================
# PARSING (sob data_lock)
# =========================================================

_RE_IDX_PAT = re.compile(r"idx=(\d+)")


def _RE_IDX(line):
    m = _RE_IDX_PAT.search(line)
    return int(m.group(1)) if m else None


def parse_line(line, scalars, now):
    line = line.strip()
    if not line:
        return

    if line.startswith("ADC"):
        parts = line.split(",")
        try:
            vals = [int(v.strip()) for v in parts[1:-1] if v.strip().isdigit()]
        except ValueError:
            return
        if len(vals) != NUM_TAXELS:
            return
        voltage_matrix[:] = np.array(vals).reshape(ROWS, COLS) * (VREF / 4095.0)
        agg = float(voltage_matrix.mean())
        I_final_data.append(agg)
        scalars.append(agg)

    elif line.startswith("CN_MM"):
        spike_times_CN_MM.append(now)
    elif line.startswith("CN_RA"):
        spike_times_CN_RA.append(now)
    elif line.startswith("CN_SA"):
        spike_times_CN_SA.append(now)

    elif line.startswith("RA"):
        m = _RE_IDX(line)
        if m is not None and 0 <= m < NUM_TAXELS:
            spike_times_RA[m].append(now)
    elif line.startswith("SA"):
        m = _RE_IDX(line)
        if m is not None and 0 <= m < NUM_TAXELS:
            spike_times_SA[m].append(now)


# =========================================================
# THREAD DE LEITURA SERIAL + RELAY
# =========================================================

def serial_reader_loop():
    global last_rx_time
    buf = ""
    while reader_running:
        try:
            chunk = ser.read(ser.in_waiting or 1)
        except serial.SerialException as exc:
            print(f"[ERRO] leitura serial: {exc}")
            break
        if not chunk:
            continue
        buf += chunk.decode(errors="ignore")
        parts = buf.split("\n")
        buf = parts[-1]
        complete = parts[:-1]
        if not complete:
            continue
        # Relay das linhas brutas (8082) FORA do lock.
        relay_lines(complete)
        scalars = []
        now = time.monotonic()
        with data_lock:
            for line in complete:
                parse_line(line, scalars, now)
        # Escalar (8081) FORA do lock.
        for s in scalars:
            send_scalar(s)
        last_rx_time = time.time()


# =========================================================
# FIGURA (estilo artigo)
# =========================================================

fig, axs = plt.subplots(2, 2, figsize=(14, 10))
ax1, ax2, ax5, ax6 = axs[0, 0], axs[0, 1], axs[1, 0], axs[1, 1]

# ---- HEATMAP ----
im_volt = ax1.imshow(voltage_matrix, cmap="jet", interpolation="bicubic",
                     vmin=0, vmax=VREF)
plt.colorbar(im_volt, ax=ax1)
ax1.set_title("Voltage Heatmap — 5x5")
ax1.set_xticks(range(COLS))
ax1.set_yticks(range(ROWS))
texts_volt = [[ax1.text(c, r, "0", ha="center", va="center", fontsize=8,
                        color="white")
               for c in range(COLS)] for r in range(ROWS)]

# ---- RASTER FA / SA (SA embaixo 1..25, FA em cima 26..50) ----
ax2.set_title("FA and SA Spike Raster")
ax2.set_xlim(-RASTER_WINDOW, 0)
ax2.set_ylim(0.3, NUM_TAXELS * 2 + 0.7)
ax2.set_xlabel("t - agora (s)")
ax2.set_yticks([5, 10, 15, 20, 25, 30, 35, 40, 45, 50])
ax2.set_yticklabels(["SA5", "SA10", "SA15", "SA20", "SA25",
                     "FA5", "FA10", "FA15", "FA20", "FA25"])
configurar_eixo_artigo(ax2)
scatter_SA = ax2.scatter([], [], s=9, color=COR_SA)
scatter_RA = ax2.scatter([], [], s=9, color=COR_RA)
ax2.text(-0.12, NUM_TAXELS * 0.5, "SA", transform=ax2.get_yaxis_transform(),
         fontweight="bold", color=COR_SA, ha="right", va="center")
ax2.text(-0.12, NUM_TAXELS * 1.5, "FA", transform=ax2.get_yaxis_transform(),
         fontweight="bold", color=COR_RA, ha="right", va="center")

# ---- I_final ----
x_fixed = np.arange(WINDOW_SIZE)
line_activation, = ax5.plot(x_fixed, list(I_final_data), lw=1.2,
                            color=COR_TENSAO_MEDIA)
ax5.set_title("I_final")
ax5.set_xlim(0, WINDOW_SIZE)
ax5.set_ylim(0, VREF)
ax5.set_ylabel("Mean Voltage (V)")
configurar_eixo_artigo(ax5)

# ---- CUNEIFORMES (CN_SA=1, CN_FA=2, CN_MM=3) ----
ax6.set_title("Cuneate Neuron Spike Raster")
ax6.set_xlim(-RASTER_WINDOW, 0)
ax6.set_ylim(0.3, 3.7)
ax6.set_xlabel("t - agora (s)")
ax6.set_yticks([1, 2, 3])
ax6.set_yticklabels(["CN_SA", "CN_FA", "CN_MM"])
configurar_eixo_artigo(ax6)
scatter_CN_SA = ax6.scatter([], [], s=14, color=COR_CNSA)
scatter_CN_RA = ax6.scatter([], [], s=14, color=COR_CNRA)
scatter_CN_MM = ax6.scatter([], [], s=14, color=COR_CNMM)

plt.tight_layout()


def _prune_rel(buf, now_t, y):
    """Poda a deque por idade real e devolve offsets (x relativo, y)."""
    cutoff = now_t - RASTER_WINDOW
    while buf and buf[0] < cutoff:
        buf.popleft()
    xs = [t - now_t for t in buf]
    return np.c_[xs, [y] * len(xs)] if xs else np.empty((0, 2))


def update(_frame):
    now_t = time.monotonic()
    cutoff = now_t - RASTER_WINDOW
    with data_lock:
        vr = np.rot90(voltage_matrix, 2)
        ifd = list(I_final_data)

        x_SA, y_SA = [], []
        for n in range(NUM_TAXELS):
            d = spike_times_SA[n]
            while d and d[0] < cutoff:
                d.popleft()
            for t in d:
                x_SA.append(t - now_t)
                y_SA.append(n + 1)
        x_RA, y_RA = [], []
        for n in range(NUM_TAXELS):
            d = spike_times_RA[n]
            while d and d[0] < cutoff:
                d.popleft()
            for t in d:
                x_RA.append(t - now_t)
                y_RA.append(NUM_TAXELS + n + 1)

        off_sa = np.c_[x_SA, y_SA] if x_SA else np.empty((0, 2))
        off_ra = np.c_[x_RA, y_RA] if x_RA else np.empty((0, 2))
        off_cn_sa = _prune_rel(spike_times_CN_SA, now_t, 1)
        off_cn_ra = _prune_rel(spike_times_CN_RA, now_t, 2)
        off_cn_mm = _prune_rel(spike_times_CN_MM, now_t, 3)

    # ---- HEATMAP ----
    im_volt.set_data(vr)
    for r in range(ROWS):
        for c in range(COLS):
            texts_volt[r][c].set_text(f"{vr[r, c]:.2f}")

    # ---- RASTER ----
    scatter_SA.set_offsets(off_sa)
    scatter_RA.set_offsets(off_ra)

    # ---- I_final ----
    line_activation.set_data(x_fixed, ifd)

    # ---- CUNEIFORMES ----
    scatter_CN_SA.set_offsets(off_cn_sa)
    scatter_CN_RA.set_offsets(off_cn_ra)
    scatter_CN_MM.set_offsets(off_cn_mm)

    return ([im_volt, scatter_SA, scatter_RA, line_activation,
             scatter_CN_SA, scatter_CN_RA, scatter_CN_MM]
            + [t for row in texts_volt for t in row])


# =========================================================
# EXECUCAO
# =========================================================

reader_thread = threading.Thread(target=serial_reader_loop,
                                 name="serial-reader", daemon=True)
reader_thread.start()

# blit=True: eixos fixos (tempo relativo) -> so os artistas mudam, animacao fluida.
ani = FuncAnimation(fig, update, interval=33, blit=True, cache_frame_data=False)

try:
    plt.show()
finally:
    reader_running = False
    reader_thread.join(timeout=1.0)
    ser.close()
    frame_sock.close()
    scalar_sock.close()
