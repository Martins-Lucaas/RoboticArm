# =========================================================
# VISUALIZADOR STM32 + IZHIKEVICH — touch sensor 4x4
# =========================================================
#
# Mesmo funcionamento do 5x5 (5x5_base): janela deslizante por TEMPO REAL de
# chegada (time.monotonic) — acumula e sai sozinha, SEM o "reset" do antigo
# note_time — e estilo de artigo nos 4 graficos. Mantem o protocolo do 4x4
# (DATA por taxel, TOTAL com I_final real, POST), diferente do ADC/CN do 5x5.
#
# Retransmite por UDP para o PC do ROS:
#   • 8082 (frame): linhas BRUTAS, para a GUI (touch_pack) reconstruir tudo;
#   • 8081 (escalar): I_final '<If' por TOTAL, para o touch_receiver_node.
# O relay roda na thread serial (best-effort), entao nao trava a plotagem.

import argparse
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
import re
import socket
import struct

# =========================================================
# CONFIG
# =========================================================

DEFAULT_PORT = None  # None = autodeteccao (Linux: /dev/ttyACM*; Windows: COMx)
BAUD = 115200

ROWS = 4
COLS = 4
NUM_TAXELS = ROWS * COLS

VREF = 3.3

RASTER_WINDOW = 5.0
WINDOW_SIZE = 50

parser = argparse.ArgumentParser(
    description="Plotter STM32 4x4 + relay UDP (8082 frame / 8081 escalar)"
)
parser.add_argument("--udp-ip", default="192.168.5.255",
                    help="destino broadcast dos pacotes UDP (rede do PC do ROS)")
parser.add_argument("--udp-port", type=int, default=8081,
                    help="porta do escalar I_final (TOUCH_SENSOR_UDP_PORT)")
parser.add_argument("--frame-port", type=int, default=8082,
                    help="porta do relay de FRAME/linhas brutas (TOUCH_FRAME_UDP_PORT)")
parser.add_argument("--port", default=DEFAULT_PORT,
                    help="porta serial do STM32 (ex.: COM7 ou /dev/ttyACM0). "
                         "Se omitida, autodetecta.")
cli_args = parser.parse_args()

UDP_IP = cli_args.udp_ip
UDP_PORT = cli_args.udp_port
FRAME_PORT = cli_args.frame_port


def _make_bcast_socket():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    return s


sock = _make_bcast_socket()        # escalar (8081)
frame_sock = _make_bcast_socket()  # frame/linhas brutas (8082)
udp_seq = 0

# =========================================================
# SERIAL
# =========================================================

def detect_serial_port():
    candidates = [p.device for p in list_ports.comports()
                  if "ACM" in p.device or "USB" in p.device
                  or p.device.upper().startswith("COM")]
    return candidates[0] if candidates else None


PORT = cli_args.port or detect_serial_port()

if PORT is None:
    disponiveis = ", ".join(p.device for p in list_ports.comports()) or "nenhuma"
    sys.exit(
        "Nenhuma porta serial encontrada.\n"
        f"Portas disponiveis: {disponiveis}\n"
        "Conecte o STM32 ou informe a porta com --port."
    )

try:
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
except serial.SerialException as e:
    sys.exit(f"Nao foi possivel abrir a porta serial '{PORT}': {e}")

print(f"Serial conectada em {PORT} @ {BAUD} baud")
print(f"Relay UDP: frame -> {UDP_IP}:{FRAME_PORT} | escalar -> {UDP_IP}:{UDP_PORT}")

# =========================================================
# DADOS (instantes guardados = relogio REAL do PC na chegada)
# =========================================================

spike_times_RA = [deque() for _ in range(NUM_TAXELS)]
spike_times_SA = [deque() for _ in range(NUM_TAXELS)]
spike_times_POST = deque()
voltage_matrix = np.zeros((ROWS, COLS))
I_final_data = deque([0.0] * WINDOW_SIZE, maxlen=WINDOW_SIZE)

data_lock = threading.Lock()
last_rx_time = 0.0
SERIAL_STALE_S = 2.0
reader_running = True

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

COR_RA = "#0B3954"   # FA / RA
COR_SA = "#B24C3D"
COR_POST = "#3D4F8A"
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
# FIGURA
# =========================================================

fig, axs = plt.subplots(2, 2, figsize=(14, 10))
ax1, ax2, ax5, ax6 = axs[0, 0], axs[0, 1], axs[1, 0], axs[1, 1]

# ---- HEATMAP ----
im_volt = ax1.imshow(voltage_matrix, cmap="jet", interpolation="bicubic",
                     vmin=0, vmax=VREF)
plt.colorbar(im_volt, ax=ax1)
ax1.set_title(f"Voltage Heatmap — {ROWS}x{COLS}")
ax1.set_xticks(range(COLS))
ax1.set_yticks(range(ROWS))
texts_volt = [[ax1.text(c, r, "0", ha="center", va="center", fontsize=8,
                        color="white")
               for c in range(COLS)] for r in range(ROWS)]

# ---- RASTER FA / SA (SA embaixo 1..N, FA em cima N+1..2N) ----
ax2.set_title("FA and SA Spike Raster")
ax2.set_xlim(-RASTER_WINDOW, 0)
ax2.set_ylim(0.3, NUM_TAXELS * 2 + 0.7)
ax2.set_xlabel("t - agora (s)")
_yt = list(range(4, NUM_TAXELS + 1, 4)) + \
    list(range(NUM_TAXELS + 4, NUM_TAXELS * 2 + 1, 4))
ax2.set_yticks(_yt)
ax2.set_yticklabels([f"SA{y}" for y in range(4, NUM_TAXELS + 1, 4)] +
                    [f"FA{y}" for y in range(4, NUM_TAXELS + 1, 4)])
configurar_eixo_artigo(ax2)
scatter_SA = ax2.scatter([], [], s=10, color=COR_SA)
scatter_RA = ax2.scatter([], [], s=10, color=COR_RA)
ax2.text(-0.12, NUM_TAXELS * 0.5, "SA", transform=ax2.get_yaxis_transform(),
         fontweight="bold", color=COR_SA, ha="right", va="center")
ax2.text(-0.12, NUM_TAXELS * 1.5, "FA", transform=ax2.get_yaxis_transform(),
         fontweight="bold", color=COR_RA, ha="right", va="center")

# ---- I_final (corrente real do neuronio, via TOTAL) ----
x_fixed = np.arange(WINDOW_SIZE)
line_I_final, = ax5.plot(x_fixed, list(I_final_data), lw=1.2,
                         color=COR_TENSAO_MEDIA)
ax5.set_title("I_final")
ax5.set_xlim(0, WINDOW_SIZE)
ax5.set_ylim(-1000, 1000)
ax5.set_ylabel("I_final")
configurar_eixo_artigo(ax5)

# ---- NEURONIO POS ----
ax6.set_title("Neuronio Pos")
ax6.set_xlim(-RASTER_WINDOW, 0)
ax6.set_ylim(-1, 1)
ax6.set_xlabel("t - agora (s)")
ax6.set_yticks([0])
ax6.set_yticklabels(["POST"])
configurar_eixo_artigo(ax6)
scatter_POST = ax6.scatter([], [], s=18, color=COR_POST)

plt.tight_layout()

# =========================================================
# RELAY UDP (best-effort, na thread serial)
# =========================================================

def relay_lines(lines):
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


# =========================================================
# PARSE (sob data_lock; instantes em time.monotonic)
# =========================================================

def process_line(line, now):
    global last_rx_time, udp_seq
    line = line.strip()
    if not line:
        return

    if line.startswith("DATA"):
        m = re.search(r"idx=(\d+),adc=(\d+),t=(\d+)", line)
        if m:
            idx = int(m.group(1))
            if 0 <= idx < NUM_TAXELS:
                adc = int(m.group(2))
                row, col = divmod(idx, COLS)
                with data_lock:
                    voltage_matrix[row, col] = adc * (VREF / 4095.0)
                    last_rx_time = time.time()

    elif line.startswith("RA"):
        m = re.search(r"idx=(\d+),adc=\d+,t=\d+", line)
        if m:
            idx = int(m.group(1))
            if 0 <= idx < NUM_TAXELS:
                with data_lock:
                    spike_times_RA[idx].append(now)
                    last_rx_time = time.time()

    elif line.startswith("SA"):
        m = re.search(r"idx=(\d+),adc=\d+,t=\d+", line)
        if m:
            idx = int(m.group(1))
            if 0 <= idx < NUM_TAXELS:
                with data_lock:
                    spike_times_SA[idx].append(now)
                    last_rx_time = time.time()

    elif line.startswith("POST"):
        with data_lock:
            spike_times_POST.append(now)
            last_rx_time = time.time()

    elif line.startswith("TOTAL"):
        m = re.search(
            r"Iexc=([-+0-9.eE]+),Iinh=([-+0-9.eE]+),Ifinal=([-+0-9.eE]+)", line)
        if m:
            I_final = float(m.group(3))
            with data_lock:
                I_final_data.append(I_final)
                last_rx_time = time.time()
            try:
                packet = struct.pack('<If', udp_seq & 0xFFFFFFFF, I_final)
                udp_seq += 1
                sock.sendto(packet, (UDP_IP, UDP_PORT))
            except OSError:
                pass


# =========================================================
# THREAD DE LEITURA SERIAL + RELAY
# =========================================================

def serial_reader_loop():
    buf = ""
    while reader_running:
        try:
            chunk = ser.read(ser.in_waiting or 1)
        except serial.SerialException as exc:
            print(f"[ERRO] leitura serial falhou: {exc}")
            break
        if not chunk:
            continue
        buf += chunk.decode(errors='ignore')
        parts = buf.split("\n")
        buf = parts[-1]
        complete = parts[:-1]
        if not complete:
            continue
        relay_lines(complete)            # 8082, fora do lock
        now = time.monotonic()
        for line in complete:
            process_line(line, now)


# =========================================================
# UPDATE (so desenho — snapshot/poda por tempo real sob lock)
# =========================================================

def update(frame):
    now_t = time.monotonic()
    cutoff = now_t - RASTER_WINDOW
    with data_lock:
        vm = voltage_matrix.copy()
        rx_age = (time.time() - last_rx_time) if last_rx_time > 0.0 else None

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

        while spike_times_POST and spike_times_POST[0] < cutoff:
            spike_times_POST.popleft()
        x_POST = [t - now_t for t in spike_times_POST]
        I_final_snapshot = list(I_final_data)

    # ── HEATMAP ──
    vr = np.rot90(vm, 2)
    im_volt.set_data(vr)
    for r in range(ROWS):
        for c in range(COLS):
            texts_volt[r][c].set_text(f"{vr[r, c]:.2f}")

    # ── RASTER FA / SA ──
    scatter_SA.set_offsets(np.c_[x_SA, y_SA] if x_SA else np.empty((0, 2)))
    scatter_RA.set_offsets(np.c_[x_RA, y_RA] if x_RA else np.empty((0, 2)))

    # ── NEURONIO POS ──
    scatter_POST.set_offsets(
        np.c_[x_POST, [0] * len(x_POST)] if x_POST else np.empty((0, 2)))

    # ── I_final ──
    line_I_final.set_data(x_fixed, I_final_snapshot)

    # ── HEARTBEAT ──
    if rx_age is None:
        fig.suptitle("Aguardando dados da serial...", color="orange", fontsize=12)
    elif rx_age > SERIAL_STALE_S:
        fig.suptitle(f"SEM DADOS DA SERIAL há {rx_age:.1f} s",
                     color="red", fontsize=12)
    else:
        fig.suptitle("")

    return [im_volt, scatter_SA, scatter_RA, scatter_POST, line_I_final]


# =========================================================
# ANIMACAO
# =========================================================

reader_thread = threading.Thread(
    target=serial_reader_loop, name="serial-reader", daemon=True)
reader_thread.start()

ani = FuncAnimation(fig, update, interval=50, cache_frame_data=False)

try:
    plt.show()
finally:
    reader_running = False
    reader_thread.join(timeout=1.0)
    ser.close()
    sock.close()
    frame_sock.close()
