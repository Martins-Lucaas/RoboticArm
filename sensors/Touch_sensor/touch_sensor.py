# =========================================================
# VISUALIZADOR STM32 + IZHIKEVICH
# =========================================================

import argparse
import sys
import threading
import time
import serial
from serial.tools import list_ports
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import re
import socket
import struct

# =========================================================
# CONFIG
# =========================================================

# Porta default por plataforma: no Windows o STM32 aparece como COMx;
# no Linux como /dev/ttyACMx ou /dev/ttyUSBx. Quando None, é detectada
# automaticamente (ver detect_serial_port).
DEFAULT_PORT = "/dev/ttyACM0"
BAUD = 115200

ROWS = 4
COLS = 4

NUM_TAXELS = ROWS * COLS

VREF = 3.3

RASTER_WINDOW = 5.0

WINDOW_SIZE = 50


# Destino UDP configurável por CLI — o default é o broadcast da rede do
# laboratório; em outra rede rode com --udp-ip <broadcast da sua rede>.
parser = argparse.ArgumentParser(
    description="Visualizador STM32 + retransmissor UDP do touch sensor"
)

parser.add_argument(
    "--udp-ip",
    default="192.168.5.255",
    help="destino dos pacotes UDP (broadcast da rede do PC do ROS)"
)

parser.add_argument(
    "--udp-port",
    type=int,
    default=8081,
    help="porta do touch_receiver_node (TOUCH_SENSOR_UDP_PORT)"
)

parser.add_argument(
    "--port",
    default=DEFAULT_PORT,
    help="porta serial do STM32 (ex.: COM7 ou /dev/ttyACM0). "
         "Se omitida no Linux, é detectada automaticamente."
)

cli_args = parser.parse_args()

UDP_IP = cli_args.udp_ip

UDP_PORT = cli_args.udp_port

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.setsockopt(
    socket.SOL_SOCKET,
    socket.SO_BROADCAST,
    1
)

# Contador de amostras do pacote UDP — o touch_receiver usa o salto do
# seq para detectar pacotes perdidos na rede.
udp_seq = 0

# =========================================================
# SERIAL
# =========================================================

def detect_serial_port():
    candidates = [
        p.device for p in list_ports.comports()
        if "ACM" in p.device or "USB" in p.device
    ]
    return candidates[0] if candidates else None


PORT = cli_args.port or detect_serial_port()

if PORT is None:
    disponiveis = ", ".join(p.device for p in list_ports.comports()) or "nenhuma"
    sys.exit(
        "Nenhuma porta serial USB/ACM encontrada.\n"
        f"Portas disponiveis: {disponiveis}\n"
        "Conecte o STM32 ou informe a porta com --port (ex.: --port /dev/ttyACM0)."
    )

try:
    ser = serial.Serial(
        PORT,
        BAUD,
        timeout=0.1
    )
except serial.SerialException as e:
    sys.exit(
        f"Nao foi possivel abrir a porta serial '{PORT}': {e}\n"
        "No Linux verifique se voce tem permissao (ex.: "
        "'sudo usermod -aG dialout $USER' e relogar)."
    )

print(f"Serial conectada em {PORT} @ {BAUD} baud")

# =========================================================
# DADOS
# =========================================================

spike_times_RA = [[] for _ in range(NUM_TAXELS)]

spike_times_SA = [[] for _ in range(NUM_TAXELS)]

spike_times_POST = []

voltage_matrix = np.zeros((ROWS, COLS))

current_time = 0.0

# Estado compartilhado entre a thread de leitura serial/UDP e o desenho do
# matplotlib. Toda a estrutura acima é tocada por AMBAS as threads, então
# leitura e escrita acontecem sob este lock.
data_lock = threading.Lock()

# Heartbeat (#9): instante (time.time()) do último dado válido recebido da
# serial. Começa em 0.0 = "nada recebido ainda". A janela avisa quando a
# serial fica muda por mais de SERIAL_STALE_S.
last_rx_time = 0.0
SERIAL_STALE_S = 2.0

# Flag de parada da thread de leitura — baixada ao fechar a janela.
reader_running = True

# =========================================================
# CORRENTE FINAL
# =========================================================

I_final_data = deque(
    [0]*WINDOW_SIZE,
    maxlen=WINDOW_SIZE
)

# =========================================================
# FIGURA
# =========================================================

fig, axs = plt.subplots(
    2,
    2,
    figsize=(14, 10)
)

ax1 = axs[0,0]
ax2 = axs[0,1]

ax5 = axs[1,0]
ax6 = axs[1,1]

# =========================================================
# HEATMAP
# =========================================================

im_volt = ax1.imshow(
    voltage_matrix,
    cmap="jet",
    interpolation="bicubic",
    vmin=0,
    vmax=VREF
)

plt.colorbar(im_volt, ax=ax1)

ax1.set_title("Voltage (0-3.3V)")

ax1.set_xticks(range(COLS))
ax1.set_yticks(range(ROWS))

texts_volt = [[
    ax1.text(
        c,
        r,
        "0",
        ha="center",
        va="center",
        fontsize=8,
        color="white"
    )
    for c in range(COLS)
] for r in range(ROWS)]

# =========================================================
# RASTER
# =========================================================

ax2.set_title("Raster RA / SA")

ax2.set_xlim(0, RASTER_WINDOW)

ax2.set_ylim(-1, NUM_TAXELS * 2)

scatter_RA = ax2.scatter(
    [],
    [],
    s=10,
    color="red",
    label="RA"
)

scatter_SA = ax2.scatter(
    [],
    [],
    s=10,
    color="blue",
    label="SA"
)

ax2.legend()

# =========================================================
# I_final
# =========================================================

x_fixed = np.arange(WINDOW_SIZE)

line_I_final, = ax5.plot(
    x_fixed,
    I_final_data,
    lw=2
)

ax5.set_title("I_final")

ax5.set_xlim(0, WINDOW_SIZE)

ax5.set_ylim(-1000, 1000)

# =========================================================
# NEURÔNIO PÓS
# =========================================================

ax6.set_title("Neuronio Pos")

ax6.set_xlim(0, RASTER_WINDOW)

ax6.set_ylim(-1, 1)

scatter_POST = ax6.scatter(
    [],
    [],
    s=20,
    color="black"
)

# =========================================================
# PARSE DE UMA LINHA (roda na thread de leitura)
# =========================================================

def note_time(t):
    """Avança ``current_time`` e detecta reset/wrap do ``micros()`` do STM32.

    Chamada SEMPRE sob ``data_lock``. O plotter é TEMPO REAL: a poda da janela
    é feita no desenho (``now_t - t <= RASTER_WINDOW``). Sem esta proteção, um
    salto grande para trás — STM32 reiniciado entre testes, ou wrap do contador
    (~a cada 71 min) — faria ``current_time`` despencar; aí ``now_t - t`` fica
    negativo para os spikes antigos, a condição da poda vira sempre verdadeira e
    eles NUNCA são descartados: as listas cresceriam sem limite e o plotter
    deixaria de mostrar o tempo real. Ao detectar a regressão, limpamos os
    buffers e re-ancoramos o relógio no novo timestamp (a persistência de dados
    é responsabilidade dos modos toque/deslizamento, não destes buffers).
    """
    global current_time
    if t + RASTER_WINDOW < current_time:
        for n in range(NUM_TAXELS):
            spike_times_RA[n].clear()
            spike_times_SA[n].clear()
        spike_times_POST.clear()
        current_time = t
        return
    if t > current_time:
        current_time = t


def process_line(line):
    """Interpreta uma linha da serial e atualiza o estado compartilhado.

    Roda na thread de leitura (serial_reader_loop), NÃO no matplotlib —
    por isso a retransmissão UDP para o ROS não depende mais da janela
    estar respondendo (#8). Toda escrita de estado é feita sob data_lock.
    """
    global current_time, last_rx_time, udp_seq

    line = line.strip()
    if not line:
        return

    # ── ADC / heatmap ────────────────────────────────────────────────
    if line.startswith("DATA"):
        m = re.search(r"idx=(\d+),adc=(\d+),t=(\d+)", line)
        if m:
            idx = int(m.group(1))
            adc = int(m.group(2))
            tstamp = int(m.group(3)) / 1e6
            row, col = divmod(idx, COLS)
            with data_lock:
                note_time(tstamp)
                voltage_matrix[row, col] = adc * (VREF / 4095.0)
                last_rx_time = time.time()

    # ── RA ───────────────────────────────────────────────────────────
    elif line.startswith("RA"):
        m = re.search(r"idx=(\d+),adc=\d+,t=(\d+)", line)
        if m:
            idx = int(m.group(1))
            tstamp = int(m.group(2)) / 1e6
            with data_lock:
                note_time(tstamp)
                spike_times_RA[idx].append(tstamp)
                last_rx_time = time.time()

    # ── SA ───────────────────────────────────────────────────────────
    elif line.startswith("SA"):
        m = re.search(r"idx=(\d+),adc=\d+,t=(\d+)", line)
        if m:
            idx = int(m.group(1))
            tstamp = int(m.group(2)) / 1e6
            with data_lock:
                note_time(tstamp)
                spike_times_SA[idx].append(tstamp)
                last_rx_time = time.time()

    # ── POST ─────────────────────────────────────────────────────────
    elif line.startswith("POST"):
        m = re.search(r"t=(\d+)", line)
        if m:
            tstamp = int(m.group(1)) / 1e6
            with data_lock:
                note_time(tstamp)
                spike_times_POST.append(tstamp)
                last_rx_time = time.time()

    # ── CORRENTE FINAL → UDP ─────────────────────────────────────────
    elif line.startswith("TOTAL"):
        m = re.search(
            r"Iexc=([-+0-9.eE]+),Iinh=([-+0-9.eE]+),Ifinal=([-+0-9.eE]+)",
            line)
        if m:
            I_final = float(m.group(3))
            with data_lock:
                I_final_data.append(I_final)
                last_rx_time = time.time()

            # Pacote UDP do touch sensor (touch_receiver_node).
            # Payload little-endian, 8 bytes: uint32 seq + float value.
            packet = struct.pack('<If', udp_seq & 0xFFFFFFFF, I_final)
            udp_seq += 1
            try:
                sock.sendto(packet, (UDP_IP, UDP_PORT))
            except OSError as exc:
                print(f"[WARN] falha no envio UDP: {exc}")
            print(f"I_final={I_final:.6f}  UDP -> {UDP_IP}:{UDP_PORT}")


# =========================================================
# THREAD DE LEITURA SERIAL + UDP (independe do matplotlib)
# =========================================================

def serial_reader_loop():
    """Lê a serial continuamente, parseia e retransmite por UDP.

    Separar isto do update() do matplotlib (#8) tem dois ganhos:
      • o fluxo de dados para o ROS não trava nem perde taxa quando o
        desenho engasga ou a janela é arrastada;
      • dá pra detectar a serial muda (#9) mesmo com a janela ociosa.
    """
    buf = ""
    while reader_running:
        try:
            chunk = ser.read(ser.in_waiting or 1)
        except serial.SerialException as exc:
            print(f"[ERRO] leitura serial falhou: {exc}")
            break
        if not chunk:
            continue  # timeout (0.1 s) — volta a checar reader_running
        buf += chunk.decode(errors='ignore')
        parts = buf.split("\n")
        buf = parts[-1]
        for line in parts[:-1]:
            process_line(line)


# =========================================================
# UPDATE (só desenho — snapshot do estado sob lock)
# =========================================================

def update(frame):
    with data_lock:
        now_t   = current_time
        vm      = voltage_matrix.copy()
        rx_age  = (time.time() - last_rx_time) if last_rx_time > 0.0 else None

        # Poda as janelas de spikes e monta os offsets já sob o lock.
        x_RA, y_RA = [], []
        for n in range(NUM_TAXELS):
            spike_times_RA[n] = [t for t in spike_times_RA[n]
                                 if now_t - t <= RASTER_WINDOW]
            for t in spike_times_RA[n]:
                x_RA.append(t); y_RA.append(n)

        x_SA, y_SA = [], []
        for n in range(NUM_TAXELS):
            spike_times_SA[n] = [t for t in spike_times_SA[n]
                                 if now_t - t <= RASTER_WINDOW]
            for t in spike_times_SA[n]:
                x_SA.append(t); y_SA.append(n + NUM_TAXELS)

        spike_times_POST[:] = [t for t in spike_times_POST
                               if now_t - t <= RASTER_WINDOW]
        x_POST = list(spike_times_POST)
        I_final_snapshot = list(I_final_data)

    # ── HEATMAP ──────────────────────────────────────────────────────
    im_volt.set_data(np.rot90(vm, 2))
    for r in range(ROWS):
        for c in range(COLS):
            texts_volt[r][c].set_text(f"{vm[r, c]:.2f}")

    # ── RASTER RA / SA ───────────────────────────────────────────────
    scatter_RA.set_offsets(np.c_[x_RA, y_RA])
    scatter_SA.set_offsets(np.c_[x_SA, y_SA])
    ax2.set_xlim(max(0, now_t - RASTER_WINDOW), now_t)

    # ── POST ─────────────────────────────────────────────────────────
    scatter_POST.set_offsets(np.c_[x_POST, [0] * len(x_POST)])
    ax6.set_xlim(max(0, now_t - RASTER_WINDOW), now_t)

    # ── CORRENTE FINAL ───────────────────────────────────────────────
    line_I_final.set_data(x_fixed, I_final_snapshot)

    # ── HEARTBEAT (#9): avisa quando a serial fica muda ──────────────
    if rx_age is None:
        fig.suptitle("Aguardando dados da serial...",
                     color="orange", fontsize=12)
    elif rx_age > SERIAL_STALE_S:
        fig.suptitle(f"SEM DADOS DA SERIAL há {rx_age:.1f} s",
                     color="red", fontsize=12)
    else:
        fig.suptitle("")

    return [im_volt, scatter_RA, scatter_SA, scatter_POST, line_I_final]


# =========================================================
# ANIMAÇÃO
# =========================================================

reader_thread = threading.Thread(
    target=serial_reader_loop, name="serial-reader", daemon=True)
reader_thread.start()

ani = FuncAnimation(fig, update, interval=50)

plt.tight_layout()

try:
    plt.show()
finally:
    # Fecha a janela → encerra a thread de leitura e a serial.
    reader_running = False
    reader_thread.join(timeout=1.0)
    ser.close()
    sock.close()
