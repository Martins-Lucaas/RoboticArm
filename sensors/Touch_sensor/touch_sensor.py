# =========================================================
# VISUALIZADOR STM32 + IZHIKEVICH
# =========================================================

import argparse
import sys
import serial
from serial.tools import list_ports
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import re
import socket
import struct
import threading

# =========================================================
# CONFIG
# =========================================================

# Porta default por plataforma: no Windows o STM32 aparece como COMx;
# no Linux como /dev/ttyACMx ou /dev/ttyUSBx. Quando None, é detectada
# automaticamente (ver detect_serial_port).
DEFAULT_PORT = "/dev/ttyACM1"
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

parser.add_argument(
    "--verbose",
    action="store_true",
    help="imprime cada I_final / pacote UDP no terminal. "
         "Desligado por padrao: o print em alta frequencia trava a GUI."
)

cli_args = parser.parse_args()

UDP_IP = cli_args.udp_ip

UDP_PORT = cli_args.udp_port

VERBOSE = cli_args.verbose

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
    """Acha a primeira porta USB/ACM disponível (STM32 via USB-CDC)."""
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

serial_buffer = ""

# =========================================================
# DADOS
# =========================================================

# Lock que protege todas as estruturas compartilhadas entre a thread de
# leitura serial (produtor) e o update() da animacao (consumidor).
data_lock = threading.Lock()

# Sinaliza a thread serial para encerrar quando a janela fecha.
stop_event = threading.Event()

spike_times_RA = [[] for _ in range(NUM_TAXELS)]

spike_times_SA = [[] for _ in range(NUM_TAXELS)]

spike_times_POST = []

voltage_matrix = np.zeros((ROWS, COLS))

current_time = 0.0

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

def _offsets(xs, ys):
    """Empacota pares (x, y) no formato (N, 2) exigido por set_offsets.

    np.c_ com listas vazias devolve shape (1, 0), o que gera warnings e
    scatters fantasmas — por isso o caso vazio e tratado a parte.
    """
    if len(xs) == 0:
        return np.empty((0, 2))
    return np.column_stack((xs, ys))


# =========================================================
# LEITURA SERIAL (thread de fundo)
# =========================================================

# Regex pre-compiladas: o parser roda em cada linha da serial; compilar
# uma vez evita o overhead de lookup do cache interno a cada chamada.
RE_DATA = re.compile(r"idx=(\d+),adc=(\d+),t=(\d+)")
RE_IDX_T = re.compile(r"idx=(\d+),adc=\d+,t=(\d+)")
RE_POST = re.compile(r"t=(\d+)")
RE_TOTAL = re.compile(
    r"Iexc=([-+0-9.eE]+),Iinh=([-+0-9.eE]+),Ifinal=([-+0-9.eE]+)"
)


def serial_worker():
    """Le e parseia a serial continuamente, fora da thread da GUI.

    Mantem o parsing e o envio UDP em baixa latencia mesmo quando a
    renderizacao do matplotlib esta lenta, e evita que rajadas grandes
    travem a janela.
    """
    global current_time, serial_buffer, udp_seq

    while not stop_event.is_set():

        try:
            # read(in_waiting or 1) bloqueia ate timeout (0.1s) por pelo
            # menos 1 byte, evitando busy-loop quando nao ha dados.
            chunk = ser.read(ser.in_waiting or 1)
        except (serial.SerialException, OSError):
            break

        if not chunk:
            continue

        serial_buffer += chunk.decode(errors='ignore')

        lines = serial_buffer.split("\n")

        serial_buffer = lines[-1]

        for line in lines[:-1]:

            line = line.strip()

            # =================================================
            # ADC
            # =================================================

            if line.startswith("DATA"):

                m = RE_DATA.search(line)

                if m:

                    idx = int(m.group(1))

                    adc = int(m.group(2))

                    tstamp = int(m.group(3)) / 1e6

                    row, col = divmod(idx, COLS)

                    with data_lock:
                        current_time = tstamp
                        voltage_matrix[row, col] = (
                            adc * (VREF / 4095.0)
                        )

            # =================================================
            # RA
            # =================================================

            elif line.startswith("RA"):

                m = RE_IDX_T.search(line)

                if m:

                    idx = int(m.group(1))

                    tstamp = int(m.group(2)) / 1e6

                    with data_lock:
                        spike_times_RA[idx].append(tstamp)

            # =================================================
            # SA
            # =================================================

            elif line.startswith("SA"):

                m = RE_IDX_T.search(line)

                if m:

                    idx = int(m.group(1))

                    tstamp = int(m.group(2)) / 1e6

                    with data_lock:
                        spike_times_SA[idx].append(tstamp)

            # =================================================
            # POST
            # =================================================

            elif line.startswith("POST"):

                m = RE_POST.search(line)

                if m:

                    tstamp = int(m.group(1)) / 1e6

                    with data_lock:
                        spike_times_POST.append(tstamp)

            # =================================================
            # CORRENTE FINAL
            # =================================================
            elif line.startswith("TOTAL"):

                m = RE_TOTAL.search(line)

                if m:

                    try:
                        I_final = float(m.group(3))
                    except ValueError:
                        # Sensor sem resposta: envia '-' / 'nan' no lugar do valor.
                        I_final = None

                    if I_final is None or not np.isfinite(I_final):
                        # Sem leitura valida neste ciclo: ignora.
                        continue

                    with data_lock:
                        I_final_data.append(I_final)
                        seq = udp_seq
                        udp_seq += 1

                    # =================================================
                    # PACOTE UDP DO TOUCH SENSOR (touch_receiver_node)
                    # Payload little-endian, 8 bytes:
                    #     uint32 seq    — contador (detecta perda)
                    #     float  value  — I_final
                    # =================================================

                    packet = struct.pack(
                        '<If',
                        seq & 0xFFFFFFFF,
                        I_final
                    )

                    sock.sendto(
                        packet,
                        (UDP_IP, UDP_PORT)
                    )

                    if VERBOSE:
                        print(
                            f"UDP -> {UDP_IP}:{UDP_PORT} | Ifinal={I_final:.3f}"
                        )


# =========================================================
# UPDATE (apenas renderiza; nao toca na serial)
# =========================================================

def update(frame):

    # Le um retrato consistente do estado compartilhado sob o lock, ja
    # podando a janela do raster. So tocamos nas estruturas compartilhadas
    # aqui dentro; o desenho dos artistas (caro) fica todo fora do lock.
    with data_lock:

        t_now = current_time

        # Janela deslizante: descartamos spikes mais antigos que a janela
        # e guardamos copias rasas para processar sem segurar o lock.
        ra_lists = []
        for n in range(NUM_TAXELS):
            spike_times_RA[n] = [
                t for t in spike_times_RA[n]
                if t_now - t <= RASTER_WINDOW
            ]
            ra_lists.append(spike_times_RA[n])

        sa_lists = []
        for n in range(NUM_TAXELS):
            spike_times_SA[n] = [
                t for t in spike_times_SA[n]
                if t_now - t <= RASTER_WINDOW
            ]
            sa_lists.append(spike_times_SA[n])

        spike_times_POST[:] = [
            t for t in spike_times_POST
            if t_now - t <= RASTER_WINDOW
        ]
        post_list = list(spike_times_POST)

        volt = voltage_matrix.copy()
        I_final_snapshot = list(I_final_data)

    # A partir daqui nada toca no estado compartilhado: a thread serial
    # roda livre enquanto o matplotlib desenha.

    # Tempo relativo a borda esquerda da janela. Manter o eixo fixo em
    # [0, RASTER_WINDOW] e a chave para o blit funcionar (sem set_xlim por
    # frame, que forcaria um redraw completo do eixo).
    x_lo = max(0.0, t_now - RASTER_WINDOW)

    x_RA = []
    y_RA = []
    for n in range(NUM_TAXELS):
        for t in ra_lists[n]:
            x_RA.append(t - x_lo)
            y_RA.append(n)

    x_SA = []
    y_SA = []
    for n in range(NUM_TAXELS):
        for t in sa_lists[n]:
            x_SA.append(t - x_lo)
            y_SA.append(n + NUM_TAXELS)

    x_POST = [t - x_lo for t in post_list]

    # =====================================================
    # HEATMAP
    # =====================================================

    im_volt.set_data(
        np.rot90(volt, 2)
    )

    for r in range(ROWS):
        for c in range(COLS):

            texts_volt[r][c].set_text(
                f"{volt[r, c]:.2f}"
            )

    # =====================================================
    # RASTER RA / SA
    # =====================================================

    scatter_RA.set_offsets(
        _offsets(x_RA, y_RA)
    )

    scatter_SA.set_offsets(
        _offsets(x_SA, y_SA)
    )

    # =====================================================
    # POST
    # =====================================================

    scatter_POST.set_offsets(
        _offsets(x_POST, [0] * len(x_POST))
    )

    # =====================================================
    # CORRENTE FINAL
    # =====================================================

    line_I_final.set_data(
        x_fixed,
        I_final_snapshot
    )

    return blit_artists

# =========================================================
# ANIMAÇÃO
# =========================================================

# Todos os artistas que mudam por frame. Com blit=True o matplotlib
# redesenha apenas estes (em vez da figura inteira), o que e o ganho
# principal de performance. Os textos do heatmap precisam entrar aqui,
# senao nao seriam redesenhados sob o blit.
blit_artists = [
    im_volt,
    scatter_RA,
    scatter_SA,
    scatter_POST,
    line_I_final,
] + [t for row in texts_volt for t in row]


def init_anim():
    """Estado inicial usado pelo blit para capturar o fundo limpo."""
    scatter_RA.set_offsets(np.empty((0, 2)))
    scatter_SA.set_offsets(np.empty((0, 2)))
    scatter_POST.set_offsets(np.empty((0, 2)))
    line_I_final.set_data(x_fixed, list(I_final_data))
    return blit_artists


serial_thread = threading.Thread(
    target=serial_worker,
    daemon=True
)
serial_thread.start()

ani = FuncAnimation(
    fig,
    update,
    init_func=init_anim,
    interval=50,
    blit=True,
    cache_frame_data=False
)

plt.tight_layout()

try:
    plt.show()
finally:
    # Encerra a thread serial e fecha os recursos ao sair.
    stop_event.set()
    serial_thread.join(timeout=1.0)
    ser.close()
    sock.close()
