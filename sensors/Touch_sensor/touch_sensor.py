# =========================================================
# VISUALIZADOR STM32 + IZHIKEVICH
# =========================================================

import argparse
import serial
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

PORT = 'COM7'
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

ser = serial.Serial(
    PORT,
    BAUD,
    timeout=0.1
)

serial_buffer = ""

# =========================================================
# DADOS
# =========================================================

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

# =========================================================
# UPDATE
# =========================================================

def update(frame):

    global current_time
    global voltage_matrix
    global serial_buffer
    global udp_seq

    # =====================================================
    # SERIAL
    # =====================================================

    if ser.in_waiting:

        serial_buffer += ser.read(
            ser.in_waiting
        ).decode(errors='ignore')

        lines = serial_buffer.split("\n")

        serial_buffer = lines[-1]

        for line in lines[:-1]:

            line = line.strip()

            # =================================================
            # ADC
            # =================================================

            if line.startswith("DATA"):

                m = re.search(
                    r"idx=(\d+),adc=(\d+),t=(\d+)",
                    line
                )

                if m:

                    idx = int(m.group(1))

                    adc = int(m.group(2))

                    tstamp = int(m.group(3)) / 1e6

                    current_time = tstamp

                    row, col = divmod(idx, COLS)

                    voltage_matrix[row, col] = (
                        adc * (VREF / 4095.0)
                    )

            # =================================================
            # RA
            # =================================================

            elif line.startswith("RA"):

                m = re.search(
                    r"idx=(\d+),adc=\d+,t=(\d+)",
                    line
                )

                if m:

                    idx = int(m.group(1))

                    tstamp = int(m.group(2)) / 1e6

                    spike_times_RA[idx].append(tstamp)

            # =================================================
            # SA
            # =================================================

            elif line.startswith("SA"):

                m = re.search(
                    r"idx=(\d+),adc=\d+,t=(\d+)",
                    line
                )

                if m:

                    idx = int(m.group(1))

                    tstamp = int(m.group(2)) / 1e6

                    spike_times_SA[idx].append(tstamp)

            # =================================================
            # POST
            # =================================================

            elif line.startswith("POST"):

                m = re.search(
                    r"t=(\d+)",
                    line
                )

                if m:

                    tstamp = int(m.group(1)) / 1e6

                    spike_times_POST.append(tstamp)

            # =================================================
            # CORRENTE FINAL
            # =================================================
            elif line.startswith("TOTAL"):

                m = re.search(
                    r"Iexc=([-+0-9.eE]+),Iinh=([-+0-9.eE]+),Ifinal=([-+0-9.eE]+)",
                    line
                )

                if m:

                    I_final = float(m.group(3))

                    print(
                        f"I_final={I_final:.6f}"
                    )

                    I_final_data.append(I_final)

                    # =================================================
                    # PACOTE UDP DO TOUCH SENSOR (touch_receiver_node)
                    # Payload little-endian, 8 bytes:
                    #     uint32 seq    — contador (detecta perda)
                    #     float  value  — I_final
                    # =================================================

                    packet = struct.pack(
                        '<If',
                        udp_seq & 0xFFFFFFFF,
                        I_final
                    )

                    udp_seq += 1

                    sock.sendto(
                        packet,
                        (UDP_IP, UDP_PORT)
                    )
                    print(
                        f"UDP -> {UDP_IP}:{UDP_PORT} | Ifinal={I_final:.3f}"
                    )


    # =====================================================
    # HEATMAP
    # =====================================================

    im_volt.set_data(
        np.rot90(voltage_matrix, 2)
    )

    for r in range(ROWS):
        for c in range(COLS):

            texts_volt[r][c].set_text(
                f"{voltage_matrix[r,c]:.2f}"
            )

    # =====================================================
    # RASTER RA
    # =====================================================

    x_RA = []
    y_RA = []

    for n in range(NUM_TAXELS):

        spike_times_RA[n] = [

            t for t in spike_times_RA[n]

            if current_time - t <= RASTER_WINDOW
        ]

        for t in spike_times_RA[n]:

            x_RA.append(t)

            y_RA.append(n)

    # =====================================================
    # RASTER SA
    # =====================================================

    x_SA = []
    y_SA = []

    for n in range(NUM_TAXELS):

        spike_times_SA[n] = [

            t for t in spike_times_SA[n]

            if current_time - t <= RASTER_WINDOW
        ]

        for t in spike_times_SA[n]:

            x_SA.append(t)

            y_SA.append(n + NUM_TAXELS)

    scatter_RA.set_offsets(
        np.c_[x_RA, y_RA]
    )

    scatter_SA.set_offsets(
        np.c_[x_SA, y_SA]
    )

    ax2.set_xlim(
        max(0, current_time - RASTER_WINDOW),
        current_time
    )

    # =====================================================
    # POST
    # =====================================================

    spike_times_POST[:] = [

        t for t in spike_times_POST

        if current_time - t <= RASTER_WINDOW
    ]

    scatter_POST.set_offsets(

        np.c_[
            spike_times_POST,
            [0]*len(spike_times_POST)
        ]
    )

    ax6.set_xlim(
        max(0, current_time - RASTER_WINDOW),
        current_time
    )

    # =====================================================
    # CORRENTE FINAL
    # =====================================================

    line_I_final.set_data(
        x_fixed,
        I_final_data
    )

    return [
        im_volt,
        scatter_RA,
        scatter_SA,
        scatter_POST,
        line_I_final
    ]

# =========================================================
# ANIMAÇÃO
# =========================================================

ani = FuncAnimation(
    fig,
    update,
    interval=50
)

plt.tight_layout()

plt.show()
