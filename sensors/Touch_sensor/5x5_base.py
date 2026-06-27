import serial
import csv
import re
import time
import os
from datetime import datetime
import tkinter as tk

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator
from collections import deque

# =========================================================
# CONFIG
# =========================================================

PORT = "/dev/ttyACM1"
BAUD = 115200

RECORD_TIME = 30

ROWS = 5
COLS = 5
NUM_TAXELS = 25

VREF = 3.3

# Janela (s) do raster e largura (frames) do grafico de I_final.
RASTER_WINDOW = 5.0
WINDOW_SIZE = 50

# Periodo minimo entre redesenhos dos graficos (s) — evita travar o loop.
# Com blitting o redesenho e barato, entao da pra atualizar mais rapido (~30 Hz).
PLOT_PERIOD = 0.03

SAVE_FOLDER = os.path.expanduser("~/coleta_seb")

os.makedirs(SAVE_FOLDER, exist_ok=True)

ser = serial.Serial(PORT, BAUD, timeout=0.1)

timestamp = datetime.now().strftime("1")

# =========================================================
# INTERFACE
# =========================================================

root = tk.Tk()
root.title("Experimento Tátil")

root.geometry("600x300")

label_status = tk.Label(
    root,
    text="AGUARDE",
    font=("Arial", 28, "bold"),
    bg="red",
    fg="white",
    width=20
)

label_status.pack(pady=20)

label_timer = tk.Label(
    root,
    text="0.0 s",
    font=("Arial", 40)
)

label_timer.pack()

# =========================================================
# ESTADO DOS GRAFICOS
# =========================================================

voltage_matrix = np.zeros((ROWS, COLS))

activation_data = deque([0] * WINDOW_SIZE, maxlen=WINDOW_SIZE)

# Os tempos guardados para o desenho sao o instante REAL de chegada
# (time.monotonic()), nao o timestamp do firmware. Assim a janela deslizante
# acumula e descarta por tempo real, sem depender do relogio do STM32 (que
# pode ter bases diferentes entre ADC e spikes e causava limpezas indevidas).
spike_times_RA = [[] for _ in range(NUM_TAXELS)]
spike_times_SA = [[] for _ in range(NUM_TAXELS)]
spike_times_CN_MM = []
spike_times_CN_RA = []
spike_times_CN_SA = []


# =========================================================
# ESTILO DE ARTIGO (cores, fonte serifada, eixos limpos)
# =========================================================

# Fonte serifada tipo Times (cai para Liberation/DejaVu se Times nao existir).
# Tamanhos maiores que os do PDF do artigo, para leitura na tela ao vivo.
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
# FIGURA
# =========================================================

plt.ion()

fig, axs = plt.subplots(2, 2, figsize=(14, 10))

ax1 = axs[0, 0]
ax2 = axs[0, 1]
ax5 = axs[1, 0]
ax6 = axs[1, 1]

# Os artistas dinamicos sao marcados animated=True: assim o draw() normal os
# ignora (eles nao entram no fundo) e cada frame so repinta eles via blitting.

# ---- HEATMAP ----
im_volt = ax1.imshow(
    voltage_matrix,
    cmap="jet",
    interpolation="bicubic",
    vmin=0,
    vmax=VREF,
    animated=True
)

plt.colorbar(im_volt, ax=ax1)

ax1.set_title("Voltage Heatmap — 5x5")
ax1.set_xticks(range(COLS))
ax1.set_yticks(range(ROWS))

texts_volt = [[
    ax1.text(c, r, "0", ha="center", va="center", fontsize=8,
             color="white", animated=True)
    for c in range(COLS)
] for r in range(ROWS)]

# ---- RASTER FA / SA ----
# Eixo em tempo relativo: 0 s = agora, -RASTER_WINDOW = mais antigo. Limites
# fixos -> o fundo nao muda e o blitting fica simples e fluido.
# Orientacao do artigo: SA embaixo (1..25), FA em cima (26..50).
ax2.set_title("FA and SA Spike Raster")
ax2.set_xlim(-RASTER_WINDOW, 0)
ax2.set_ylim(0.3, NUM_TAXELS * 2 + 0.7)
ax2.set_xlabel("t - agora (s)")
ax2.set_yticks([5, 10, 15, 20, 25, 30, 35, 40, 45, 50])
ax2.set_yticklabels(["SA5", "SA10", "SA15", "SA20", "SA25",
                     "FA5", "FA10", "FA15", "FA20", "FA25"])
configurar_eixo_artigo(ax2)

scatter_SA = ax2.scatter([], [], s=9, color=COR_SA, animated=True)
scatter_RA = ax2.scatter([], [], s=9, color=COR_RA, animated=True)

ax2.text(-0.12, NUM_TAXELS * 0.5, "SA",
         transform=ax2.get_yaxis_transform(), fontweight="bold",
         color=COR_SA, ha="right", va="center")
ax2.text(-0.12, NUM_TAXELS * 1.5, "FA",
         transform=ax2.get_yaxis_transform(), fontweight="bold",
         color=COR_RA, ha="right", va="center")

# ---- I_final ----
x_fixed = np.arange(WINDOW_SIZE)

line_activation, = ax5.plot(x_fixed, activation_data, lw=1.2,
                            color=COR_TENSAO_MEDIA, animated=True)

ax5.set_title("I_final")
ax5.set_xlim(0, WINDOW_SIZE)
ax5.set_ylim(0, VREF)
ax5.set_ylabel("Mean Voltage (V)")
configurar_eixo_artigo(ax5)

# ---- NEURONIO POS / CUNEIFORMES ----
ax6.set_title("Cuneate Neuron Spike Raster")
ax6.set_xlim(-RASTER_WINDOW, 0)
ax6.set_ylim(0.3, 3.7)
ax6.set_xlabel("t - agora (s)")
ax6.set_yticks([1, 2, 3])
ax6.set_yticklabels(["CN_SA", "CN_FA", "CN_MM"])
configurar_eixo_artigo(ax6)

scatter_CN_SA = ax6.scatter([], [], s=14, color=COR_CNSA, animated=True)
scatter_CN_RA = ax6.scatter([], [], s=14, color=COR_CNRA, animated=True)
scatter_CN_MM = ax6.scatter([], [], s=14, color=COR_CNMM, animated=True)

plt.tight_layout()

fig.show()

# Fundo estatico (eixos, ticks, colorbar, legenda) capturado uma vez. Quando a
# janela e redimensionada ele e recapturado (bg volta a None).
_bg = None


def _on_resize(event):
    global _bg
    _bg = None


fig.canvas.mpl_connect("resize_event", _on_resize)


def update_plots():
    """Repinta apenas os artistas dinamicos sobre o fundo cacheado (blitting)."""
    global _bg
    now_t = time.monotonic()

    if _bg is None:
        fig.canvas.draw()
        _bg = fig.canvas.copy_from_bbox(fig.bbox)

    fig.canvas.restore_region(_bg)

    # ---- HEATMAP ----
    vr = np.rot90(voltage_matrix, 2)
    im_volt.set_data(vr)
    ax1.draw_artist(im_volt)
    for r in range(ROWS):
        for c in range(COLS):
            txt = texts_volt[r][c]
            txt.set_text(f"{vr[r, c]:.2f}")
            ax1.draw_artist(txt)

    # ---- RASTER FA / SA (tempo relativo; SA embaixo 1..25, FA em cima 26..50) ----
    x_SA, y_SA = [], []
    for n in range(NUM_TAXELS):
        spike_times_SA[n][:] = [t for t in spike_times_SA[n]
                                if now_t - t <= RASTER_WINDOW]
        for t in spike_times_SA[n]:
            x_SA.append(t - now_t)
            y_SA.append(n + 1)

    x_RA, y_RA = [], []
    for n in range(NUM_TAXELS):
        spike_times_RA[n][:] = [t for t in spike_times_RA[n]
                                if now_t - t <= RASTER_WINDOW]
        for t in spike_times_RA[n]:
            x_RA.append(t - now_t)
            y_RA.append(NUM_TAXELS + n + 1)

    scatter_SA.set_offsets(np.c_[x_SA, y_SA] if x_SA else np.empty((0, 2)))
    scatter_RA.set_offsets(np.c_[x_RA, y_RA] if x_RA else np.empty((0, 2)))
    ax2.draw_artist(scatter_SA)
    ax2.draw_artist(scatter_RA)

    # ---- I_final ----
    line_activation.set_data(x_fixed, list(activation_data))
    ax5.draw_artist(line_activation)

    # ---- CUNEIFORMES (tempo relativo; CN_SA=1, CN_FA=2, CN_MM=3) ----
    def _cn_offsets(buf, y):
        buf[:] = [t for t in buf if now_t - t <= RASTER_WINDOW]
        xs = [t - now_t for t in buf]
        return np.c_[xs, [y] * len(xs)] if xs else np.empty((0, 2))

    scatter_CN_SA.set_offsets(_cn_offsets(spike_times_CN_SA, 1))
    scatter_CN_RA.set_offsets(_cn_offsets(spike_times_CN_RA, 2))
    scatter_CN_MM.set_offsets(_cn_offsets(spike_times_CN_MM, 3))
    ax6.draw_artist(scatter_CN_SA)
    ax6.draw_artist(scatter_CN_RA)
    ax6.draw_artist(scatter_CN_MM)

    fig.canvas.blit(fig.bbox)
    fig.canvas.flush_events()


# =========================================================
# ARQUIVOS
# =========================================================

adc_file = open(
    os.path.join(
        SAVE_FOLDER,
        f"adc_{timestamp}.csv"
    ),
    "w",
    newline=""
)

spike_file = open(
    os.path.join(
        SAVE_FOLDER,
        f"spikes_{timestamp}.csv"
    ),
    "w",
    newline=""
)

cn_file = open(
    os.path.join(
        SAVE_FOLDER,
        f"cuneiformes_{timestamp}.csv"
    ),
    "w",
    newline=""
)

adc_writer = csv.writer(adc_file)
spike_writer = csv.writer(spike_file)
cn_writer = csv.writer(cn_file)

# =========================================================
# CABEÇALHOS
# =========================================================

adc_writer.writerow([
    "tempo",
    *[f"taxel_{i}" for i in range(NUM_TAXELS)]
])

spike_writer.writerow([
    "tempo",
    "tipo",
    "idx",
    "adc"
])

cn_writer.writerow([
    "tempo",
    "tipo"
])

# =========================================================
# SERIAL
# =========================================================

serial_buffer = ""

start_time = time.time()
last_plot = 0.0

print("Gravando...")
print(SAVE_FOLDER)

# =========================================================
# LOOP
# =========================================================

while (time.time() - start_time) < RECORD_TIME:

    elapsed = time.time() - start_time

    # =====================================================
    # INTERFACE
    # =====================================================

    label_timer.config(
        text=f"{elapsed:.1f} s"
    )

    if elapsed < 10:

        label_status.config(
            text="AGUARDE",
            bg="red"
        )

    elif elapsed < 20:

        label_status.config(
            text="APERTE O SENSOR",
            bg="green"
        )

    else:

        label_status.config(
            text="SOLTE O SENSOR",
            bg="gray"
        )

    root.update()

    # =====================================================
    # SERIAL
    # =====================================================

    if ser.in_waiting:

        serial_buffer += ser.read(
            ser.in_waiting
        ).decode(errors="ignore")

        lines = serial_buffer.split("\n")

        serial_buffer = lines[-1]

        for line in lines[:-1]:

            line = line.strip()

            # =============================================
            # ADC
            # =============================================

            if line.startswith("ADC"):

                try:

                    valores = line.split(",")

                    tstamp = int(
                        valores[-1].replace("t=", "")
                    ) / 1e6

                    adc_values = []

                    for v in valores[1:-1]:

                        v = v.strip()

                        if v.isdigit():
                            adc_values.append(int(v))

                    if len(adc_values) != 25:
                        continue

                    adc_writer.writerow([
                        tstamp,
                        *adc_values
                    ])

                    # ---- alimenta heatmap + I_final ----
                    voltage_matrix[:] = (
                        np.array(adc_values).reshape(ROWS, COLS)
                        * (VREF / 4095.0)
                    )
                    activation_data.append(float(voltage_matrix.mean()))

                except Exception as e:

                    print("Erro ADC:", e)

            # =============================================
            # RA
            # =============================================

            elif line.startswith("RA"):

                m = re.search(
                    r"idx=(\d+),adc=(\d+),t=(\d+)",
                    line
                )

                if m:

                    idx = int(m.group(1))
                    adc = int(m.group(2))
                    t = int(m.group(3))/1e6

                    spike_writer.writerow([
                        t,
                        "RA",
                        idx,
                        adc
                    ])

                    if 0 <= idx < NUM_TAXELS:
                        spike_times_RA[idx].append(time.monotonic())

            # =============================================
            # SA
            # =============================================

            elif line.startswith("SA"):

                m = re.search(
                    r"idx=(\d+),adc=(\d+),t=(\d+)",
                    line
                )

                if m:

                    idx = int(m.group(1))
                    adc = int(m.group(2))
                    t = int(m.group(3))/1e6

                    spike_writer.writerow([
                        t,
                        "SA",
                        idx,
                        adc
                    ])

                    if 0 <= idx < NUM_TAXELS:
                        spike_times_SA[idx].append(time.monotonic())

            # =============================================
            # CN_MM
            # =============================================

            elif line.startswith("CN_MM"):

                m = re.search(
                    r"t=(\d+)",
                    line
                )

                if m:

                    t = int(m.group(1))/1e6

                    cn_writer.writerow([
                        t,
                        "CN_MM"
                    ])

                    spike_times_CN_MM.append(time.monotonic())

            # =============================================
            # CN_RA
            # =============================================

            elif line.startswith("CN_RA"):

                m = re.search(
                    r"t=(\d+)",
                    line
                )

                if m:

                    t = int(m.group(1))/1e6

                    cn_writer.writerow([
                        t,
                        "CN_RA"
                    ])

                    spike_times_CN_RA.append(time.monotonic())

            # =============================================
            # CN_SA
            # =============================================

            elif line.startswith("CN_SA"):

                m = re.search(
                    r"t=(\d+)",
                    line
                )

                if m:

                    t = int(m.group(1))/1e6

                    cn_writer.writerow([
                        t,
                        "CN_SA"
                    ])

                    spike_times_CN_SA.append(time.monotonic())

    # =====================================================
    # GRAFICOS (atualiza no maximo a cada PLOT_PERIOD)
    # =====================================================

    now = time.time()
    if now - last_plot >= PLOT_PERIOD:
        update_plots()
        last_plot = now

# =========================================================
# FECHAMENTO
# =========================================================

adc_file.close()
spike_file.close()
cn_file.close()

ser.close()

root.destroy()

print("Gravação concluída.")

# Mantem os graficos abertos para inspecao ate o usuario fechar a janela.
# Desliga o modo animated e redesenha tudo, senao a janela final mostraria
# apenas o fundo (os artistas animated nao entram num draw normal).
for _artist in (im_volt, scatter_RA, scatter_SA, line_activation,
                scatter_CN_SA, scatter_CN_RA, scatter_CN_MM,
                *[t for row in texts_volt for t in row]):
    _artist.set_animated(False)

fig.canvas.draw()

plt.ioff()
plt.show()
