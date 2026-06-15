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
import threading
from collections import deque
from typing import Callable, Optional

import numpy as np

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
ROWS = 4
COLS = 4
NUM_TAXELS = ROWS * COLS
VREF = 3.3
RASTER_WINDOW = 5.0      # s de histórico no raster
WINDOW_SIZE = 50         # amostras no gráfico de I_final

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
                 on_sample: Optional[Callable[[float], None]] = None):
        # port=None → auto-detect no start().
        self._port_req = port
        self.port: Optional[str] = None
        self._on_sample = on_sample

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._ser = None
        self._thread: Optional[threading.Thread] = None
        self._buffer = ""

        self.connected = False
        self.error: Optional[str] = None

        # Estado compartilhado (protegido por self._lock).
        self.voltage_matrix = np.zeros((ROWS, COLS))
        self.spike_times_RA = [[] for _ in range(NUM_TAXELS)]
        self.spike_times_SA = [[] for _ in range(NUM_TAXELS)]
        self.spike_times_POST: list[float] = []
        self.I_final_data: deque = deque([0.0] * WINDOW_SIZE, maxlen=WINDOW_SIZE)
        self.current_time = 0.0

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
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._worker, daemon=True, name="touch-serial")
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self.connected = False

    # ──────────────────────────────────────────────────────────────────
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

            self._buffer += chunk.decode(errors="ignore")
            lines = self._buffer.split("\n")
            self._buffer = lines[-1]

            for line in lines[:-1]:
                self._parse_line(line.strip())

    def _parse_line(self, line: str) -> None:
        if line.startswith("DATA"):
            m = RE_DATA.search(line)
            if m:
                idx = int(m.group(1))
                adc = int(m.group(2))
                tstamp = int(m.group(3)) / 1e6
                row, col = divmod(idx, COLS)
                with self._lock:
                    self.current_time = tstamp
                    self.voltage_matrix[row, col] = adc * (VREF / 4095.0)

        elif line.startswith("RA"):
            m = RE_IDX_T.search(line)
            if m:
                idx = int(m.group(1))
                tstamp = int(m.group(2)) / 1e6
                with self._lock:
                    self.spike_times_RA[idx].append(tstamp)

        elif line.startswith("SA"):
            m = RE_IDX_T.search(line)
            if m:
                idx = int(m.group(1))
                tstamp = int(m.group(2)) / 1e6
                with self._lock:
                    self.spike_times_SA[idx].append(tstamp)

        elif line.startswith("POST"):
            m = RE_POST.search(line)
            if m:
                tstamp = int(m.group(1)) / 1e6
                with self._lock:
                    self.spike_times_POST.append(tstamp)

        elif line.startswith("TOTAL"):
            m = RE_TOTAL.search(line)
            if m:
                try:
                    i_final = float(m.group(3))
                except ValueError:
                    return
                if not np.isfinite(i_final):
                    return
                with self._lock:
                    self.I_final_data.append(i_final)
                if self._on_sample is not None:
                    # Fora do lock: o callback (publish ROS) não deve segurar
                    # o lock de dados.
                    try:
                        self._on_sample(i_final)
                    except Exception:
                        pass

    # ──────────────────────────────────────────────────────────────────
    def snapshot(self) -> dict:
        """Retrato consistente do estado para renderização. Já poda a janela
        do raster (descarta spikes mais antigos que RASTER_WINDOW) e devolve
        cópias — o desenho roda fora do lock."""
        with self._lock:
            t_now = self.current_time
            volt = self.voltage_matrix.copy()
            for n in range(NUM_TAXELS):
                self.spike_times_RA[n] = [
                    t for t in self.spike_times_RA[n]
                    if t_now - t <= RASTER_WINDOW]
                self.spike_times_SA[n] = [
                    t for t in self.spike_times_SA[n]
                    if t_now - t <= RASTER_WINDOW]
            self.spike_times_POST[:] = [
                t for t in self.spike_times_POST
                if t_now - t <= RASTER_WINDOW]
            ra = [list(lst) for lst in self.spike_times_RA]
            sa = [list(lst) for lst in self.spike_times_SA]
            post = list(self.spike_times_POST)
            i_final = list(self.I_final_data)
            latest_v = float(self.I_final_data[-1]) if self.I_final_data else 0.0
        return {
            "t_now": t_now,
            "volt": volt,
            "ra": ra,
            "sa": sa,
            "post": post,
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
        self.fig = Figure(figsize=(9.5, 7.0), dpi=100, facecolor=facecolor)
        axs = self.fig.subplots(2, 2)
        ax1, ax2 = axs[0, 0], axs[0, 1]
        ax5, ax6 = axs[1, 0], axs[1, 1]
        self.ax_heat, self.ax_raster = ax1, ax2
        self.ax_ifinal, self.ax_post = ax5, ax6

        # ── Heatmap ───────────────────────────────────────────────────
        self.im_volt = ax1.imshow(
            np.zeros((ROWS, COLS)), cmap="jet",
            interpolation="bicubic", vmin=0, vmax=VREF)
        self.fig.colorbar(self.im_volt, ax=ax1)
        ax1.set_title("Tensão (0–3.3 V)")
        ax1.set_xticks(range(COLS))
        ax1.set_yticks(range(ROWS))
        self.texts_volt = [[
            ax1.text(c, r, "0", ha="center", va="center",
                     fontsize=8, color="white")
            for c in range(COLS)] for r in range(ROWS)]

        # ── Raster RA / SA (tempo relativo, eixo fixo) ────────────────
        ax2.set_title("Raster RA / SA")
        ax2.set_xlim(0, RASTER_WINDOW)
        ax2.set_ylim(-1, NUM_TAXELS * 2)
        ax2.set_xlabel("tempo (s, janela deslizante)")
        self.scatter_RA = ax2.scatter([], [], s=10, color="red", label="RA")
        self.scatter_SA = ax2.scatter([], [], s=10, color="blue", label="SA")
        ax2.legend(loc="upper right", fontsize=8)

        # ── I_final ───────────────────────────────────────────────────
        self.x_fixed = np.arange(WINDOW_SIZE)
        (self.line_I_final,) = ax5.plot(
            self.x_fixed, [0.0] * WINDOW_SIZE, lw=2)
        ax5.set_title("I_final")
        ax5.set_xlim(0, WINDOW_SIZE)
        ax5.set_ylim(-1000, 1000)

        # ── Neurônio pós ──────────────────────────────────────────────
        ax6.set_title("Neurônio Pós")
        ax6.set_xlim(0, RASTER_WINDOW)
        ax6.set_ylim(-1, 1)
        ax6.set_xlabel("tempo (s, janela deslizante)")
        self.scatter_POST = ax6.scatter([], [], s=20, color="black")

        try:
            self.fig.tight_layout()
        except Exception:
            pass

        # Artistas que mudam a cada frame — com blit=True o matplotlib
        # redesenha SOMENTE estes (não a figura inteira), o que elimina o
        # travamento e faz o raster deslizar suave. Os textos do heatmap
        # precisam entrar aqui, senão não são redesenhados sob o blit.
        self.blit_artists = [
            self.im_volt,
            self.scatter_RA,
            self.scatter_SA,
            self.scatter_POST,
            self.line_I_final,
        ] + [t for row in self.texts_volt for t in row]

    # ──────────────────────────────────────────────────────────────────
    def init_blit(self) -> list:
        """Estado inicial usado pelo blit para capturar o fundo limpo."""
        self.scatter_RA.set_offsets(np.empty((0, 2)))
        self.scatter_SA.set_offsets(np.empty((0, 2)))
        self.scatter_POST.set_offsets(np.empty((0, 2)))
        self.line_I_final.set_data(self.x_fixed, [0.0] * WINDOW_SIZE)
        return self.blit_artists

    # ──────────────────────────────────────────────────────────────────
    def update(self, *_frame) -> list:
        snap = self.source.snapshot()
        t_now = snap["t_now"]
        x_lo = max(0.0, t_now - RASTER_WINDOW)

        # Heatmap (rot90 ×2 = mesma orientação do plotter original).
        volt = snap["volt"]
        self.im_volt.set_data(np.rot90(volt, 2))
        for r in range(ROWS):
            for c in range(COLS):
                self.texts_volt[r][c].set_text(f"{volt[r, c]:.2f}")

        # Raster RA / SA.
        x_ra, y_ra = [], []
        ra = snap["ra"]
        for n in range(NUM_TAXELS):
            for t in ra[n]:
                x_ra.append(t - x_lo)
                y_ra.append(n)
        x_sa, y_sa = [], []
        sa = snap["sa"]
        for n in range(NUM_TAXELS):
            for t in sa[n]:
                x_sa.append(t - x_lo)
                y_sa.append(n + NUM_TAXELS)
        self.scatter_RA.set_offsets(_offsets(x_ra, y_ra))
        self.scatter_SA.set_offsets(_offsets(x_sa, y_sa))

        # Neurônio pós.
        x_post = [t - x_lo for t in snap["post"]]
        self.scatter_POST.set_offsets(_offsets(x_post, [0] * len(x_post)))

        # I_final.
        self.line_I_final.set_data(self.x_fixed, snap["i_final"])

        return self.blit_artists
