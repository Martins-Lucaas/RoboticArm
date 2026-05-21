"""
real_driver.py — Camada de comunicação com o controlador Dobot CR10 real.

Baseado em:
  - Dobot CRStudio User Guide V4.13.0_V2.14.0 (cap. ServoJ p.187, portas p.161/p.216)
  - Dobot CR A Series User Guide V1.8 (cap.4 habilitação pp.30/40, pp.46-48 LAN)
  - github.com/Dobot-Arm/TCP-IP-Protocol-V4
  - github.com/Dobot-Arm/TCP-IP-CR-Python-V4

Portas TCP (sempre as três):
    29999  dashboard ASCII   habilitar / erro / DO / SpeedFactor
    30003  feedback 1440 B   leitura @125 Hz (8 ms) das juntas atuais
    30004  motion port       MovJ / MovL / ServoJ / ServoP / Sync

Modo de uso típico (do GraspExecutor ou da GUI manual):

    drv = CR10RealDriver(ip='192.168.5.1', dry_run=False)
    drv.connect()
    drv.enable()                                # ClearError + EnableRobot + presets
    drv.servo_j([0, math.pi/2, 0, math.pi/2, 0, 0])  # convenção DOBOT, RADIANO
    q = drv.read_joints_rad()                   # 6 floats em radianos
    drv.stop()                                  # DisableRobot
    drv.close()

Observações:
    - ServoJ NÃO é afetado por SpeedFactor; o ritmo é dado pelo seu intervalo
      de envio (recomendado 30 ms / 33 Hz).
    - Use `dry_run=True` para validar o pipeline sem hardware — todos os
      sends apenas vão para o log.
    - O conversor URDF↔DOBOT está em `kinematics.urdf_to_dobot` / `dobot_to_urdf`
      (offsets das juntas 2 e 4 = ±π/2). NUNCA passe q_urdf direto: use
      `drv.servo_j_urdf(q_urdf)` ou converta antes.
"""
from __future__ import annotations

import logging
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Iterable, Sequence

import numpy as np

# Tentar reusar a conversão URDF↔DOBOT do módulo kinematics; queda atrasada
# para evitar import circular se este módulo for usado isoladamente.
try:
    from .kinematics import urdf_to_dobot, dobot_to_urdf  # noqa: F401
    _HAS_CONV = True
except Exception:  # pragma: no cover
    _HAS_CONV = False

log = logging.getLogger('touch_pack.real_driver')


# ─── Portas oficiais ────────────────────────────────────────────────────────
DASH_PORT = 29999
MOTION_PORT = 30004
FEEDBACK_PORT = 30003

# Offset (em bytes) do campo `q_actual` (6 × float64) dentro do struct de
# 1440 B do feedback. O valor 432 é o offset referenciado no SDK oficial
# (dobot_api.py do TCP-IP-CR-Python-V4). Se o controlador retornar valores
# incoerentes, validar com `read_feedback_raw()` e ajustar.
FEEDBACK_Q_ACTUAL_OFFSET = 432
FEEDBACK_PACKET_SIZE = 1440


@dataclass
class CR10RealDriverConfig:
    ip: str = '192.168.5.1'
    dashboard_port: int = DASH_PORT
    motion_port: int = MOTION_PORT
    feedback_port: int = FEEDBACK_PORT
    connect_timeout_s: float = 3.0
    recv_timeout_s: float = 1.0
    speed_factor: int = 20            # 10–50 nos primeiros bring-ups
    collision_level: int = 3          # 0 = off; 3 = padrão CR
    payload_kg: float = 0.5           # mão COVVI ≈ 0.5 kg
    payload_cog_m: tuple = (0.0, 0.0, 0.05)
    servoj_period_s: float = 0.030    # 33 Hz recomendado
    servoj_lookahead: int = 50        # [20, 100]
    servoj_gain: int = 500            # [200, 1000]


class CR10RealDriverError(RuntimeError):
    """Erro genérico da camada CR10."""


class CR10RealDriver:
    """Encapsula os três sockets TCP do controlador CR10.

    Pode operar em três regimes:
      * conectado, real:   `dry_run=False`, sockets abertos, comandos vão pro robô.
      * conectado, dry-run: `dry_run=True`, sockets NÃO são abertos; sends são logados.
      * desconectado:       sockets fechados, `is_connected()` retorna False.
    """

    def __init__(self, ip: str = '192.168.5.1', dry_run: bool = False,
                 config: CR10RealDriverConfig | None = None):
        self.cfg = config or CR10RealDriverConfig()
        self.cfg.ip = ip
        self.dry_run = dry_run

        self._dash: socket.socket | None = None
        self._move: socket.socket | None = None
        self._feed: socket.socket | None = None

        self._lock = threading.Lock()         # serializa sends no dashboard/motion
        self._feed_lock = threading.Lock()    # serializa recv no feedback
        self._enabled = False
        self._last_send_t = 0.0

    # ── conexão ──────────────────────────────────────────────────────────
    def is_connected(self) -> bool:
        if self.dry_run:
            return True
        return all(s is not None for s in (self._dash, self._move, self._feed))

    def connect(self) -> None:
        """Abre as três conexões TCP. Idempotente — chamada repetida é no-op."""
        if self.dry_run:
            log.info('[DRY-RUN] connect() → noop')
            return
        if self.is_connected():
            return
        try:
            self._dash = socket.create_connection(
                (self.cfg.ip, self.cfg.dashboard_port),
                timeout=self.cfg.connect_timeout_s)
            self._move = socket.create_connection(
                (self.cfg.ip, self.cfg.motion_port),
                timeout=self.cfg.connect_timeout_s)
            self._feed = socket.create_connection(
                (self.cfg.ip, self.cfg.feedback_port),
                timeout=self.cfg.connect_timeout_s)
            for s in (self._dash, self._move, self._feed):
                s.settimeout(self.cfg.recv_timeout_s)
        except OSError as exc:
            self.close()
            raise CR10RealDriverError(
                f'Falha ao abrir sockets para {self.cfg.ip}: {exc}') from exc

    def close(self) -> None:
        """Fecha todas as conexões TCP. Não desabilita o robô — chame stop() antes."""
        for attr in ('_dash', '_move', '_feed'):
            s = getattr(self, attr)
            if s is not None:
                try:
                    s.close()
                except OSError:
                    pass
            setattr(self, attr, None)
        self._enabled = False

    # ── primitivas TCP ASCII ─────────────────────────────────────────────
    def _send_dash(self, cmd: str, expect_reply: bool = True) -> str:
        """Envia comando ao dashboard (29999) e devolve a string de resposta.

        Formato CR: comando ASCII terminado em \n; resposta termina em ';'.
        """
        if self.dry_run:
            log.info('[DRY-RUN dash] %s', cmd)
            return ''
        if self._dash is None:
            raise CR10RealDriverError('Dashboard não conectado')
        with self._lock:
            self._dash.sendall((cmd + '\n').encode('ascii'))
            if not expect_reply:
                return ''
            buf = b''
            try:
                while b';' not in buf:
                    chunk = self._dash.recv(2048)
                    if not chunk:
                        break
                    buf += chunk
            except socket.timeout:
                pass
        return buf.decode('ascii', errors='replace').strip()

    def _send_motion(self, cmd: str) -> None:
        """Envia comando na motion port (30004). ServoJ não retorna ack útil."""
        if self.dry_run:
            log.info('[DRY-RUN motion] %s', cmd)
            return
        if self._move is None:
            raise CR10RealDriverError('Motion port não conectado')
        with self._lock:
            self._move.sendall((cmd + '\n').encode('ascii'))
            self._last_send_t = time.time()

    # ── sequência de bring-up ────────────────────────────────────────────
    def enable(self) -> None:
        """Executa a sequência obrigatória do CR antes de qualquer movimento.

        CRStudio cap.4 / CR A Guide pp.30 e 40:
            ClearError, EnableRobot, SpeedFactor, SetCollisionLevel, PayLoad.
        """
        if not self.is_connected() and not self.dry_run:
            raise CR10RealDriverError('Driver não está conectado')
        self._send_dash('ClearError()')
        self._send_dash('EnableRobot()')
        self._send_dash(f'SpeedFactor({self.cfg.speed_factor})')
        self._send_dash(f'SetCollisionLevel({self.cfg.collision_level})')
        cg = self.cfg.payload_cog_m
        self._send_dash(
            f'PayLoad({self.cfg.payload_kg:.3f}, {cg[0]:.3f}, '
            f'{cg[1]:.3f}, {cg[2]:.3f})')
        self._enabled = True
        log.info('CR10 habilitado em %s (SpeedFactor=%d, Coll=%d, Payload=%.2fkg)',
                 self.cfg.ip, self.cfg.speed_factor, self.cfg.collision_level,
                 self.cfg.payload_kg)

    def stop(self) -> None:
        """Parada por software — StopRobot seguido de DisableRobot.

        NÃO substitui o botão físico de E-STOP do controlador.
        """
        try:
            self._send_dash('StopRobot()', expect_reply=False)
        except CR10RealDriverError:
            pass
        try:
            self._send_dash('DisableRobot()', expect_reply=False)
        except CR10RealDriverError:
            pass
        self._enabled = False

    # ── movimentação ─────────────────────────────────────────────────────
    def servo_j(self, q_rad: Sequence[float]) -> None:
        """ServoJ — fluxo de setpoints articulares em RADIANO (convenção DOBOT).

        Frequência recomendada: 33 Hz (período 30 ms).
        Não chama Sync(); é responsabilidade do chamador respeitar o ritmo.
        """
        q = list(q_rad)
        if len(q) != 6:
            raise ValueError(f'servo_j requer 6 valores, recebeu {len(q)}')
        cmd = (
            'ServoJ({{joint={{{values}}}}}, "t={t:.3f} '
            'lookahead_time={la} gain={g}")'
        ).format(
            values=','.join(f'{v:.6f}' for v in q),
            t=self.cfg.servoj_period_s,
            la=self.cfg.servoj_lookahead,
            g=self.cfg.servoj_gain)
        self._send_motion(cmd)

    def servo_j_urdf(self, q_urdf: Sequence[float]) -> None:
        """Wrapper que aplica `urdf_to_dobot` antes de chamar `servo_j`."""
        q = np.asarray(q_urdf, dtype=np.float64)
        if _HAS_CONV:
            q = urdf_to_dobot(q)
        self.servo_j(q.tolist())

    def mov_j_joint_deg(self, q_deg: Sequence[float]) -> None:
        """JointMovJ — PTP articular em GRAUS (convenção DOBOT)."""
        q = list(q_deg)
        if len(q) != 6:
            raise ValueError(f'mov_j_joint_deg requer 6 valores')
        cmd = ('JointMovJ({{joint={{{values}}}}})').format(
            values=','.join(f'{v:.6f}' for v in q))
        self._send_motion(cmd)

    def sync(self) -> None:
        """Bloqueia o controlador até a fila de movimento esvaziar."""
        self._send_motion('Sync()')

    # ── leitura ──────────────────────────────────────────────────────────
    def read_feedback_raw(self) -> bytes:
        """Lê um pacote completo de 1440 B do feedback port (8 ms)."""
        if self.dry_run:
            return b'\x00' * FEEDBACK_PACKET_SIZE
        if self._feed is None:
            raise CR10RealDriverError('Feedback port não conectado')
        with self._feed_lock:
            buf = b''
            while len(buf) < FEEDBACK_PACKET_SIZE:
                chunk = self._feed.recv(FEEDBACK_PACKET_SIZE - len(buf))
                if not chunk:
                    raise CR10RealDriverError('Feedback port fechado')
                buf += chunk
            return buf

    def read_joints_rad(self) -> np.ndarray:
        """Lê as 6 juntas atuais (convenção DOBOT, radianos)."""
        if self.dry_run:
            return np.zeros(6, dtype=np.float64)
        buf = self.read_feedback_raw()
        return np.frombuffer(
            buf, offset=FEEDBACK_Q_ACTUAL_OFFSET,
            count=6, dtype='<f8').copy()

    def read_joints_urdf(self) -> np.ndarray:
        """Idem, mas já na convenção URDF (joint2 e joint4 ajustados)."""
        q = self.read_joints_rad()
        if _HAS_CONV:
            q = dobot_to_urdf(q)
        return q

    # ── diagnóstico ──────────────────────────────────────────────────────
    def robot_mode(self) -> str | None:
        """RobotMode() — retorna a string crua do dashboard (5 = habilitado)."""
        try:
            return self._send_dash('RobotMode()')
        except CR10RealDriverError:
            return None

    def get_angle_deg(self) -> str | None:
        """GetAngle() — útil como sanity check fora do feedback estruturado."""
        try:
            return self._send_dash('GetAngle()')
        except CR10RealDriverError:
            return None

    # ── DO da flange (24 V já alimenta a COVVI; ToolDOExecute opcional) ──
    def tool_do(self, index: int, on: bool) -> None:
        """ToolDOExecute(idx, ON|OFF) — DO_1/DO_2 do conector aviation M8."""
        self._send_dash(f'ToolDOExecute({index}, {"ON" if on else "OFF"})')

    # ── context manager ─────────────────────────────────────────────────
    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            self.stop()
        finally:
            self.close()


# ─── helpers livres ────────────────────────────────────────────────────────
def resample_trajectory(q_points: Iterable[np.ndarray],
                         t_points: Iterable[float],
                         period_s: float = 0.030) -> list[np.ndarray]:
    """Reamostra uma trajetória articular (q_i, t_i) para uma malha uniforme.

    `t_points` em segundos a partir do tempo zero. Usado para fatiar o goal
    do action client em setpoints @33 Hz antes de despachar via ServoJ.
    """
    qs = [np.asarray(q, dtype=np.float64) for q in q_points]
    ts = list(t_points)
    if not qs or len(qs) != len(ts):
        return []
    t0, tf = ts[0], ts[-1]
    n = max(2, int(round((tf - t0) / period_s)) + 1)
    out: list[np.ndarray] = []
    for i in range(n):
        t = t0 + i * period_s
        if t >= tf:
            out.append(qs[-1])
            break
        # busca segmento
        j = 0
        while j + 1 < len(ts) and ts[j + 1] < t:
            j += 1
        a = (t - ts[j]) / max(1e-9, ts[j + 1] - ts[j])
        out.append(qs[j] + a * (qs[j + 1] - qs[j]))
    return out
