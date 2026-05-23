"""
real_driver.py — Camada de comunicação com o controlador Dobot CR10 real.

Baseado em:
  - Dobot TCP-IP-Python-V4 SDK (github.com/Dobot-Arm/TCP-IP-Python-V4)
  - Dobot TCP/IP Remote Control Interface Guide V3 (2025-05-08)
  - Dobot CRStudio User Guide V4.13.0_V2.14.0

Portas TCP — firmware V4.x (CR10a V4.5.1):
    29999  TODOS os comandos    dashboard + motion — único socket de controlo
    30004  feedback @8 ms       struct 1440 B com q_actual, TCPForce (125 Hz)
    30005  feedback @200 ms     mesmo struct, taxa reduzida

  !! O porto 30003 (motion queue V3) NÃO existe no firmware V4. O driver
     tenta ligação por compatibilidade mas cai imediatamente para 29999.

Sintaxe dos comandos de motion no firmware V4 (DIFERENTE do V3):
    ServoJ  →  ServoJ(J1,...,J6,t=<s>,aheadtime=<n>,gain=<n>)  [keyword args]
    MovJ    →  MovJ(joint={J1,...,J6})                           [braces obrigatórias]
    V3 usava JointMovJ(…) e ServoJ posicional — retornam -10000/-50001 em V4.

Modo de uso típico (do GraspExecutor ou da GUI manual):

    drv = CR10RealDriver(ip='192.168.5.2', dry_run=False)
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
import math
import re
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


# ─── Portas oficiais (protocolo V3) ────────────────────────────────────────
DASH_PORT = 29999      # dashboard (Immediate commands)
MOTION_PORT = 30003    # motion queue commands (MovJ, ServoJ, JointMovJ…)
FEEDBACK_PORT = 30004  # struct 1440 B @ 8 ms (125 Hz)

# Offset (em bytes) do campo `q_actual` (6 × float64) dentro do struct de
# 1440 B do feedback. O valor 432 é o offset referenciado no SDK oficial
# (dobot_api.py do TCP-IP-CR-Python-V4). Se o controlador retornar valores
# incoerentes, validar com `read_feedback_raw()` e ajustar.
FEEDBACK_Q_ACTUAL_OFFSET = 432
# Offset (em bytes) do campo `actual_TCPForce` (6 × float64 = [Fx,Fy,Fz,Tx,Ty,Tz])
# dentro do struct de 1440 B do feedback. Assume layout:
#   q_actual(432) → qd_actual(480) → qdd_actual(528) → i_actual(576)
#   → tool_vector_actual(624) → TCPSpeed_actual(672) → TCPForce(720)
# Validar com `read_feedback_raw()` em diferentes firmwares e ajustar se o
# struct do TCP-IP-CR-Python-V4 desviar nessa versão do controlador.
FEEDBACK_TCP_FORCE_OFFSET = 720
FEEDBACK_PACKET_SIZE = 1440


@dataclass
class CR10RealDriverConfig:
    ip: str = '192.168.5.2'
    dashboard_port: int = DASH_PORT      # 29999
    motion_port: int = MOTION_PORT       # 30003 (queue commands)
    feedback_port: int = FEEDBACK_PORT   # 30004
    connect_timeout_s: float = 3.0
    recv_timeout_s: float = 1.0
    motion_connect_timeout_s: float = 1.0   # timeout curto p/ porto 30003
    speed_factor: int = 50            # 50% — responsivo para mirror slider
    collision_level: int = 3          # 0 = off; 3 = padrão CR
    payload_kg: float = 0.5           # mão COVVI ≈ 0.5 kg
    payload_cog_m: tuple = (0.0, 0.0, 0.05)
    servoj_period_s: float = 0.030    # 33 Hz recomendado
    servoj_lookahead: int = 50        # [20, 100]
    servoj_gain: int = 500            # [200, 1000]


class CR10RealDriverError(RuntimeError):
    """Erro genérico da camada CR10."""


class CR10RealDriver:
    """Encapsula os três sockets TCP do controlador CR10 (protocolo V3).

    Arquitetura de portas (doc TCP/IP Remote Control Interface Guide V3):
      - 29999 (dashboard):  Immediate commands — EnableRobot, RobotMode, …
      - 30003 (motion):     Queue commands — MovJ, ServoJ, JointMovJ, …
      - 30004 (feedback):   struct 1440 B @ 8 ms com q_actual, TCPForce

    O porto 30003 pode estar fechado antes de EnableRobot() ser chamado.
    O driver tenta ligação em connect() e novamente após enable().

    Pode operar em três regimes:
      * conectado, real:    `dry_run=False`, sockets abertos, comandos vão pro robô.
      * conectado, dry-run: `dry_run=True`, sockets NÃO são abertos; sends são logados.
      * desconectado:       sockets fechados, `is_connected()` retorna False.
    """

    def __init__(self, ip: str = '192.168.5.2', dry_run: bool = False,
                 config: CR10RealDriverConfig | None = None):
        self.cfg = config or CR10RealDriverConfig()
        self.cfg.ip = ip
        self.dry_run = dry_run

        self._dash: socket.socket | None = None
        self._move: socket.socket | None = None   # porto 30003 (motion queue)
        self._feed: socket.socket | None = None

        self._dash_lock = threading.Lock()    # serializa sends/recvs em 29999
        self._move_lock = threading.Lock()    # serializa sends/recvs em 30003
        self._feed_lock = threading.Lock()    # serializa recv no feedback (30004)
        self._enabled = False
        self._last_send_t = 0.0

    # ── conexão ──────────────────────────────────────────────────────────
    def is_connected(self) -> bool:
        """True se dashboard (29999) e feedback (30004) estão abertos."""
        if self.dry_run:
            return True
        return self._dash is not None and self._feed is not None

    def motion_port_open(self) -> bool:
        """True se o porto de motion (30003) está conectado."""
        if self.dry_run:
            return True
        return self._move is not None

    def _try_connect_motion_port(self) -> bool:
        """Tenta abrir o porto 30003. Retorna True se conseguiu."""
        if self.dry_run:
            return True
        if self._move is not None:
            return True
        try:
            s = socket.create_connection(
                (self.cfg.ip, self.cfg.motion_port),
                timeout=self.cfg.motion_connect_timeout_s)
            s.settimeout(self.cfg.recv_timeout_s)
            self._move = s
            log.info('[MOVE] Porto 30003 (motion) conectado')
            return True
        except OSError as e:
            log.warning('[MOVE] Porto 30003 indisponível (%s) — motion via 29999', e)
            self._move = None
            return False

    def connect(self) -> None:
        """Abre as conexões TCP. Idempotente — chamada repetida é no-op."""
        if self.dry_run:
            log.info('[DRY-RUN] connect() → noop')
            return
        if self.is_connected():
            return
        try:
            self._dash = socket.create_connection(
                (self.cfg.ip, self.cfg.dashboard_port),
                timeout=self.cfg.connect_timeout_s)
            self._feed = socket.create_connection(
                (self.cfg.ip, self.cfg.feedback_port),
                timeout=self.cfg.connect_timeout_s)
            self._dash.settimeout(self.cfg.recv_timeout_s)
            self._feed.settimeout(self.cfg.recv_timeout_s)
            # O dashboard envia um banner na conexão; descartá-lo antes de
            # enviar comandos para não deslocar o emparelhamento cmd→resposta.
            self._drain_welcome()
            # Porto 30003 pode já estar aberto; se não estiver, tentamos
            # novamente em enable() após EnableRobot().
            self._try_connect_motion_port()
        except OSError as exc:
            self.close()
            raise CR10RealDriverError(
                f'Falha ao abrir sockets para {self.cfg.ip}: {exc}') from exc

    def close(self) -> None:
        """Fecha as conexões TCP. Não desabilita o robô — chame stop() antes."""
        for attr in ('_dash', '_move', '_feed'):
            s = getattr(self, attr)
            if s is not None:
                try:
                    s.close()
                except OSError:
                    pass
            setattr(self, attr, None)
        self._enabled = False

    def _drain_welcome(self) -> None:
        """Descarta o banner inicial enviado pelo dashboard (porta 29999)."""
        if self._dash is None:
            return
        self._dash.settimeout(0.5)
        try:
            buf = b''
            while True:
                chunk = self._dash.recv(4096)
                if not chunk:
                    break
                buf += chunk
                if b'\n' in buf or b';' in buf:
                    break
        except socket.timeout:
            pass
        finally:
            self._dash.settimeout(self.cfg.recv_timeout_s)
        if buf:
            log.info('[DASH] welcome: %s',
                     buf.decode('ascii', errors='replace').strip())

    # ── primitivas TCP ASCII ─────────────────────────────────────────────
    @staticmethod
    def _recv_line(sock: socket.socket) -> str:
        """Lê uma linha de resposta ASCII terminada em ';' ou '\\n'."""
        buf = b''
        try:
            while b'\n' not in buf and b';' not in buf:
                chunk = sock.recv(2048)
                if not chunk:
                    break
                buf += chunk
        except socket.timeout:
            pass
        return buf.decode('ascii', errors='replace').strip()

    def _send_dash(self, cmd: str, expect_reply: bool = True) -> str:
        """Envia Immediate command ao dashboard (29999) e devolve a resposta."""
        if self.dry_run:
            log.info('[DRY-RUN dash] %s', cmd)
            return ''
        if self._dash is None:
            raise CR10RealDriverError('Dashboard não conectado')
        with self._dash_lock:
            self._dash.sendall((cmd + '\n').encode('ascii'))
            if not expect_reply:
                return ''
            return self._recv_line(self._dash)

    def _send_move(self, cmd: str) -> str:
        """Envia Queue command pelo porto de motion (30003) e devolve a resposta."""
        if self.dry_run:
            log.info('[DRY-RUN move] %s', cmd)
            return ''
        if self._move is None:
            raise CR10RealDriverError('Porto de motion (30003) não conectado')
        with self._move_lock:
            self._move.sendall((cmd + '\n').encode('ascii'))
            return self._recv_line(self._move)

    def _send_motion(self, cmd: str) -> str:
        """Envia queue command de motion e devolve a resposta.

        Usa porto 30003 (protocolo V3) quando disponível; recorre a 29999
        como fallback (pode retornar -30001 em algumas versões de firmware).
        """
        self._last_send_t = time.time()
        if self._move is not None:
            return self._send_move(cmd)
        log.debug('[MOTION via 29999] %s', cmd)
        return self._send_dash(cmd, expect_reply=True)

    # ── sequência de bring-up ────────────────────────────────────────────
    def _wait_mode(self, target: int, timeout_s: float = 8.0) -> bool:
        """Espera até RobotMode() == target. Retorna True se alcançado."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            resp = self.robot_mode() or ''
            m = re.search(r'\{(\d+)\}', resp)
            if m and int(m.group(1)) == target:
                return True
            time.sleep(0.2)
        return False

    def enable(self) -> None:
        """Executa a sequência de enable do CR10 (protocolo V4, firmware V4.5.1).

        Sequência V4:
          1. ClearError()  — limpa alarmes
          2. PowerOn()     — ativa subsistema de potência (modo 4 → pré-enable)
                             necessário quando o robô está em modo 4 (DISABLED)
          3. EnableRobot() — habilita os servos (assíncrono: aguarda modo 5)
          4. Se mode 5 não chegar em 8s, tenta EnableRobot(load,cx,cy,cz)
          5. SpeedFactor + SetCollisionLevel
        """
        if not self.is_connected() and not self.dry_run:
            raise CR10RealDriverError('Driver não está conectado')

        resp = self._send_dash('ClearError()')
        log.info('[DASH] ClearError → %s', resp)

        # PowerOn() ativa o subsistema de potência em V4. Ignorar erro se
        # já estiver ligado (pode retornar -2 / "already on").
        resp = self._send_dash('PowerOn()')
        log.info('[DASH] PowerOn → %s', resp)
        time.sleep(0.5)   # PowerOn é assíncrono; dar tempo ao firmware

        # EnableRobot() — V4: assíncrono, retorna imediatamente.
        # Tentar primeiro sem parâmetros (forma mais simples e compatível).
        resp = self._send_dash('EnableRobot()')
        log.info('[DASH] EnableRobot() → %s', resp)

        # Aguardar modo 5 (ENABLE). EnableRobot é assíncrono em V4.
        if not self._wait_mode(5, timeout_s=8.0):
            log.warning('[DASH] Modo 5 não atingido após EnableRobot() — '
                        'tentando EnableRobot(load,cx,cy,cz)...')
            cx, cy, cz = (v * 1000.0 for v in self.cfg.payload_cog_m)
            resp2 = self._send_dash(
                f'EnableRobot({self.cfg.payload_kg:.3f},{cx:.1f},{cy:.1f},{cz:.1f})')
            log.info('[DASH] EnableRobot(load,cog) → %s', resp2)
            if not self._wait_mode(5, timeout_s=8.0):
                mode = self.robot_mode()
                raise CR10RealDriverError(
                    f'EnableRobot falhou — modo atual: {mode}. '
                    f'Verifique E-STOP / botão físico no controlador.')

        # Porto 30003 pode só abrir após EnableRobot() — tentar novamente
        if not self.motion_port_open():
            self._try_connect_motion_port()

        resp = self._send_dash(f'SpeedFactor({self.cfg.speed_factor})')
        log.info('[DASH] SpeedFactor → %s', resp)
        resp = self._send_dash(f'SetCollisionLevel({self.cfg.collision_level})')
        log.info('[DASH] SetCollisionLevel → %s', resp)
        self._enabled = True
        log.info('CR10 habilitado em %s (motion_port=%s, SpeedFactor=%d, '
                 'Coll=%d, Payload=%.2fkg)',
                 self.cfg.ip,
                 '30003' if self.motion_port_open() else '29999(fallback)',
                 self.cfg.speed_factor, self.cfg.collision_level,
                 self.cfg.payload_kg)

    def prepare_servoj(self) -> None:
        """Reinicia estado interno antes de iniciar o streaming ServoJ.

        Chamar APÓS sync() (PTP de alinhamento concluído) e ANTES do
        primeiro ServoJ. No firmware V4.5.1 (protocolo V3), ServoJ
        retorna -50001 quando iniciado imediatamente após JointMovJ —
        ClearError() + 200 ms de estabilização resolve a transição.
        """
        if self.dry_run:
            return
        resp = self._send_dash('ClearError()')
        log.info('[DASH] prepare_servoj ClearError → %s', resp)
        time.sleep(0.2)
        mode = self.robot_mode()
        log.info('[DASH] RobotMode antes do primeiro ServoJ: %s', mode)

    def stop(self) -> None:
        """Parada por software — Stop() seguido de DisableRobot().

        V4 firmware usa Stop() (não StopRobot/ResetRobot, que retornam -10000).
        NÃO substitui o botão físico de E-STOP do controlador.
        """
        try:
            self._send_dash('Stop()', expect_reply=False)
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
        q_deg = [math.degrees(v) for v in q]
        # V4 firmware: ServoJ(J1,...,J6,t=<s>,aheadtime=<n>,gain=<n>)
        # Os parâmetros opcionais são KEYWORD ARGS (não posicionais).
        # V3 usava ServoJ(J1,...,J6,t,la,g) posicional → retorna -50001 em V4.
        cmd = 'ServoJ({values},t={t:.3f},aheadtime={la},gain={g})'.format(
            values=','.join(f'{v:.6f}' for v in q_deg),
            t=self.cfg.servoj_period_s,
            la=self.cfg.servoj_lookahead,
            g=self.cfg.servoj_gain)
        resp = self._send_motion(cmd)
        if resp and not resp.startswith('0'):
            code = resp.split(',')[0].strip()
            log.warning('[MOVE] ServoJ erro %s', code)
            if code in ('-50001', '-1', '-2', '-3'):
                # -50001: servo subsystem não pronto (estado pós-PTP).
                # -1: robô desabilitado / comando não executável agora.
                # -2: robô em alarme. -3: modo errado para ServoJ.
                raise CR10RealDriverError(f'ServoJ não executável ({code})')

    def servo_j_urdf(self, q_urdf: Sequence[float]) -> None:
        """Wrapper que aplica `urdf_to_dobot` antes de chamar `servo_j`."""
        q = np.asarray(q_urdf, dtype=np.float64)
        if _HAS_CONV:
            q = urdf_to_dobot(q)
        self.servo_j(q.tolist())

    def mov_j_joint_deg(self, q_deg: Sequence[float]) -> None:
        """MovJ articular — PTP em GRAUS (convenção DOBOT).

        V4 firmware: MovJ(joint={J1,...,J6}) — braces obrigatórias.
        V3 usava JointMovJ(J1,...,J6) → retorna -10000 em V4.
        """
        q = list(q_deg)
        if len(q) != 6:
            raise ValueError(f'mov_j_joint_deg requer 6 valores')
        # V4: MovJ(joint={J1,J2,J3,J4,J5,J6}) — braces no argumento joint
        cmd = 'MovJ(joint={{{values}}})'.format(
            values=','.join(f'{v:.6f}' for v in q))
        resp = self._send_motion(cmd)
        if resp and not resp.startswith('0'):
            log.warning('[MOVE] JointMovJ erro: %s', resp.split(',')[0].strip())

    def sync(self, timeout_s: float = 30.0) -> None:
        """Bloqueia até o robô terminar o movimento (RobotMode == 5).

        Sync() não existe no firmware V4.5.1 — usa polling de RobotMode().
        Modo 7 = Running; modo 5 = Enabled/Idle (movimento concluído).
        """
        if self.dry_run:
            return
        # The firmware takes a few hundred ms to process a MovJ command and
        # enter mode 7 (RUNNING). Polling immediately can see mode 5 and
        # return before the motion starts — causing ServoJ to be sent while
        # the PTP move is still pending, which triggers -50001.
        time.sleep(0.5)
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            resp = self.robot_mode() or ''
            # resposta: "0,{5},RobotMode();" — extrair o inteiro entre { }
            m = re.search(r'\{(\d+)\}', resp)
            if m and int(m.group(1)) not in (7, 8):
                return
            time.sleep(0.1)
        log.warning('sync() timeout após %.1f s', timeout_s)

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
        """Lê as 6 juntas atuais em radianos (convenção DOBOT).

        O struct de feedback armazena q_actual em GRAUS; esta função converte
        para radianos antes de devolver, para consistência com servo_j e
        dobot_to_urdf.
        """
        if self.dry_run:
            return np.zeros(6, dtype=np.float64)
        buf = self.read_feedback_raw()
        q_deg = np.frombuffer(
            buf, offset=FEEDBACK_Q_ACTUAL_OFFSET,
            count=6, dtype='<f8').copy()
        return np.deg2rad(q_deg)

    def read_joints_urdf(self) -> np.ndarray:
        """Idem, mas já na convenção URDF (joint2 e joint4 ajustados)."""
        q = self.read_joints_rad()
        if _HAS_CONV:
            q = dobot_to_urdf(q)
        return q

    def read_tcp_force(self) -> np.ndarray:
        """Lê o wrench externo estimado no TCP a partir do feedback do CR10.

        O controlador da Dobot estima [Fx, Fy, Fz, Tx, Ty, Tz] subtraindo
        o modelo dinâmico (gravidade + PayLoad declarado) dos torques
        medidos em cada uma das 6 juntas. NÃO é um F/T externo — a
        qualidade depende do `PayLoad(...)` estar calibrado e há drift de
        zero que deve ser compensado externamente (tarar antes do contato).

        Returns:
            np.ndarray (6,) — [Fx, Fy, Fz, Tx, Ty, Tz] em N / N·m no
            frame do TCP. Em `dry_run`, retorna zeros.
        """
        if self.dry_run:
            return np.zeros(6, dtype=np.float64)
        buf = self.read_feedback_raw()
        return np.frombuffer(
            buf, offset=FEEDBACK_TCP_FORCE_OFFSET,
            count=6, dtype='<f8').copy()

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

    def get_error_id(self) -> str | None:
        """GetErrorID() — códigos de alarme activos no controlador."""
        try:
            return self._send_dash('GetErrorID()')
        except CR10RealDriverError:
            return None

    # ── DO da flange (24 V já alimenta a COVVI; ToolDOExecute opcional) ──
    def tool_do(self, index: int, on: bool) -> None:
        """ToolDOExecute(idx, 1|0) — DO_1/DO_2 do conector aviation M8."""
        self._send_dash(f'ToolDOExecute({index},{1 if on else 0})')

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
