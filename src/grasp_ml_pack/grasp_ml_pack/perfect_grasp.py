"""
perfect_grasp.py — fechamento incremental da mão COVVI com detecção de
contato por LAG ARTICULAR (commanded vs actual position).

Problema resolvido
──────────────────
Sem detecção de contato, comandar `Index=1.6 rad` enquanto o dedo está
em 0 rad faz o PID do trajectory controller saturar no esforço máximo
(8 N·m) — o impulso resultante na borda do objeto é grande o bastante
para ejetá-lo da palma antes do atrito desenvolver.

Solução (padrão industrial: Robotiq adaptive, Schunk SDH "force closure")
──────────────────────────────────────────────────────────────────────
Rampa o target em N passos pequenos. Após cada passo:
  1. lê o JointState atual;
  2. para cada dedo, calcula `lag = target_commanded − pos_actual`;
  3. se `lag > LAG_THRESHOLD` por `STALL_TICKS` consecutivos → dedo está
     bloqueado por contato. CONGELA esse dedo em `pos_actual` e tira-o
     da lista de dedos ativos;
  4. continua rampa apenas com dedos ainda livres;
  5. termina quando TODOS os dedos contataram OU o target final foi
     alcançado OU o timeout expirou.

Cada dedo conforma à geometria do objeto sem aplicar força excessiva,
pois nenhum dedo individual passa do ponto de contato — a força é só
o ganho residual da rigidez do PID com lag pequeno.

Uso
───
    pg = PerfectGrasp(node)
    result = pg.close_until_contact(target_cfg, label='frasco/palm_grip')
    # result.stalled_fingers: lista dos dedos que contataram
    # result.final_cfg: posição final de cada dedo (congelada ou no target)
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# Juntas primárias da mão (driver joints). Os 25 mimic seguem em ratio.
_PRIMARY = ('Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Rotate')

# Mapa de mimic joints → (primary, ratio) — duplicado de grasp_executor
# para o módulo ser autocontido (sem dependência circular).
_MIMIC_RATIOS: Dict[str, tuple] = {
    '_lisa_j01':            ('Rotate', 1.07338),
    '_thumb_chassis_j01':   ('Rotate', 1.53340),
    '_thumb_proximal_j01':  ('Thumb',  0.72022),
    '_thumb_distal_j01':    ('Thumb',  1.06686),
    '_thumb_link_j01':      ('Thumb',  0.76799),
    '_thumb_follower_j01':  ('Thumb',  0.93733),
    '_index_proximal_j01':  ('Index',  1.51604),
    '_index_distal_j01':    ('Index',  1.33574),
    '_index_knuckle_j01':   ('Index',  1.25182),
    '_index_follower_j01':  ('Index',  0.26423),
    '_index_link_j01':      ('Index',  1.33574),
    '_middle_proximal_j01': ('Middle', 1.51604),
    '_middle_distal_j01':   ('Middle', 1.34986),
    '_middle_knuckle_j01':  ('Middle', 1.25181),
    '_middle_follower_j01': ('Middle', 0.26423),
    '_middle_link_j01':     ('Middle', 1.34986),
    '_ring_proximal_j01':   ('Ring',   1.51604),
    '_ring_distal_j01':     ('Ring',   1.34878),
    '_ring_knuckle_j01':    ('Ring',   1.25182),
    '_ring_follower_j01':   ('Ring',   0.26423),
    '_ring_link_j01':       ('Ring',   1.34878),
    '_little_proximal_j01': ('Little', 1.51604),
    '_little_distal_j01':   ('Little', 1.31664),
    '_little_knuckle_j01':  ('Little', 1.25182),
    '_little_follower_j01': ('Little', 0.26423),
    '_little_link_j01':     ('Little', 1.31664),
}


@dataclass
class GraspResult:
    """Resumo do fechamento."""
    stalled: Dict[str, bool] = field(default_factory=dict)
    final_cfg_rad: Dict[str, float] = field(default_factory=dict)
    steps_taken: int = 0
    contact_detected: bool = False
    timed_out: bool = False
    elapsed_s: float = 0.0

    def summary(self) -> str:
        stalled_names = [j for j, v in self.stalled.items() if v]
        return (f'steps={self.steps_taken} '
                f'contact={"yes" if self.contact_detected else "no"} '
                f'stalled={stalled_names} '
                f'elapsed={self.elapsed_s:.2f}s')


class PerfectGrasp:
    """Wrapper de fechamento incremental com contact-detection.

    Não cria publishers/subscribers próprios — espera o `node` chamador já
    ter inscrição em `/joint_states` e action client para `hand_position_
    controller/follow_joint_trajectory`. O `latest_joint_state` é lido via
    callback compartilhado (mesmo do executor / manual_control).
    """

    # Parâmetros do algoritmo de fechamento
    STEP_RAD          = 0.06   # incremento por passo (~3.4°)
    STEP_DT           = 0.10   # 100 ms entre passos
    LAG_THRESHOLD_RAD = 0.04   # >2.3° de lag = bloqueio
    STALL_TICKS       = 2      # ticks consecutivos para confirmar contato
    TIMEOUT_S         = 4.0    # falha se levar mais que isso
    MIN_STEPS_BEFORE_CHECK = 3 # ignora detecção nos 3 primeiros passos

    def __init__(self, node: Node, send_hand_fn):
        """
        Args:
            node: rclpy Node (para logger + clock)
            send_hand_fn: callable(cfg_dict, duration_s) que envia o
                target de junta para o controller da mão (preempta o
                goal anterior, não bloqueia). O sender já existente
                em grasp_executor é compatível.
        """
        self._node = node
        self._send_hand = send_hand_fn
        self._lock = threading.Lock()
        self._latest_positions: Dict[str, float] = {}

    # ── leitura de JointState (delegado pelo nó hospedeiro) ──────────
    def update_from_joint_state(self, msg: JointState) -> None:
        """Chamar a partir do callback /joint_states do nó hospedeiro."""
        with self._lock:
            for n, p in zip(msg.name, msg.position):
                if n in _PRIMARY:
                    self._latest_positions[n] = float(p)

    def _read_actual(self) -> Dict[str, float]:
        with self._lock:
            return dict(self._latest_positions)

    # ── núcleo: closing loop ─────────────────────────────────────────
    def close_until_contact(self,
                            target_cfg_rad: Dict[str, float],
                            *, label: str = 'grasp',
                            preshape_cfg_rad: Optional[Dict[str, float]] = None
                            ) -> GraspResult:
        """Fecha a mão em direção ao `target_cfg_rad` (junta primária →
        radianos, mesmo formato de HAND_CONFIGS em kinematics.py)
        parando por dedo quando o lag indica contato.

        Args:
            target_cfg_rad: cfg-alvo em radianos para as 6 juntas
                primárias da mão (Thumb, Index, Middle, Ring, Little,
                Rotate). Mimic joints são expandidas automaticamente
                pelo `send_hand_fn` injetado.
            label: identificador no log.
            preshape_cfg_rad: opcional — pré-conforma a mão neste cfg
                antes do fechamento (sem detecção). Útil para garantir
                que o polegar e Rotate cheguem à posição correta antes
                dos dedos curvarem em torno do objeto.

        Returns:
            GraspResult com posição final de cada dedo.
        """
        target_rad = {j: float(target_cfg_rad.get(j, 0.0)) for j in _PRIMARY}

        # Etapa opcional: pré-shape (sem detecção de contato — apenas
        # posicionamento; assume que o pré-shape não toca o objeto).
        if preshape_cfg_rad is not None:
            pre_rad = {j: float(preshape_cfg_rad.get(j, 0.0)) for j in _PRIMARY}
            self._send_hand_rad(pre_rad, duration=0.6)
            time.sleep(0.65)

        # Estado inicial: lê posição atual de cada dedo
        time.sleep(0.05)  # pequena pausa para garantir JointState fresco
        actual = self._read_actual()
        if not actual:
            self._node.get_logger().warn(
                f'[perfect_grasp:{label}] sem JointState — fechamento '
                f'sem detecção (fallback aberto).')
            self._send_hand_rad(target_rad, duration=1.2)
            time.sleep(1.3)
            return GraspResult(
                final_cfg_rad=target_rad, steps_taken=0, timed_out=True)

        # Estado do algoritmo
        result = GraspResult()
        # Commanded[j] é o último target enviado para j (rampa monotônica)
        commanded: Dict[str, float] = {j: actual.get(j, 0.0) for j in _PRIMARY}
        stalled: Dict[str, bool] = {j: False for j in _PRIMARY}
        stall_ctr: Dict[str, int] = {j: 0 for j in _PRIMARY}

        t0 = time.time()
        step_i = 0
        max_steps = int(max(abs(target_rad[j] - commanded[j])
                             for j in _PRIMARY) / self.STEP_RAD) + 2

        self._node.get_logger().info(
            f'[perfect_grasp:{label}] iniciando — '
            f'max_steps≈{max_steps}, step={self.STEP_RAD:.3f}rad, '
            f'lag_thr={self.LAG_THRESHOLD_RAD:.3f}rad')

        while True:
            elapsed = time.time() - t0
            if elapsed > self.TIMEOUT_S:
                result.timed_out = True
                break

            step_i += 1
            # 1) Incrementa commanded em direção ao target — apenas para
            # dedos ainda ativos. Dedos stalled mantêm o commanded
            # congelado em pos_actual.
            any_active = False
            for j in _PRIMARY:
                if stalled[j]:
                    continue
                delta = target_rad[j] - commanded[j]
                if abs(delta) <= self.STEP_RAD:
                    commanded[j] = target_rad[j]
                else:
                    commanded[j] += math.copysign(self.STEP_RAD, delta)
                    any_active = True

            # 2) Envia o goal incremental (preempta o anterior)
            self._send_hand_rad(commanded, duration=self.STEP_DT * 1.5)

            # 3) Espera + lê actual
            time.sleep(self.STEP_DT)
            actual = self._read_actual()

            # 4) Detecção de stall (a partir do passo MIN_STEPS_BEFORE_CHECK)
            if step_i >= self.MIN_STEPS_BEFORE_CHECK:
                for j in _PRIMARY:
                    if stalled[j]:
                        continue
                    lag = commanded[j] - actual.get(j, commanded[j])
                    # Só conta stall quando o dedo está EFETIVAMENTE fechando
                    # (target > start) — abrir não dispara contato.
                    closing = target_rad[j] > self._latest_positions.get(j, 0.0)
                    if closing and lag > self.LAG_THRESHOLD_RAD:
                        stall_ctr[j] += 1
                        if stall_ctr[j] >= self.STALL_TICKS:
                            stalled[j] = True
                            # Congela commanded no pos_actual atual para
                            # parar de empurrar o objeto.
                            commanded[j] = actual.get(j, commanded[j])
                            result.contact_detected = True
                            self._node.get_logger().info(
                                f'[perfect_grasp:{label}] CONTATO em {j} '
                                f'(lag={lag:.3f}rad, pos={commanded[j]:.3f}rad) — congelado')
                    else:
                        stall_ctr[j] = max(0, stall_ctr[j] - 1)

            # 5) Critérios de término
            all_stalled = all(stalled[j] or
                              abs(commanded[j] - target_rad[j]) < 1e-4
                              for j in _PRIMARY)
            if all_stalled or not any_active:
                # Re-envia o último commanded para travar (sem step extra)
                # com effort do PID — o dedo já não vai mais andar mas o
                # PID continua aplicando torque residual = atrito de aperto.
                self._send_hand_rad(commanded, duration=0.4)
                time.sleep(0.4)
                break

        result.steps_taken = step_i
        result.elapsed_s = time.time() - t0
        result.stalled = dict(stalled)
        result.final_cfg_rad = dict(commanded)
        self._node.get_logger().info(
            f'[perfect_grasp:{label}] {result.summary()}')
        return result

    # ── envio com mimic expansion ────────────────────────────────────
    def _send_hand_rad(self, cfg_rad: Dict[str, float],
                       duration: float) -> None:
        """Envia o trajeto da mão (6 primárias + 25 mimic) via send_hand_fn."""
        self._send_hand(dict(cfg_rad), float(duration))
