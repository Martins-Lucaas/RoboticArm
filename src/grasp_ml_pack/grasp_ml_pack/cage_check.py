"""
cage_check.py — verifica geometria de "engaiolamento" antes do fechamento.

Padrão da literatura de power-grasp top-down: antes de fechar a mão, os
fingertips precisam estar AO REDOR do objeto, não pairando ACIMA dele.
Se o fechamento é iniciado com os fingertips sobre o topo do objeto, o
PerfectGrasp empurra a mão para baixo no objeto antes de o lag-detection
disparar, ejetando ou esmagando o item.

Condição correta (top-down, palma virada para −Z):

  • ``tip_z ≤ object_top + ε_top``   (tip no nível ou abaixo do topo,
    nunca acima — senão fecha em cima do objeto)
  • ``r_tip ≥ object_radius − ε_pen`` (tip fora do cilindro horizontal
    com tolerância da pele — caso contrário já está penetrando)
  • ``r_tip ≤ object_radius + reach`` (tip dentro do alcance de
    fechamento — senão close_until_contact nem toca o objeto)

A função :func:`cage_status` retorna :class:`CageStatus`, com lista de
violações se algo está fora. **Não aborta**: o caller (executor) loga
warn e segue — FK aproximada pode produzir falsos-positivos, e melhor
deixar o PerfectGrasp tentar do que parar prematuramente.

Dedos cujo target primary ainda está próximo de ``HAND_LOWER[j]`` são
ignorados (não estão engaiolando — ex.: Middle/Ring/Little durante
fingertip-grip, que ficam em rest pose).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .collision import PICK_OBJ_BBOX
from .kinematics import HAND_LOWER, forward_kinematics, hand_fk


# Tolerâncias (metros)
_Z_TOP_TOL = 0.010   # 10 mm acima do topo ainda OK (FK + pele têm essa folga)
_R_PEN_TOL = 0.005   # 5 mm de "penetração" tolerada pela pele inflada
_R_REACH   = 0.060   # 60 mm de alcance horizontal a partir do tip atual

_CAGING_FINGERS = ('Thumb', 'Index', 'Middle', 'Ring', 'Little')


@dataclass
class CageStatus:
    valid: bool
    violations: list[str] = field(default_factory=list)

    def __bool__(self) -> bool:
        return self.valid

    def summary(self) -> str:
        if self.valid:
            return 'cage OK'
        return 'cage INVÁLIDO: ' + ' | '.join(self.violations)


def cage_status(q_arm: np.ndarray,
                hand_state: dict,
                obj_class: str,
                *,
                world_obj_pos: Optional[np.ndarray] = None,
                rest_eps: float = 0.03) -> CageStatus:
    """Avalia se a mão forma uma "gaiola" válida ao redor do objeto.

    Args:
        q_arm:      vetor (6,) de ângulos do braço CR10 em rad.
        hand_state: dict ``{junta: rad}`` com pose corrente da mão. Use
            o preshape que será aplicado JUST ANTES do fechamento.
        obj_class:  'frasco' | 'tubo' | 'ampola'.
        world_obj_pos: (opcional) posição mundial em (3,) — quando dada,
            sobrescreve o centro de :data:`PICK_OBJ_BBOX`. Use quando o
            objeto não está exatamente no slot nominal (ex.: tracker de
            ``/gazebo/model_states``).
        rest_eps:   tolerância para considerar um dedo "em rest pose"
            e ignorá-lo na checagem. Default 30 mrad (~1.7°).

    Returns:
        :class:`CageStatus`.
    """
    if obj_class not in PICK_OBJ_BBOX:
        return CageStatus(False, [f'objeto desconhecido: {obj_class!r}'])

    cx, cy, cz, sx, sy, sz = PICK_OBJ_BBOX[obj_class]
    if world_obj_pos is not None:
        cx = float(world_obj_pos[0])
        cy = float(world_obj_pos[1])
        cz = float(world_obj_pos[2])
    obj_r   = 0.5 * max(sx, sy)
    obj_top = cz + 0.5 * sz

    T_wh = forward_kinematics(np.asarray(q_arm, dtype=float), include_hand=True)
    R_wh = T_wh[:3, :3]
    t_wh = T_wh[:3, 3]

    fk = hand_fk(hand_state)

    violations: list[str] = []
    for finger in _CAGING_FINGERS:
        primary = float(hand_state.get(finger, 0.0))
        if primary <= HAND_LOWER[finger] + rest_eps:
            continue

        tip_world = R_wh @ fk[f'tip_{finger}'] + t_wh

        if tip_world[2] > obj_top + _Z_TOP_TOL:
            violations.append(
                f'{finger}: tip_z={tip_world[2]*1000:.0f}mm > '
                f'obj_top+{_Z_TOP_TOL*1000:.0f}mm '
                f'({(obj_top+_Z_TOP_TOL)*1000:.0f}mm) — risco fechar em cima')

        dx = tip_world[0] - cx
        dy = tip_world[1] - cy
        r_tip = float(np.hypot(dx, dy))

        if r_tip < obj_r - _R_PEN_TOL:
            violations.append(
                f'{finger}: r_tip={r_tip*1000:.0f}mm < obj_r-tol='
                f'{(obj_r-_R_PEN_TOL)*1000:.0f}mm — dedo penetrando objeto')
        elif r_tip > obj_r + _R_REACH:
            violations.append(
                f'{finger}: r_tip={r_tip*1000:.0f}mm > obj_r+reach='
                f'{(obj_r+_R_REACH)*1000:.0f}mm — fora do alcance')

    return CageStatus(valid=(not violations), violations=violations)
