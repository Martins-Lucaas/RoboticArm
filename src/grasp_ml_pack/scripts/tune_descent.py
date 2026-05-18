#!/usr/bin/env python3
"""
tune_descent.py — calibração offline da descida extra na fase F4 do
ciclo de pick (T25 do SDD §9.1).

Para cada objeto, varre descent_extra_mm ∈ {-30, -20, -10, 0, +10,
+20, +30, +40, +50} e reporta:
  • colisão (sim/não) usando collision.pose_is_safe
  • folga da palma (hand_base_link origin) ao topo do objeto, em mm

A descida extra é aplicada como um ajuste linear empírico em joint2/3
(equivalente ao `_adjusted_pick_pose` da GUI).

Uso:
    cd /home/lucas-lpc/RoboticArm
    source install/setup.bash
    python3 src/grasp_ml_pack/scripts/tune_descent.py
"""

from __future__ import annotations

import math
import sys

import numpy as np

sys.path.insert(0, 'src/grasp_ml_pack')

from grasp_ml_pack import poses
from grasp_ml_pack.kinematics import forward_kinematics, fk_partial, T_HAND_ATTACH
from grasp_ml_pack.collision import (
    pose_is_safe, PICK_OBJ_BBOX, ROBOT_BASE_Z,
)


def adjusted_pick_pose(obj_class: str, descent_extra_mm: float) -> dict:
    base = dict(poses.PICK_POSES_DEG[obj_class])
    if abs(descent_extra_mm) < 1e-3:
        return base
    d = descent_extra_mm / 10.0
    base['joint2'] += 0.2 * d
    base['joint3'] -= 0.8 * d
    return base


def palm_z_world(q_rad: np.ndarray) -> float:
    """Z da origem do hand_base_link (palma) em world."""
    T_base = np.eye(4); T_base[2, 3] = ROBOT_BASE_Z
    T_w_link6 = T_base @ fk_partial(q_rad, 6)
    T_w_hand = T_w_link6 @ T_HAND_ATTACH
    # hand_base_link origin ≈ T_w_link6 origin (T_HAND_ATTACH só translada
    # 115mm em z local = no plano dos fingertips). Usamos Link6 origin.
    return float(T_w_link6[2, 3])


def main():
    rangos = [-30, -20, -10, 0, +10, +20, +30, +40, +50]
    print('tune_descent.py — varredura da descida extra na F4')
    print('=' * 70)
    for obj in ('frasco', 'ampola', 'tubo'):
        cx, cy, cz, sx, sy, sz = PICK_OBJ_BBOX[obj]
        obj_top = cz + sz / 2
        print(f'\n--- {obj}  topo={obj_top:.3f}m ---')
        print('  d_extra | safe | palm_z (m) | palm−topo (mm)')
        for d in rangos:
            pose_deg = adjusted_pick_pose(obj, d)
            q = np.array([math.radians(pose_deg[j])
                          for j in poses.ARM_JOINTS])
            safe, msg = pose_is_safe(q)
            palm_z = palm_z_world(q)
            gap_mm = (palm_z - obj_top) * 1000
            flag = 'OK ' if safe else 'XX '
            tag = '   ← ótimo (palma logo acima)' if (safe and 0 <= gap_mm <= 30) else ''
            print(f'  {d:+4d}    | {flag}  | {palm_z:.3f}    | {gap_mm:+6.1f}{tag}')


if __name__ == '__main__':
    main()
