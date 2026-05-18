#!/usr/bin/env python3
"""
tune_rotate.py — calibração offline do parâmetro Rotate (polegar) por
preensão (T28 do SDD §9.1).

Para cada grip do projeto (palm / claw / fingertip), varre Rotate em
sliders ∈ {0, 30, 60, 90, 120, 150, 200} e reporta a distância
polegar→objeto via FK. O valor com menor distância (sem penetração) é
o ótimo.

Uso:
    cd /home/lucas-lpc/RoboticArm
    source install/setup.bash
    python3 src/grasp_ml_pack/scripts/tune_rotate.py
"""

from __future__ import annotations

import math
import sys

import numpy as np

sys.path.insert(0, 'src/grasp_ml_pack')

from grasp_ml_pack import poses
from grasp_ml_pack.kinematics import (
    forward_kinematics, fk_partial,
    T_HAND_ATTACH,
    _thumb_tip_in_hand, _finger_tip_in_hand_long,
)
from grasp_ml_pack.collision import PICK_OBJ_BBOX, ROBOT_BASE_Z


def thumb_tip_world(q_arm_rad: np.ndarray,
                     thumb_slider: int,
                     rotate_slider: int) -> np.ndarray:
    """Ponta do polegar em world frame, dado q do braço e sliders."""
    thumb_rad  = thumb_slider  / 200.0 * 1.6
    rotate_rad = rotate_slider / 200.0 * 1.0
    tip_hand = _thumb_tip_in_hand(thumb_rad, rotate_rad)
    T_base = np.eye(4); T_base[2, 3] = ROBOT_BASE_Z
    T_w_link6 = T_base @ fk_partial(q_arm_rad, 6)
    T_w_hand = T_w_link6 @ T_HAND_ATTACH
    tip_world = T_w_hand @ np.array([tip_hand[0], tip_hand[1],
                                       tip_hand[2], 1.0])
    return tip_world[:3]


def main():
    rotates = [0, 30, 60, 90, 120, 150, 200]
    pairs = [
        ('frasco', 'Palm Grip (frasco)'),
        ('tubo',   'Claw Grip (tubo)'),
        ('ampola', 'Fingertip Grip (ampola)'),
    ]

    print('tune_rotate.py — varredura do Rotate por preensão')
    print('=' * 70)

    for obj, grip in pairs:
        pose_deg = poses.PICK_POSES_DEG[obj]
        q_arm = np.array([math.radians(pose_deg[j])
                          for j in poses.ARM_JOINTS])
        cx, cy, cz, sx, sy, sz = PICK_OBJ_BBOX[obj]
        grip_target = poses.HAND_GRIPS[grip]
        thumb_slider = int(grip_target.get('Thumb', 0))

        print(f'\n--- {obj} ({grip}) — Thumb fixo = {thumb_slider} ---')
        print('  Rotate  | tip_world (m)               | d_to_obj (mm)')
        best = (1e9, 0)
        for rot in rotates:
            tip_w = thumb_tip_world(q_arm, thumb_slider, rot)
            # Distância do tip ao centro do AABB do objeto
            dx = abs(tip_w[0] - cx) - sx / 2
            dy = abs(tip_w[1] - cy) - sy / 2
            dz = abs(tip_w[2] - cz) - sz / 2
            outside = max(dx, dy, dz)
            d_mm = outside * 1000.0
            mark = '  ←' if abs(d_mm) < best[0] else ''
            if abs(d_mm) < best[0]:
                best = (abs(d_mm), rot)
            print(f'  {rot:4d}    | {tip_w.round(3).tolist()}    | {d_mm:+8.1f}{mark}')
        print(f'  → ótimo Rotate ≈ {best[1]} '
              f'(distância polegar↔objeto = {best[0]:.1f} mm)')


if __name__ == '__main__':
    main()
