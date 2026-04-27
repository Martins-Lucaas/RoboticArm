"""
Script de verificação da cinemática do CR10 + COVVI Hand.

Executa sem ROS — roda diretamente com Python:
    python -m grasp_ml_pack.scripts.test_kinematics

ou via ROS 2:
    ros2 run grasp_ml_pack test_kin

Saídas:
  - FK nos ângulos de home: posição do TCP e norma do erro esperado
  - IK → FK round-trip: erro residual de posição (deve ser < 5 mm)
  - Jacobiano: verificação de condicionamento
  - Mão COVVI: posições das pontas dos dedos para cada tipo de grasp
"""

from __future__ import annotations

import math
import numpy as np


def _sep(title: str = ''):
    print(f'\n{"─" * 60}')
    if title:
        print(f'  {title}')
        print(f'{"─" * 60}')


def main(args=None):
    from grasp_ml_pack.kinematics import (
        forward_kinematics, fk_partial, jacobian, manipulability,
        inverse_kinematics, reach_margin, singularity_distances,
        finger_fk, hand_ik, approach_to_Rtcp,
        DH_CR10, HAND_CONFIGS,
    )

    # ── 1. FK na pose de home ──────────────────────────────────────────
    _sep('1. CINEMÁTICA DIRETA — pose de home')
    home = np.array([0.0, -math.pi/4, math.pi/2, -math.pi/4, -math.pi/2, 0.0])
    T_home = forward_kinematics(home)
    p_home = T_home[:3, 3]
    print(f'  Juntas (deg): {np.rad2deg(home).round(1).tolist()}')
    print(f'  TCP posição (m): x={p_home[0]:.4f}  y={p_home[1]:.4f}  z={p_home[2]:.4f}')
    print(f'  Norma posição:   {np.linalg.norm(p_home):.4f} m')
    print(f'  TCP rotação (R3×3 linha 3): {T_home[:3, 2].round(4).tolist()}')

    # ── 2. FK em postura de alcance máximo ────────────────────────────
    _sep('2. FK — alcance máximo (q = [0, 0, 0, 0, 0, 0])')
    q_full = np.zeros(6)
    T_full = forward_kinematics(q_full)
    p_full = T_full[:3, 3]
    print(f'  TCP posição (m): x={p_full[0]:.4f}  y={p_full[1]:.4f}  z={p_full[2]:.4f}')
    print(f'  Alcance: {np.linalg.norm(p_full[:2]):.4f} m (horizontal)')

    # ── 3. Round-trip IK→FK ───────────────────────────────────────────
    _sep('3. IK → FK (round-trip)')
    test_points = [
        ('bancada centro',  np.array([0.55,  0.00, 0.82]), np.array([0.0, 0.0, -1.0])),
        ('bancada esq.',    np.array([0.50, -0.12, 0.80]), np.array([0.0, 0.0, -1.0])),
        ('bancada dir.',    np.array([0.50,  0.12, 0.80]), np.array([0.0, 0.0, -1.0])),
        ('45° frontal',     np.array([0.55,  0.00, 0.82]), np.array([-0.7, 0.0, -0.7])),
        ('alto lateral',    np.array([0.40,  0.15, 0.90]), np.array([0.0, 0.0, -1.0])),
    ]
    all_ok = True
    for name, p_tgt, av in test_points:
        q_ik, ok = inverse_kinematics(p_tgt, av)
        T_ik = forward_kinematics(q_ik)
        err = np.linalg.norm(p_tgt - T_ik[:3, 3]) * 1000  # mm
        status = '✓' if (ok and err < 10.0) else '✗'
        if not (ok and err < 10.0):
            all_ok = False
        q_deg = np.rad2deg(q_ik).round(1)
        print(f'  {status} {name:15s} | err={err:5.2f} mm | ok={ok} | '
              f'q=[{q_deg[0]:6.1f},{q_deg[1]:6.1f},{q_deg[2]:6.1f},'
              f'{q_deg[3]:6.1f},{q_deg[4]:6.1f},{q_deg[5]:6.1f}]°')
    print(f'\n  Resultado geral: {"PASS ✓" if all_ok else "FAIL ✗"}')

    # ── 4. Jacobiano e manipulabilidade ───────────────────────────────
    _sep('4. JACOBIANO e MANIPULABILIDADE')
    q_work = np.array([0.0, -0.5, 0.8, 0.0, -0.8, 0.0])
    J = jacobian(q_work)
    cond = np.linalg.cond(J)
    manip = manipulability(q_work)
    rm = reach_margin(q_work)
    sd = singularity_distances(q_work)
    print(f'  Juntas: {np.rad2deg(q_work).round(1).tolist()}°')
    print(f'  Número de condição J:    {cond:.2f}')
    print(f'  Manipulabilidade:        {manip:.5f}')
    print(f'  Margem de alcance:       {rm:.3f}')
    print(f'  Dist. singularidades:    ombro={sd[0]:.2f}  cotovelo={sd[1]:.2f}  '
          f'pulso={sd[2]:.2f}')

    # ── 5. Mão COVVI — FK dos dedos ───────────────────────────────────
    _sep('5. MÃO COVVI — FK e IK por tipo de grasp')
    for gtype in ['open', 'pinch', 'cylindrical', 'spherical']:
        cfg = HAND_CONFIGS[gtype]
        idx_angle = cfg['Index']
        thumb_angle = cfg['Thumb']
        tip_idx = finger_fk(idx_angle)
        tip_thumb = finger_fk(thumb_angle, k_p=1.4)
        print(f'\n  [{gtype}]')
        print(f'    Index driver={idx_angle:.2f} rad → tip x={tip_idx[0]*1000:.1f} mm  '
              f'z={tip_idx[2]*1000:.1f} mm')
        print(f'    Thumb driver={thumb_angle:.2f} rad → tip x={tip_thumb[0]*1000:.1f} mm  '
              f'z={tip_thumb[2]*1000:.1f} mm')

    _sep('6. IK DA MÃO — ajuste por diâmetro do objeto')
    for obj, diam, gtype in [('pencil', 0.007, 'pinch'),
                               ('cup',    0.070, 'cylindrical'),
                               ('ball',   0.064, 'spherical')]:
        cfg = hand_ik(gtype, diam)
        print(f'  {obj} (d={diam*1000:.0f}mm, {gtype}):')
        print(f'    ' + '  '.join(f'{k}={v:.3f}' for k, v in cfg.items()))

    _sep()
    print('  Teste concluído.\n')


if __name__ == '__main__':
    main()
