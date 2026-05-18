"""
collision.py — geometria de colisão compartilhada (mundo + links).

Centraliza:
  • _WORLD_OBSTACLES — bbox de cada obstáculo estático (esteira, caixas,
    paredes, câmera, prateleira, pedestal).
  • _LINK_STL_BOUNDS — bbox dos STLs dos links do CR10 em frame local.
  • _PICK_OBJ_BBOX  — bbox dos objetos picáveis (frasco/tubo/ampola).
  • _BIN_BBOX       — bbox externo das caixas de entrega.
  • Funções `arm_clears_world`, `arm_clears_bbox`, `bbox_overlap` que
    consultam `kinematics.fk_partial` para calcular AABB world dos links.

Reutilizado por `grasp_executor.py`, `poses.py` e (indiretamente)
`manual_control_node.py`. Não tem dependências ROS.
"""

from __future__ import annotations

import numpy as np


# Altura do base_link do robô no world frame.
ROBOT_BASE_Z: float = 0.405


# ── Bounding boxes dos links STL (frame local, m) ─────────────────────
LINK_STL_BOUNDS: dict[str, tuple] = {
    'base_link': (-0.139, +0.093, -0.093, +0.093,  0.000, +0.093),
    'Link1':     (-0.077, +0.077, -0.102, +0.077, -0.097, +0.109),
    'Link2':     (-0.669, +0.077, -0.077, +0.077, +0.102, +0.302),
    'Link3':     (-0.614, +0.061, -0.061, +0.061, -0.023, +0.124),
    'Link4':     (-0.046, +0.046, -0.068, +0.089, -0.067, +0.046),
    'Link5':     (-0.046, +0.046, -0.101, +0.067, -0.057, +0.046),
    'Link6':     (-0.045, +0.045, -0.055, +0.045, -0.042,  0.000),
}


# ── Objetos picáveis (world frame, cx cy cz sx sy sz) ─────────────────
PICK_OBJ_BBOX: dict[str, tuple] = {
    'frasco': (0.75, 0.00, 0.851, 0.090, 0.090, 0.090),
    'tubo':   (0.75, 0.00, 0.866, 0.030, 0.030, 0.120),
    'ampola': (0.75, 0.00, 0.844, 0.015, 0.015, 0.075),
}


# ── Caixas de entrega (world frame) ───────────────────────────────────
BIN_BBOX: dict[str, tuple] = {
    'box1': (-0.05, 0.65, 0.603, 0.270, 0.260, 0.205),
    'box2': ( 0.25, 0.65, 0.603, 0.270, 0.260, 0.205),
    'box3': ( 0.55, 0.65, 0.603, 0.270, 0.260, 0.205),
}


# ── Obstáculos estáticos (world frame, AABB) ──────────────────────────
WORLD_OBSTACLES: dict[str, tuple] = {
    'robot_pedestal':  ( 0.00,  0.00, 0.1875, 0.180, 0.180, 0.375),
    'belt_frame':      ( 0.95,  0.00, 0.400,  0.800, 0.360, 0.800),
    'belt_surface':    ( 0.95,  0.00, 0.803,  0.780, 0.340, 0.006),
    'belt_leg_front':  ( 0.65,  0.00, 0.200,  0.050, 0.300, 0.400),
    'belt_leg_back':   ( 1.25,  0.00, 0.200,  0.050, 0.300, 0.400),
    'camera_column':   ( 1.45,  0.00, 0.900,  0.040, 0.040, 1.800),
    'camera_arm':      ( 1.35,  0.00, 1.750,  0.200, 0.030, 0.030),
    'sort_shelf_body': ( 0.25,  0.65, 0.240,  0.860, 0.300, 0.480),
    'sort_shelf_top':  ( 0.25,  0.65, 0.493,  0.860, 0.300, 0.014),
    'wall_back':       ( 0.50, -0.90, 1.250,  3.000, 0.080, 2.500),
    'wall_left':       (-0.90,  0.30, 1.250,  0.080, 2.800, 2.500),
}


# ── Primitivas de AABB ────────────────────────────────────────────────
def bbox_overlap(a: tuple, b: tuple,
                 margin: float = 0.005) -> tuple[bool, float]:
    """True se AABBs `a` e `b` (cx,cy,cz,sx,sy,sz) se sobrepõem (com
    margem em metros). Retorna (colide, clearance_mm)."""
    cx1, cy1, cz1, sx1, sy1, sz1 = a
    cx2, cy2, cz2, sx2, sy2, sz2 = b
    ox = abs(cx1 - cx2) - (sx1 + sx2) / 2 - margin
    oy = abs(cy1 - cy2) - (sy1 + sy2) / 2 - margin
    oz = abs(cz1 - cz2) - (sz1 + sz2) / 2 - margin
    collides = ox < 0 and oy < 0 and oz < 0
    return collides, max(ox, oy, oz) * 1000.0


def link_world_aabb(q: np.ndarray, link_idx: int) -> tuple:
    """AABB world (cx,cy,cz,sx,sy,sz) do link `link_idx` (0=base, 1..6)."""
    from .kinematics import fk_partial
    T_base = np.eye(4); T_base[2, 3] = ROBOT_BASE_Z
    if link_idx == 0:
        T_world = T_base; key = 'base_link'
    else:
        T_world = T_base @ fk_partial(q, link_idx)
        key = f'Link{link_idx}'
    xmin, xmax, ymin, ymax, zmin, zmax = LINK_STL_BOUNDS[key]
    pts = []
    for x in (xmin, xmax):
        for y in (ymin, ymax):
            for z in (zmin, zmax):
                p = T_world @ np.array([x, y, z, 1.0])
                pts.append(p[:3])
    pts = np.array(pts)
    cx = float((pts[:, 0].min() + pts[:, 0].max()) / 2)
    cy = float((pts[:, 1].min() + pts[:, 1].max()) / 2)
    cz = float((pts[:, 2].min() + pts[:, 2].max()) / 2)
    sx = float(pts[:, 0].max() - pts[:, 0].min())
    sy = float(pts[:, 1].max() - pts[:, 1].min())
    sz = float(pts[:, 2].max() - pts[:, 2].min())
    return (cx, cy, cz, sx, sy, sz)


def arm_clears_bbox(q: np.ndarray, bbox: tuple,
                    links: tuple = (1, 2, 3, 4, 5, 6),
                    margin: float = 0.005) -> tuple[bool, str]:
    """True se nenhum dos `links` colide com `bbox`."""
    for li in links:
        la = link_world_aabb(q, li)
        coll, clr = bbox_overlap(la, bbox, margin=margin)
        if coll:
            return False, f'Link{li} penetra ({clr:.1f}mm)'
    return True, 'OK'


def arm_clears_world(q: np.ndarray,
                     links: tuple = (4, 5, 6),
                     margin: float = 0.010,
                     skip: set | None = None
                     ) -> tuple[bool, str]:
    """True se os `links` do braço não colidem com nenhum obstáculo de
    WORLD_OBSTACLES. `skip` permite ignorar obstáculos específicos.

    Por padrão usamos links 4-6 (punho + flange) porque a AABB de Link2
    do STL é grande demais e gera falsos positivos sobre obstáculos
    baixos. Para validação rigorosa de pose final, passe links=(1..6).
    """
    skip = skip or set()
    for name, bbox in WORLD_OBSTACLES.items():
        if name in skip:
            continue
        ok, msg = arm_clears_bbox(q, bbox, links=links, margin=margin)
        if not ok:
            return False, f'colisão com {name}: {msg}'
    return True, 'OK'


def pose_is_safe(q: np.ndarray,
                 margin_arm: float = 0.010,
                 margin_wrist: float = 0.005,
                 allow_object: str | None = None
                 ) -> tuple[bool, str]:
    """Valida uma pose IK contra TODOS os obstáculos do mundo:

      • links 1-3 (ombro/cotovelo): checados contra esteira, paredes,
        pedestal — margem `margin_arm` (10 mm).
      • links 4-6 (punho/flange): checados contra todos os obstáculos
        com margem `margin_wrist` (5 mm — mais permissivo porque a mão
        precisa chegar perto dos objetos).
      • `allow_object`: nome do objeto picável que a mão pode tocar
        (e.g. 'frasco' no instante do pick). Ignora apenas esse objeto.
    """
    # Subset 1: links 1-3 vs obstáculos volumosos (não ignoramos nada
    # exceto se allow_object aplicar — mas objetos picáveis estão na
    # esteira, não obstáculos)
    ok, msg = arm_clears_world(q, links=(1, 2, 3),
                                margin=margin_arm)
    if not ok:
        return False, msg
    # Subset 2: links 4-6 — margem menor
    ok, msg = arm_clears_world(q, links=(4, 5, 6),
                                margin=margin_wrist)
    if not ok:
        return False, msg
    return True, 'OK'
