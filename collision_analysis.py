#!/usr/bin/env python3
"""
Análise de colisão física real para todos os links do CR10.
Usa dimensões STL reais dos meshes e FK para cada link.
"""
import sys
import math
import numpy as np
from itertools import product as iproduct

sys.path.insert(0, 'src/grasp_ml_pack')
from grasp_ml_pack.kinematics import (
    fk_partial, forward_kinematics, inverse_kinematics, JOINT_MIN, JOINT_MAX
)

# ── Parâmetros do sistema ────────────────────────────────────────────────
BASE_Z      = 0.405   # altura do base_link no world frame
AP_CLEAR    = 0.15    # m — pré-abordagem
LIFT_H      = 0.22    # m — levantamento pós-grasp
TRANSIT_Z   = 1.15 - BASE_Z   # 0.745 m no robot frame (= 1.15 world)
MARGIN      = 0.005   # 5 mm de margem de colisão

_AV_DOWN    = np.array([0.0, 0.0, -1.0])
_HOME_Q     = np.array([0.0, 0.0, math.pi/2,
                        -math.pi/2, -math.pi/2, 0.0])
# Seeds compactos por objeto para pick_xy=(0.75,0).
# q_pick é calculado primeiro com o seed do objeto, depois q_ap seeded a partir de q_pick.
# frasco/ampola usam q4<0 (mesmo ramo); tubo usa q4>0 (apenas ramo estendido disponível).
_PICK_SEED_Q = {
    'frasco': np.array([0.411, -0.277, -1.335, -1.529,  0.411,  0.0  ]),
    'tubo':   np.array([0.411, -0.381, -1.652,  2.032, -0.411,  3.142]),
    'ampola': np.array([0.411, -0.277, -1.341, -1.524,  0.411,  0.0  ]),
}

# Seed compacto para IK de approach_box: q2 e q3 negativos mantêm o braço
# "dobrado para cima" e evitam que Link2/Link3 entrem na sort_shelf (z ≤ 0.48m).
# Funciona para os três boxes (x=−0.05, 0.25, 0.55 m; y=0.65 m; z=0.75 m world).
_APPROACH_BOX_SEED_Q = np.array([0.0, -0.4, -1.5, -1.3, 0.0, 0.0])

# Seed para via_box (z=1.15m world): converge para ramo compacto (q2≈-0.5,
# q3≈-0.8) com Link2/Link3 y_max<0.52 para todos os três destinos.
# Seed encadeado approach_box→via_box divergia para ramo errado em box2/box3.
_VIA_BOX_SEED_Q = np.array([0.5, -0.5, -0.8, -1.5, 0.5, 0.0])

# ── STL bounding boxes (frame do link) ──────────────────────────────────
# Formato: (xmin, xmax, ymin, ymax, zmin, zmax)
STL_BOUNDS = {
    'base_link': (-0.139, +0.093, -0.093, +0.093,  0.000, +0.093),
    'Link1':     (-0.077, +0.077, -0.102, +0.077, -0.097, +0.109),
    'Link2':     (-0.669, +0.077, -0.077, +0.077, +0.102, +0.302),
    'Link3':     (-0.614, +0.061, -0.061, +0.061, -0.023, +0.124),
    'Link4':     (-0.046, +0.046, -0.068, +0.089, -0.067, +0.046),
    'Link5':     (-0.046, +0.046, -0.101, +0.067, -0.057, +0.046),
    'Link6':     (-0.045, +0.045, -0.055, +0.045, -0.042,  0.000),
}

# ── Obstáculos estáticos: (cx, cy, cz, sx, sy, sz) world frame ──────────
OBS_STATIC = {
    'belt_frame':     (0.95,  0.0,   0.40,  0.80, 0.36,  0.80),
    'belt_surface':   (0.95,  0.0,   0.803, 0.78, 0.34,  0.006),
    'belt_guide_L':   (0.95,  0.240, 0.870, 0.80, 0.025, 0.13),
    'belt_guide_R':   (0.95, -0.240, 0.870, 0.80, 0.025, 0.13),
    'belt_leg_front': (0.65,  0.0,   0.20,  0.05, 0.30,  0.40),
    'sort_shelf':     (0.25,  0.65,  0.24,  0.86, 0.30,  0.48),
    'sort_shelf_top': (0.25,  0.65,  0.493, 0.86, 0.30,  0.014),
    'pedestal':       (0.0,   0.0,   0.1875,0.18, 0.18,  0.375),
}

# ── Objetos spawnados: pick objects (presentes apenas nas fases 0-3) ──────
# AABB do cilindro: sx=sy=2r (diâmetro), sz=altura, cz=belt_top+h/2
OBS_PICK = {
    'pick_frasco': (0.75, 0.00, 0.851, 0.090, 0.090, 0.090),  # r=42mm h=90mm
    'pick_tubo':   (0.75, 0.00, 0.866, 0.030, 0.030, 0.120),  # r=12mm h=120mm
    'pick_ampola': (0.75, 0.00, 0.844, 0.015, 0.015, 0.075),  # r=5mm  h=75mm
}

# ── Objetos spawnados: caixas de destino (sempre presentes) ──────────────
# Hull exterior: pose=(cx,0.65,0.535), paredes h=0.17, topo em z=0.705m.
# cx±0.135 em x, ±0.130 em y  →  sx=0.270, sy=0.260; sz=floor_h+wall_h=0.205
OBS_BINS = {
    'bin_frasco': (-0.05, 0.65, 0.603, 0.270, 0.260, 0.205),
    'bin_tubo':   ( 0.25, 0.65, 0.603, 0.270, 0.260, 0.205),
    'bin_ampola': ( 0.55, 0.65, 0.603, 0.270, 0.260, 0.205),
}

# Conjunto completo de obstáculos (usado por check_all_collisions)
OBS = {**OBS_STATIC, **OBS_BINS}

# ── Box positions (world frame) ──────────────────────────────────────────
BOXES = {
    'box1': np.array([-0.05, 0.65, 0.60]),
    'box2': np.array([ 0.25, 0.65, 0.60]),
    'box3': np.array([ 0.55, 0.65, 0.60]),
}

# ── Pick z world frame por objeto ───────────────────────────────────────
PICK_Z = {
    'frasco': 0.916,
    'tubo':   0.946,
    'ampola': 0.913,
}
PICK_XY = np.array([0.75, 0.00])


# ────────────────────────────────────────────────────────────────────────
# Funções auxiliares
# ────────────────────────────────────────────────────────────────────────

def w2r(pos):
    """World → robot base frame."""
    return np.array([pos[0], pos[1], pos[2] - BASE_Z])


def get_8_corners(bounds):
    """Retorna os 8 cantos do bounding box (frame local)."""
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    corners = []
    for x in (xmin, xmax):
        for y in (ymin, ymax):
            for z in (zmin, zmax):
                corners.append([x, y, z, 1.0])
    return np.array(corners).T  # 4×8


def transform_corners(T, bounds):
    """Transforma os 8 cantos pelo T 4×4. Retorna array (3,8)."""
    pts = get_8_corners(bounds)
    world_pts = T @ pts   # 4×8
    return world_pts[:3, :]  # 3×8


def aabb_of_corners(corners_3x8):
    """Calcula AABB a partir de corners (3×8). Retorna (cx,cy,cz,sx,sy,sz)."""
    xmin, xmax = corners_3x8[0].min(), corners_3x8[0].max()
    ymin, ymax = corners_3x8[1].min(), corners_3x8[1].max()
    zmin, zmax = corners_3x8[2].min(), corners_3x8[2].max()
    cx, cy, cz = (xmin+xmax)/2, (ymin+ymax)/2, (zmin+zmax)/2
    sx, sy, sz = xmax-xmin, ymax-ymin, zmax-zmin
    return (cx, cy, cz, sx, sy, sz), (xmin, xmax, ymin, ymax, zmin, zmax)


def aabb_overlap(a, b, margin=MARGIN):
    """Verifica overlap entre dois AABBs (cx,cy,cz,sx,sy,sz)."""
    cx1, cy1, cz1, sx1, sy1, sz1 = a
    cx2, cy2, cz2, sx2, sy2, sz2 = b
    ox = abs(cx1-cx2) - (sx1+sx2)/2 - margin
    oy = abs(cy1-cy2) - (sy1+sy2)/2 - margin
    oz = abs(cz1-cz2) - (sz1+sz2)/2 - margin
    # penetração em cada eixo (negativo = overlap)
    return ox, oy, oz


def check_collision(link_aabb, obs_aabb, margin=MARGIN):
    """Retorna (collides, clearance_mm).
    clearance = distância mínima de separação (negativa se colide)."""
    ox, oy, oz = aabb_overlap(link_aabb, obs_aabb, margin)
    # clearance = máximo das penetrações por eixo (positivo = livre)
    clearance = max(ox, oy, oz)  # se todos negativos → colide
    collides = (ox < 0 and oy < 0 and oz < 0)
    return collides, clearance * 1000.0  # mm


def get_link_transforms(q):
    """
    Retorna T world de cada link dado q (URDF, 6 juntas).
    base_link é fixo em world z=BASE_Z.
    Link1..6 = fk_partial(q, i) deslocado por BASE_Z.
    """
    T_world_base = np.eye(4)
    T_world_base[2, 3] = BASE_Z

    transforms = {}
    # base_link: fixo
    T_bl = T_world_base.copy()
    transforms['base_link'] = T_bl

    # Link i = T_world_base @ fk_partial(q, i)
    for i in range(1, 7):
        T_robot = fk_partial(q, i)   # frame do link i no frame base do robô
        transforms[f'Link{i}'] = T_world_base @ T_robot

    return transforms


def compute_link_aabbs(q):
    """Computa AABB world de cada link para configuração q."""
    transforms = get_link_transforms(q)
    link_aabbs = {}
    link_bounds = {}
    for link_name, T in transforms.items():
        bounds = STL_BOUNDS[link_name]
        corners = transform_corners(T, bounds)
        aabb, minmax = aabb_of_corners(corners)
        link_aabbs[link_name] = aabb
        link_bounds[link_name] = minmax
    return link_aabbs, link_bounds


def check_all_collisions(q, waypoint_name, obs_dict=None, margin=MARGIN):
    """Verifica colisões de todos os links com todos os obstáculos em obs_dict."""
    if obs_dict is None:
        obs_dict = OBS
    link_aabbs, link_bounds = compute_link_aabbs(q)
    collisions = []
    for link_name, link_aabb in link_aabbs.items():
        for obs_name, obs_tuple in obs_dict.items():
            collides, clearance = check_collision(link_aabb, obs_tuple, margin)
            if collides:
                collisions.append({
                    'waypoint': waypoint_name,
                    'link': link_name,
                    'obstacle': obs_name,
                    'clearance_mm': clearance,
                    'link_aabb': link_aabb,
                    'link_minmax': link_bounds[link_name],
                })
    return collisions, link_aabbs, link_bounds


# ────────────────────────────────────────────────────────────────────────
# IK helpers
# ────────────────────────────────────────────────────────────────────────

def ik_robot(pos_world, approach=_AV_DOWN, seed=None, elbow_up=False):
    """IK com posição world frame → converte para robot frame."""
    p_r = w2r(pos_world)
    q, ok = inverse_kinematics(p_r, approach, q_seed=seed, elbow_up=elbow_up)
    return q, ok


def fk_tcp_world(q):
    """TCP position no world frame."""
    T_r = forward_kinematics(q)
    p_r = T_r[:3, 3]
    return np.array([p_r[0], p_r[1], p_r[2] + BASE_Z])


# ────────────────────────────────────────────────────────────────────────
# Gera todos os waypoints para todos os objetos
# ────────────────────────────────────────────────────────────────────────

def generate_waypoints(obj_name, box_key):
    """
    Gera lista de (nome_waypoint, q, ok) para um objeto/caixa.
    Seeding idêntico ao grasp_executor.py (incluindo via_pick).
    """
    box_world = BOXES[box_key]
    pick_z_w  = PICK_Z[obj_name]
    pick_world = np.array([PICK_XY[0], PICK_XY[1], pick_z_w])

    waypoints = []

    # 1. HOME
    q_home = _HOME_Q.copy()
    waypoints.append(('HOME', q_home, True))

    # pick primeiro (seed por objeto); approach/via_pick/lift seeded a partir de q_pick.
    # Esta ordem garante que q_pick converge para o ramo compacto correto.
    q_pick, ok_pick = ik_robot(pick_world, seed=_PICK_SEED_Q[obj_name], elbow_up=False)

    ap_pick_world = np.array([PICK_XY[0], PICK_XY[1], pick_z_w + AP_CLEAR])
    q_ap, ok_ap = ik_robot(ap_pick_world, seed=q_pick, elbow_up=False)

    via_pick_world = np.array([PICK_XY[0], PICK_XY[1], BASE_Z + TRANSIT_Z])
    q_vp, ok_vp = ik_robot(via_pick_world, seed=q_ap, elbow_up=False)

    lift_world = np.array([PICK_XY[0], PICK_XY[1], pick_z_w + LIFT_H])
    q_lift, ok_lift = ik_robot(lift_world, seed=q_vp, elbow_up=False)

    # approach_box primeiro (seed compacto); via_box seeded de q_ap_box para
    # evitar que o IK diverga para ramo errado em z=1.15m para box2/box3.
    ap_box_world = np.array([box_world[0], box_world[1], box_world[2] + AP_CLEAR])
    q_ap_box, ok_ap_box = ik_robot(ap_box_world, seed=_APPROACH_BOX_SEED_Q, elbow_up=False)

    via_world = np.array([box_world[0], box_world[1], BASE_Z + TRANSIT_Z])
    q_via, ok_via = ik_robot(via_world, seed=_VIA_BOX_SEED_Q, elbow_up=False)

    # Append em ordem de movimento
    waypoints.append(('via_pick',      q_vp,     ok_vp))
    waypoints.append(('approach_pick', q_ap,     ok_ap))
    waypoints.append(('pick',          q_pick,   ok_pick))
    waypoints.append(('lift',          q_lift,   ok_lift))
    waypoints.append(('via_box',       q_via,    ok_via))
    waypoints.append(('approach_box',  q_ap_box, ok_ap_box))

    return waypoints


# ────────────────────────────────────────────────────────────────────────
# Análise descent pick: clearance Link2/Link3 vs belt_guides durante descida
# ────────────────────────────────────────────────────────────────────────

def analyze_descent_clearance(obj_name):
    """Análise do descenso approach_pick → pick."""
    pick_z_w  = PICK_Z[obj_name]
    ap_z_w    = pick_z_w + AP_CLEAR

    ap_world  = np.array([PICK_XY[0], PICK_XY[1], ap_z_w])
    pick_world = np.array([PICK_XY[0], PICK_XY[1], pick_z_w])

    q_pick, ok_pick = ik_robot(pick_world, seed=_PICK_SEED_Q[obj_name], elbow_up=False)
    q_ap, ok_ap     = ik_robot(ap_world,   seed=q_pick,                 elbow_up=False)

    steps = 20
    results = []
    guides = ['belt_guide_L', 'belt_guide_R']

    for i in range(steps + 1):
        alpha = i / steps
        q_step = q_ap + alpha * (q_pick - q_ap)
        tcp_w  = fk_tcp_world(q_step)

        _, link_bounds = compute_link_aabbs(q_step)
        link_aabbs, _ = compute_link_aabbs(q_step)

        for link_name in ['Link2', 'Link3']:
            for guide in guides:
                coll, clr = check_collision(link_aabbs[link_name], OBS[guide], margin=0.0)
                results.append({
                    'step': i,
                    'tcp_z_world': float(tcp_w[2]),
                    'link': link_name,
                    'guide': guide,
                    'clearance_mm': clr,
                    'collides': coll,
                })
    return results


# ────────────────────────────────────────────────────────────────────────
# Verificar quais links ficam abaixo da superfície da esteira durante pick
# ────────────────────────────────────────────────────────────────────────

def check_below_belt(q, waypoint_name):
    """Links cujo AABB max-z está abaixo de belt_surface.zmax = 0.806m world."""
    belt_surf_zmax = 0.803 + 0.006/2  # centro z + metade sz = 0.806
    _, link_bounds = compute_link_aabbs(q)
    below = []
    for link_name, (xmin,xmax,ymin,ymax,zmin,zmax) in link_bounds.items():
        if zmax < belt_surf_zmax:
            below.append((link_name, zmax))
    return below


# ────────────────────────────────────────────────────────────────────────
# Verificação exata: cantos STL vs obstáculo
# ────────────────────────────────────────────────────────────────────────

def corners_in_obs(q, link_name, obs_name):
    """Verifica se algum dos 8 cantos STL do link (em world frame) está dentro
    do obstáculo. Retorna (any_inside, n_inside, corners_world_3x8)."""
    T_world_base = np.eye(4)
    T_world_base[2, 3] = BASE_Z

    link_idx = int(link_name.replace('Link', ''))
    T_robot = fk_partial(q, link_idx)
    T_world = T_world_base @ T_robot

    bounds = STL_BOUNDS[link_name]
    corners_w = transform_corners(T_world, bounds)  # 3×8

    all_obs = {**OBS, **OBS_PICK}
    cx, cy, cz, sx, sy, sz = all_obs[obs_name]
    xmin, xmax = cx - sx/2, cx + sx/2
    ymin, ymax = cy - sy/2, cy + sy/2
    zmin, zmax = cz - sz/2, cz + sz/2

    n_inside = 0
    for i in range(8):
        px, py, pz = corners_w[:, i]
        if xmin <= px <= xmax and ymin <= py <= ymax and zmin <= pz <= zmax:
            n_inside += 1
    return n_inside > 0, n_inside, corners_w


def verify_aabb_hits(collision_list):
    """Roda verificação de cantos para todas as colisões AABB não óbvias.
    Imprime resultado e retorna lista de colisões reais confirmadas."""
    # belt_frame já é visual-only: ignora
    skip_obs = {'belt_frame'}
    real = []
    checked = set()
    for c in collision_list:
        obs = c['obstacle']
        if obs in skip_obs:
            continue
        # Recuperar q a partir do waypoint — precisamos do q correto.
        # A função recebe o dicionário com link_aabb mas não o q; usamos
        # o campo 'waypoint' como chave de identificação.
        key = (c['waypoint'], c['link'], obs)
        if key in checked:
            continue
        checked.add(key)
        # Nota: não temos q no dicionário de colisão; retornamos para o
        # chamador processar via waypoint_qs lookup.
        real.append(c)
    return real


# ────────────────────────────────────────────────────────────────────────
# MAIN
# ────────────────────────────────────────────────────────────────────────

def main():
    print("=" * 80)
    print("ANÁLISE DE COLISÃO FÍSICA REAL — CR10 + CÉLULA DE MANUFATURA")
    print("=" * 80)

    OBJECTS = [
        ('frasco', 'box1'),
        ('tubo',   'box2'),
        ('ampola', 'box3'),
    ]

    all_collisions = []
    all_below_belt = []
    waypoint_qs    = {}   # key = "obj/wp" → q (para corner check)

    # Waypoints em que o pick object já está presente (fases 0-3 do executor)
    PICK_PRESENT_WPS = {'HOME', 'via_pick', 'approach_pick', 'pick'}

    for obj_name, box_key in OBJECTS:
        print(f"\n{'─'*60}")
        print(f"OBJETO: {obj_name.upper()}  →  {box_key.upper()}")
        print(f"{'─'*60}")

        pick_obj_key = f'pick_{obj_name}'

        waypoints = generate_waypoints(obj_name, box_key)

        for wp_name, q, ok in waypoints:
            wp_key = f"{obj_name}/{wp_name}"
            waypoint_qs[wp_key] = q.copy()

            tcp_w = fk_tcp_world(q)
            print(f"\n  [{wp_name}]  IK_ok={ok}")
            print(f"    q = [{', '.join(f'{v:.3f}' for v in q)}]")
            print(f"    TCP world = [{tcp_w[0]:.4f}, {tcp_w[1]:.4f}, {tcp_w[2]:.4f}]")

            # Escolhe conjunto de obstáculos ativo para este waypoint
            if wp_name in PICK_PRESENT_WPS:
                # Objeto spawnado presente; verificar que braço não toca objeto.
                # Em 'pick' apenas Link6 pode entrar no objeto (mão toca): excluir da check.
                obs_active = dict(OBS)
                obs_active[f'pick_{obj_name}'] = OBS_PICK[f'pick_{obj_name}']
            else:
                obs_active = dict(OBS)

            collisions_raw, link_aabbs, link_bounds = check_all_collisions(q, wp_key, obs_active)

            # Em pick: Link6 pode tocar o objeto (mão envolve objeto) — filtrar
            if wp_name == 'pick':
                collisions = [c for c in collisions_raw
                              if not (c['link'] == 'Link6'
                                      and c['obstacle'] == f'pick_{obj_name}')]
            else:
                collisions = collisions_raw

            # Imprime AABB de cada link
            for lname, (cx,cy,cz,sx,sy,sz) in link_aabbs.items():
                mn = link_bounds[lname]
                print(f"    {lname:10s}: world z=[{mn[4]:.3f},{mn[5]:.3f}] "
                      f"x=[{mn[0]:.3f},{mn[1]:.3f}] y=[{mn[2]:.3f},{mn[3]:.3f}]")

            if collisions:
                for c in collisions:
                    print(f"    *** COLISÃO: link={c['link']}  obs={c['obstacle']}  "
                          f"clearance={c['clearance_mm']:.1f}mm")
                    la = c['link_aabb']
                    print(f"         link AABB: cx={la[0]:.3f} cy={la[1]:.3f} cz={la[2]:.3f} "
                          f"sx={la[3]:.3f} sy={la[4]:.3f} sz={la[5]:.3f}")
                all_collisions.extend(collisions)
            else:
                print(f"    --> Nenhuma colisão detectada.")

            # Check below belt durante pick / approach_pick
            if 'pick' in wp_name or 'approach_pick' in wp_name:
                below = check_below_belt(q, wp_name)
                if below:
                    for lname, zmax in below:
                        entry = {'obj': obj_name, 'waypoint': wp_name,
                                 'link': lname, 'zmax_world': zmax}
                        all_below_belt.append(entry)
                        print(f"    --- ABAIXO BELT: {lname} z_max_world={zmax:.4f}m "
                              f"(belt_surface_top=0.806m)")

    # ── Análise descent para todos os objetos ──────────────────────────
    print(f"\n{'='*80}")
    print("ANÁLISE DESCENT pick — clearance Link2/Link3 vs belt_guides")
    print(f"(guides: z_center=0.870, sz=0.13 → z=[0.805,0.935]m world)")
    print(f"{'='*80}")

    descent_issues = []
    for obj_name, _ in OBJECTS:
        print(f"\n  Objeto: {obj_name}")
        results = analyze_descent_clearance(obj_name)
        min_clr = {}  # (link, guide) → min clearance
        for r in results:
            key = (r['link'], r['guide'])
            if key not in min_clr or r['clearance_mm'] < min_clr[key][0]:
                min_clr[key] = (r['clearance_mm'], r['step'], r['tcp_z_world'])

        for (link, guide), (clr, step, tcp_z) in sorted(min_clr.items()):
            status = "COLISÃO" if clr < 0 else "OK"
            print(f"    {link} vs {guide}: min_clearance={clr:.1f}mm  "
                  f"[step={step}, tcp_z_world={tcp_z:.4f}]  {status}")
            if clr < 0:
                descent_issues.append({'obj': obj_name, 'link': link,
                                       'guide': guide, 'clearance_mm': clr,
                                       'tcp_z': tcp_z})

    # Detalhe passo-a-passo para frasco (objeto de referência)
    print(f"\n  Detalhamento passo-a-passo para 'frasco':")
    results_frasco = analyze_descent_clearance('frasco')
    print(f"  {'step':>4} {'tcp_z':>7} {'L2 vs L':>10} {'L2 vs R':>10} {'L3 vs L':>10} {'L3 vs R':>10}")
    by_step = {}
    for r in results_frasco:
        s = r['step']
        k = (r['link'], r['guide'])
        if s not in by_step:
            by_step[s] = {'tcp_z': r['tcp_z_world']}
        by_step[s][k] = r['clearance_mm']
    for step in sorted(by_step.keys()):
        d = by_step[step]
        tcp_z = d['tcp_z']
        l2l = d.get(('Link2','belt_guide_L'), 9999)
        l2r = d.get(('Link2','belt_guide_R'), 9999)
        l3l = d.get(('Link3','belt_guide_L'), 9999)
        l3r = d.get(('Link3','belt_guide_R'), 9999)
        print(f"  {step:>4}  {tcp_z:.4f}  {l2l:>+10.1f}  {l2r:>+10.1f}  {l3l:>+10.1f}  {l3r:>+10.1f}")

    # ── Resumo final ────────────────────────────────────────────────────
    print(f"\n{'='*80}")
    print("RESUMO DE PROBLEMAS ENCONTRADOS")
    print(f"{'='*80}")

    if all_collisions:
        print(f"\n[1] COLISÕES AABB DETECTADAS ({len(all_collisions)} total):")
        # Agrupa por combinação única
        seen = set()
        for c in all_collisions:
            key = (c['waypoint'], c['link'], c['obstacle'])
            if key not in seen:
                seen.add(key)
                print(f"   • {c['waypoint']}: link={c['link']}  obs={c['obstacle']}  "
                      f"clr={c['clearance_mm']:.1f}mm")
    else:
        print("\n[1] Nenhuma colisão AABB detectada.")

    if all_below_belt:
        print(f"\n[2] LINKS ABAIXO DA SUPERFÍCIE DA ESTEIRA durante pick/approach_pick:")
        for e in all_below_belt:
            print(f"   • {e['obj']}/{e['waypoint']}: {e['link']} z_max={e['zmax_world']:.4f}m")
    else:
        print("\n[2] Nenhum link abaixo da superfície da esteira.")

    if descent_issues:
        print(f"\n[3] COLISÕES NA DESCIDA (descent):")
        for d in descent_issues:
            print(f"   • {d['obj']}: {d['link']} vs {d['guide']} clearance={d['clearance_mm']:.1f}mm")
    else:
        print("\n[3] Nenhuma colisão durante descida detectada.")

    # ── Recomendações ────────────────────────────────────────────────────
    print(f"\n{'='*80}")
    print("RECOMENDAÇÕES DE AJUSTE DE PARÂMETROS")
    print(f"{'='*80}")

    # Avaliar via_box: Link2/Link3 vs belt_guides/belt_surface
    print("\nVerificando via_box especificamente...")
    for obj_name, box_key in OBJECTS:
        box_w = BOXES[box_key]
        ap_box_w = np.array([box_w[0], box_w[1], box_w[2] + AP_CLEAR])
        q_ab_chk, _ = ik_robot(ap_box_w, seed=_APPROACH_BOX_SEED_Q, elbow_up=False)
        via_world = np.array([box_w[0], box_w[1], BASE_Z + TRANSIT_Z])
        q_via, ok_via = ik_robot(via_world, seed=_VIA_BOX_SEED_Q, elbow_up=False)
        tcp_w = fk_tcp_world(q_via)
        _, link_aabbs_via = compute_link_aabbs(q_via)  # bounds
        link_aabbs_d, link_bounds_via = compute_link_aabbs(q_via)
        print(f"\n  via_box ({obj_name}): TCP_world_z={tcp_w[2]:.4f}m, IK_ok={ok_via}")
        for lname in ['Link2','Link3']:
            mn = link_bounds_via[lname]
            print(f"    {lname}: world z=[{mn[4]:.3f},{mn[5]:.3f}]")
        for guide in ['belt_guide_L','belt_guide_R','belt_surface','sort_shelf']:
            for lname in ['Link2','Link3','Link4','Link5','Link6']:
                coll, clr = check_collision(link_aabbs_d[lname], OBS[guide], margin=0.0)
                if coll or clr < 50:
                    status = "COLISÃO" if coll else "perto"
                    print(f"    {lname} vs {guide}: clr={clr:.1f}mm  {status}")

    # Check HOME
    print(f"\n  HOME check:")
    home_coll, home_aabbs, home_bounds = check_all_collisions(_HOME_Q, "HOME")
    for lname in ['Link2','Link3']:
        mn = home_bounds[lname]
        print(f"    {lname}: world z=[{mn[4]:.3f},{mn[5]:.3f}]")

    # ── Verificação de cantos STL: colisões que não são belt_frame ───────
    print(f"\n{'='*80}")
    print("VERIFICAÇÃO EXATA DE CANTOS STL (exclui belt_frame: visual-only)")
    print(f"{'='*80}")
    skip_obs_corner = {'belt_frame'}
    seen_corner = set()
    real_corner_collisions = []
    for c in all_collisions:
        obs = c['obstacle']
        if obs in skip_obs_corner:
            continue
        wp_key = c['waypoint']
        link   = c['link']
        key    = (wp_key, link, obs)
        if key in seen_corner:
            continue
        seen_corner.add(key)
        q_c = waypoint_qs.get(wp_key)
        if q_c is None or link == 'base_link':
            continue
        any_in, n_in, corners_w = corners_in_obs(q_c, link, obs)
        z_min_corner = corners_w[2].min()
        tag = f"  REAL ({n_in}/8 cantos dentro)" if any_in else f"  FALSO POSITIVO (0/8 cantos; z_min_canto={z_min_corner:.3f}m)"
        print(f"  {wp_key}: {link} vs {obs}  AABB_clr={c['clearance_mm']:.1f}mm{tag}")
        if any_in:
            real_corner_collisions.append({**c, 'n_corners_inside': n_in})

    print(f"\n  Colisões reais confirmadas pelos cantos: {len(real_corner_collisions)}")
    for r in real_corner_collisions:
        print(f"    >>> {r['waypoint']}: {r['link']} vs {r['obstacle']}  "
              f"AABB_clr={r['clearance_mm']:.1f}mm  cantos_dentro={r['n_corners_inside']}/8")

    print(f"\n{'='*80}")
    print("FIM DA ANÁLISE")
    print(f"{'='*80}")
    print(f"\nParâmetros atuais usados:")
    print(f"  BASE_Z       = {BASE_Z} m")
    print(f"  AP_CLEAR     = {AP_CLEAR} m")
    print(f"  LIFT_HEIGHT  = {LIFT_H} m")
    print(f"  TRANSIT_Z    = {TRANSIT_Z:.4f} m robot frame ({BASE_Z+TRANSIT_Z:.4f} world)")
    print(f"  HOME_Q (deg) = {np.rad2deg(_HOME_Q).round(1)}")
    print(f"  PICK_XY      = {PICK_XY}")
    print(f"  PICK_Z       = {PICK_Z}")
    print(f"  BOXES world  = {BOXES}")


if __name__ == '__main__':
    main()
