"""
poses.py — fonte única de poses, grips e fases para a célula CR10+COVVI.

Conforme `docs/SDD_grasp_perfection.md` v1.0.0 §4–§7. Reutilizado por
`manual_control_node.py` (GUI step-by-step) e por `grasp_executor.py`
(ciclo autônomo). Não depende de ROS — apenas de `math` e `numpy` (via
`kinematics`).

Para qualquer mudança em pose/grasp, atualize antes o SDD e referencie
a seção `§4` no commit.
"""

from __future__ import annotations

import math


# ── Conjunto de juntas ────────────────────────────────────────────────
ARM_JOINTS = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6')
HAND_JOINTS = ('Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Rotate')

ARM_LIMITS_DEG = {
    'joint1': (-180, 180), 'joint2': (-260,  80),
    'joint3': (-135, 135), 'joint4': (-260,  80),
    'joint5': (-135, 135), 'joint6': (-360, 360),
}

MAX_RAD = {'Thumb': 1.6, 'Index': 1.6, 'Middle': 1.6,
           'Ring':  1.6, 'Little': 1.6, 'Rotate': 1.0}


# ── Presets de braço (poses-base) ─────────────────────────────────────
ARM_PRESETS_BASE = {
    'Home':      {'joint1':   0, 'joint2':   0, 'joint3':  90,
                  'joint4': -90, 'joint5': -90, 'joint6':   0},
    'Vertical':  {'joint1':   0, 'joint2':   0, 'joint3':   0,
                  'joint4': -90, 'joint5': -90, 'joint6':   0},
    'Estendido': {'joint1':   0, 'joint2':   0, 'joint3': -90,
                  'joint4':   0, 'joint5':   0, 'joint6':   0},
}


# ── Orientações canônicas de TCP (R_tcp 3x3, world frame) ─────────────
# Convenção da cadeia: TCP_z = direção dos dedos (hand_y), TCP_y = -palm
# front (palma-frente = −TCP_y), TCP_x = largura da palma (hand_x).
#
#   • Palm-Down: palma normal aponta para −Z mundo (palma encara o
#     objeto por cima). Dedos extendem horizontalmente em +Y (para
#     trás, longe do operador). É o que o frasco e a ampola precisam:
#     a palma sobrepõe o topo do objeto e os dedos curvam em volta.
#
#   • Lateral-Claw: palma normal aponta para +Y mundo (palma encara o
#     tubo a partir do lado do operador −Y). Dedos extendem para −Z
#     (apontados para baixo), de modo que quando fecham eles cruzam
#     o eixo do cilindro vertical e o abraçam por um lado, com o
#     polegar pelo outro. É o que o tubo precisa (ver images/claw.png).
def _Rtcp_palm_down(finger_dir: tuple = (0.0, 1.0, 0.0)):
    """R_tcp com palma normal = −Z mundo e dedos em `finger_dir`.
    Retorna numpy.array 3x3.
    """
    import numpy as np
    z = np.asarray(finger_dir, float); z /= np.linalg.norm(z) + 1e-12
    # palm-front = -TCP_y → TCP_y = -palm_dir; palm_dir = (0,0,-1)
    y = np.array([0.0, 0.0, 1.0])
    # Ortogonaliza y em relação a z
    y = y - z * float(np.dot(y, z)); y /= np.linalg.norm(y) + 1e-12
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


def _Rtcp_lateral_front(finger_dir: tuple = (0.0, 1.0, 0.0)):
    """R_tcp para pegar objeto pela LATERAL FRONTAL (lado do robô).

    Palm normal = +X mundo (palma encara o cilindro a partir do
    pedestal do robô — o frasco fica à frente, palma vertical,
    rosto da palma virado para o frasco). Dedos extendem em
    `finger_dir` (default +Y mundo, lateral à direita do operador).

    É equivalente a pegar uma lata de refrigerante pela lateral:
    palma plana contra a face traseira da lata (do ponto de vista
    do operador), 4 dedos extendem lateralmente e enrolam para o
    +X envolvendo a outra metade do cilindro, polegar oposto pela
    -Y completa o aperto.

    Vantagem vs. top-down palm-on-top:
      • palma SOBREPÕE a lateral do cilindro com 4cm de contato
        de área (não só os 1-2cm do topo);
      • a "pele" injetada nas falanges abraça toda a superfície
        vertical do frasco, friccionando contra a maior parte da
        área cilíndrica;
      • a descida do braço continua sendo top-down em Z (sem
        flip de ramo IK), mas a postura FINAL é lateral.
    """
    import numpy as np
    z = np.asarray(finger_dir, float); z /= np.linalg.norm(z) + 1e-12
    # palm_normal = +X mundo → TCP_y = -palm_front = -palm_normal = -X
    y = np.array([-1.0, 0.0, 0.0])
    y = y - z * float(np.dot(y, z)); y /= np.linalg.norm(y) + 1e-12
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


def _Rtcp_lateral_claw(palm_dir: tuple = (0.0, 1.0, 0.0),
                       finger_dir: tuple = (0.0, 0.0, -1.0)):
    """R_tcp com palma normal = `palm_dir` (horizontal) e dedos em
    `finger_dir` (vertical para baixo). Para grasp lateral em garra
    sobre cilindro vertical (tubo)."""
    import numpy as np
    z = np.asarray(finger_dir, float); z /= np.linalg.norm(z) + 1e-12
    y = -np.asarray(palm_dir, float)
    y = y - z * float(np.dot(y, z))
    y /= np.linalg.norm(y) + 1e-12
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


def _best_palm_down_for_target(palm_world_pick,
                                 palm_world_pre=None,
                                 palm_world_appr=None):
    """Busca θ (rotação horizontal de finger_dir) que minimiza |q1| E
    é collision-free em TODAS as 3 fases (pre, approach, pick).

    Para palm-down: TCP = palm + 0.070·finger_dir (palm-grip).
    """
    try:
        import numpy as np
        from .kinematics import (
            _ik_candidates, _ik_refine, forward_kinematics, _rot_error,
        )
        from .collision import pose_is_safe
    except Exception:
        return None, None

    palm_pick = np.array(palm_world_pick, float)
    palm_pre  = np.array(palm_world_pre  if palm_world_pre  else palm_pick, float)
    palm_appr = np.array(palm_world_appr if palm_world_appr else palm_pick, float)

    def _solve(tcp_world, R, seed_q):
        p_base = tcp_world.copy(); p_base[2] -= 0.405
        best_local = None
        for cand in _ik_candidates(p_base, R, q_seed=seed_q, elbow_up=True):
            q, _ = _ik_refine(p_base, R, cand)
            T = forward_kinematics(q)
            err_p = float(np.linalg.norm(T[:3, 3] - p_base))
            err_o = float(np.linalg.norm(_rot_error(T[:3, :3], R)))
            if err_p > 0.008 or err_o > 0.15:
                continue
            q2 = math.degrees(q[1])
            if q2 < -100.0 or q2 > 60.0:
                continue
            safe, _ = pose_is_safe(q, margin_arm=0.005, margin_wrist=0.005)
            if not safe:
                continue
            score = abs(math.degrees(q[0])) + 50 * err_p
            if best_local is None or score < best_local[0]:
                best_local = (score, q)
        return best_local

    best = None
    for deg in range(0, 360, 15):
        th = math.radians(deg)
        finger_dir = np.array([math.cos(th), math.sin(th), 0.0])
        R = _Rtcp_palm_down(finger_dir=tuple(finger_dir))

        # Resolve pick primeiro (alvo mais crítico)
        seed = np.array([0.0, math.radians(-20), math.radians(-90),
                         0.0, math.radians(-90), 0.0])
        tcp_pick = palm_pick + 0.070 * finger_dir
        pick_sol = _solve(tcp_pick, R, seed)
        if pick_sol is None:
            continue
        # Resolve appr e pre usando pick como semente para cascata
        seed_q = pick_sol[1]
        tcp_appr = palm_appr + 0.070 * finger_dir
        appr_sol = _solve(tcp_appr, R, seed_q)
        if appr_sol is None:
            continue
        tcp_pre = palm_pre + 0.070 * finger_dir
        pre_sol  = _solve(tcp_pre, R, appr_sol[1])
        if pre_sol is None:
            continue

        # Score: soma dos |j1| das 3 fases + soma dos err_p
        total_j1 = sum(abs(math.degrees(s[1][0]))
                       for s in (pre_sol, appr_sol, pick_sol))
        if best is None or total_j1 < best[0]:
            best = (total_j1, R, finger_dir, deg)

    if best is None:
        return None, None
    return best[1], tuple(float(v) for v in best[2])


# ── CONVENÇÃO TCP POR TIPO DE GRIP (CRÍTICO) ──────────────────────────
#
# T_HAND_ATTACH define TCP = ponto de convergência dos fingertips =
# hand_base_link + 0.115·finger_dir.
#
# A PALMA FÍSICA (corpo da mão entre o flange e os MCPs) está deslocada
# de hand_base_link em +hand_y por ~45mm — meio caminho até os MCPs
# (91mm). Portanto a palma FICA ENTRE o flange (joint6) e o TCP:
#
#     joint6/hand_base  ──45mm──►  palm_center  ──70mm──►  TCP (fingertips)
#                       └────────  115mm  total ─────────┘
#
# Cada tipo de grip exige um alvo diferente para a IK:
#
#   PALM grip (frasco): a PALMA é a área de contato. O target é a
#     posição que queremos para a PALMA; convertemos para TCP via
#     TCP = palm + 0.070·finger_dir (palm está 70mm atrás do TCP).
#     Antes usávamos 0.115 → resultado: hand_base/joint6 ficava sobre
#     o objeto e a palma ~5cm de lado, o que fazia o objeto ficar
#     visualmente "abaixo da joint6 em vez da palma".
#
#   FINGERTIP grip (ampola): os fingertips são a área de contato. O
#     target IK é exatamente a posição do objeto, pois é ali que a
#     pinça polegar+indicador deve convergir.
#
#   CLAW grip (tubo): os fingertips abraçam o objeto pela lateral. O
#     target IK é a posição do objeto (fingertips convergem ali); a
#     palma fica 115mm atrás na direção dos dedos (= direção do approach).
#
# Por isso _OPTIMAL_FINGER_DIR=+Y (que minimizava j1) estava errado para
# o frasco: dedos extendiam fora do cilindro. Voltamos para +X.
#
# v2.0 (refactor 2026-05-18): TODOS os três objetos agora usam TOP-DOWN
# palm-down approach. Motivo: a versão lateral do frasco produzia ramo
# IK inconsistente entre PRE_APPROACH e PICK (joint4 flipava +130° ↔ -75°),
# fazendo o punho varrer ~200° durante a interpolação articular e mergulhar
# o conjunto palma+dedos dentro do belt_surface no caminho. Top-down com
# descida puramente em Z mantém ramo IK consistente (Z monotônico) e é
# o padrão industrial para objetos sobre conveyor horizontal.
_FINGER_DIR_FRASCO = (1.0, 0.0, 0.0)   # top-down palm grip, dedos em +X
_FINGER_DIR_AMPOLA = (1.0, 0.0, 0.0)   # fingertip pinch top-down

# Offset físico palma→fingertip convergence (m). hand_base→TCP = 0.115
# inclui ~45mm de palma + ~70mm de fingertip extension; o centro útil
# da palma está em hand_base + 0.045·hand_y, então palm→TCP = 0.070.
_PALM_TO_TCP_OFFSET: float = 0.070

# Tipo de cada grip (informativo + usado em compute_targets)
GRIP_TYPE = {
    'frasco': 'palm',       # palma toca o objeto
    'ampola': 'fingertip',  # fingertips (pinça) tocam o objeto
    'tubo':   'claw',       # fingertips (claw) tocam o objeto
}


def _build_rtcp_by_obj():
    """R_tcp por objeto. Lateral para frasco/tubo (wrap horizontal em
    torno do cilindro vertical); top-down pinch para ampola."""
    try:
        import numpy as np
        from .kinematics import approach_to_Rtcp
    except Exception:
        return {}
    return {
        # Frasco — TOP-DOWN palm grip: palma horizontal sobre o frasco,
        # dedos extendem em +X e curvam para baixo abraçando a parte
        # superior do cilindro. Padrão industrial para preensão de
        # objetos cilíndricos sobre conveyor (Robotiq/Schunk/MoveIt).
        # (Versão lateral-front foi revertida — fazia o polegar bater
        # na esteira durante a aproximação.)
        'frasco': _Rtcp_palm_down(finger_dir=_FINGER_DIR_FRASCO),
        'ampola': _Rtcp_palm_down(finger_dir=_FINGER_DIR_AMPOLA),
        # Tubo — mantido lateral (cilindro vertical alto 120mm — top-down
        # exigiria descida muito grande). Lateral claw aborda em +Y.
        'tubo':   approach_to_Rtcp(np.array([0.0, 1.0, 0.0])),
        '_finger_dir': {
            'frasco': _FINGER_DIR_FRASCO,
            'ampola': _FINGER_DIR_AMPOLA,
        },
    }


def recompute_optimal_finger_dir():
    """Re-otimiza finger_dir via `_best_palm_down_for_target` e
    imprime o resultado (para colar em _OPTIMAL_FINGER_DIR_*).
    Demora ~10s — não rodar em import time.

    Uso (off-line):
        from grasp_ml_pack.poses import recompute_optimal_finger_dir
        recompute_optimal_finger_dir()
    """
    for obj in ('frasco', 'ampola'):
        R, fdir = _best_palm_down_for_target(
            palm_world_pick=(0.75, 0.0, 0.92),
            palm_world_appr=(0.75, 0.0, 0.95),
            palm_world_pre=(0.75, 0.0, 1.05))
        print(f'  {obj}: finger_dir = {fdir}')
    print('Cole esses valores em _OPTIMAL_FINGER_DIR_FRASCO/AMPOLA.')


try:
    RTCP_BY_OBJ = _build_rtcp_by_obj()
except Exception:
    RTCP_BY_OBJ = {}


# Legado: vetores de approach (motion direction, NÃO orientação de TCP)
# Mantido para compatibilidade com qualquer chamador que ainda use
# `solve_pose(p, q_seed, approach_vec)` com a convenção antiga
# (`approach_to_Rtcp`). Para alvos de pick/approach refinados, prefira
# `solve_pose_R` com `RTCP_BY_OBJ[obj]`.
APPROACH_VEC_BY_OBJ: dict[str, tuple] = {
    'frasco': (0.0, 0.0, -1.0),
    'ampola': (0.0, 0.0, -1.0),
    'tubo':   (0.0, 1.0,  0.0),
}

# ── Alvos de TCP (world frame, metros) ───────────────────────────────
# Frasco: bbox cz=0.851 sz=0.090 → topo z=0.896. Palma encosta no topo
# com pequeno offset (palm thickness 15 mm → TCP 5 mm acima do topo).
# Ampola: bbox cz=0.844 sz=0.075 → topo z=0.881. Pinça polegar+indicador
# tocam o topo da ampola (TCP no topo).
# Tubo: bbox cz=0.866 sz=0.120 (vertical) → centro z=0.866. TCP no centro
# do cilindro com mão lateral; descida lateral em +Y traz a palma até y=0.
# Convenção crucial: TCP é a posição de CONVERGÊNCIA DOS FINGERTIPS,
# 115 mm em finger-direction a partir do wrist (hand_base_link).
# Para PALM-DOWN com finger_dir=+X, a palma fica em (TCP_x − 0.115, ...).
# Logo para a palma sobrepor o frasco em (0.75, 0, z), o TCP deve ficar
# em (0.75 + 0.115, 0, z) = (0.865, 0, z). Caso contrário, só a ponta
# dos dedos toca o objeto.
# ── Limites verticais por objeto (proteção: a palma/fingertips nunca
# deve passar do topo do objeto + margem de segurança). Calculados a
# partir de PICK_OBJ_BBOX (collision.py). Para palm-grip o limite é
# `obj_top + PALM_CLEARANCE`; para fingertip pinch o limite é o próprio
# topo do objeto (onde a pinça TOCA).
_OBJ_TOP_Z: dict[str, float] = {
    'frasco': 0.851 + 0.090 / 2,  # 0.896
    'ampola': 0.844 + 0.075 / 2,  # 0.881
    'tubo':   0.866 + 0.120 / 2,  # 0.926 (lateral grasp — não é descida)
}
_PALM_CLEARANCE: float = 0.020   # 20 mm — palma 2 cm acima do topo
_PINCH_CLEARANCE: float = 0.000  # 0 mm — pinça TOCA o topo (objetivo)

# Alvos da PALMA / TCP por objeto. Para palm-grip (frasco) usamos a
# posição da PALMA física (não do flange/joint6 nem do TCP). Para
# fingertip/claw usamos o TCP. Os módulos abaixo convertem para TCP.
_FRASCO_PALM_Z_DESIRED: float = max(0.920,
                                     _OBJ_TOP_Z['frasco'] + _PALM_CLEARANCE)

PRE_APPROACH_TCP_WORLD: dict[str, tuple] = {
    'frasco': (0.0, 0.0, 0.0),  # placeholder — preenchido em _refresh_*
    'ampola': (0.0, 0.0, 0.0),
    'tubo':   (0.75, -0.185, 0.866),
}

APPROACH_TCP_WORLD_BY_OBJ: dict[str, tuple] = {
    'frasco': (0.0, 0.0, 0.0),
    'ampola': (0.0, 0.0, 0.0),
    'tubo':   (0.75, -0.065, 0.866),
}

PICK_TCP_WORLD: dict[str, tuple] = {
    'frasco': (0.0, 0.0, 0.0),
    'ampola': (0.0, 0.0, 0.0),
    'tubo':   (0.75, 0.000, 0.866),
}


def _refresh_tcp_from_grip_type():
    """Recalcula TCP por objeto respeitando o TIPO DE GRIP:

      • palm grip   → palma sobre o objeto; TCP = palm + 0.070·fdir
                       (a palma física fica ~45mm à frente do flange,
                       70mm atrás do TCP)
      • fingertip   → TCP target at object (fingertips convergem ali)
      • claw        → TCP target at object (idem)

    Para palm grip, palm_z respeita _OBJ_TOP_Z + _PALM_CLEARANCE para a
    palma nunca passar do topo do objeto (limite inferior obrigatório).
    Para fingertip, TCP_z respeita _OBJ_TOP_Z + _PINCH_CLEARANCE.
    """
    fdirs = RTCP_BY_OBJ.get('_finger_dir', {}) if isinstance(
        RTCP_BY_OBJ, dict) else {}

    # ── TOP-DOWN PALM GRIP: frasco — palma horizontal sobre o
    # cilindro, dedos em +X curvam para baixo abraçando a parte
    # superior. Geometria (finger_dir=+X):
    #   palm_center = TCP − 0.070·fdir   → palm em x = TCP_x − 0.070
    #   MCPs (juntas proximais dos 4 dedos) = palm + 0.046·fdir
    #                                       → x = TCP_x − 0.024
    # Para os MCPs ficarem alinhados com o centro do frasco (x=0.75),
    # TCP_x = 0.75 + 0.024 ≈ 0.774. Palm_x = 0.704 (à -X do frasco),
    # dedos extendem +X cruzando sobre o frasco e curvam para baixo
    # envolvendo a parte superior (z=0.851±0.045).
    # Z do PICK: palm a 25mm acima do topo do frasco (top=0.896 →
    # palm_z=0.921). TCP_z = palm_z (finger_dir é horizontal, sem
    # componente Z). APPROACH a +6cm; PRE_APPROACH a +18cm.
    PICK_TCP_WORLD['frasco']            = (0.774, 0.000, 0.921)
    APPROACH_TCP_WORLD_BY_OBJ['frasco'] = (0.774, 0.000, 0.981)
    PRE_APPROACH_TCP_WORLD['frasco']    = (0.774, 0.000, 1.101)

    # ── FINGERTIP GRIP: ampola — TCP AT ampola top (clampado) ──────
    # PICK_z = max(topo, parâmetro) → garante que a ponta nunca passa
    # do topo do objeto. Topo ampola = 0.881.
    ampola_top = _OBJ_TOP_Z['ampola']
    pick_z_ampola = max(0.881, ampola_top + _PINCH_CLEARANCE)
    PRE_APPROACH_TCP_WORLD['ampola']    = (0.75, 0.0, pick_z_ampola + 0.119)
    APPROACH_TCP_WORLD_BY_OBJ['ampola'] = (0.75, 0.0, pick_z_ampola + 0.049)
    PICK_TCP_WORLD['ampola']            = (0.75, 0.0, pick_z_ampola)

    # ── CLAW GRIP: tubo — TCP AT tubo center (0.75, 0, 0.866) ──────
    # finger_dir=+Y (lateral), palma em -Y do tubo.
    PRE_APPROACH_TCP_WORLD['tubo']    = (0.75, -0.185, 0.866)
    APPROACH_TCP_WORLD_BY_OBJ['tubo'] = (0.75, -0.065, 0.866)
    PICK_TCP_WORLD['tubo']            = (0.75,  0.000, 0.866)


_refresh_tcp_from_grip_type()

# Caixas de entrega
DELIVERY_XY_WORLD = {
    'frasco': (-0.05, 0.65),
    'tubo':   ( 0.25, 0.65),
    'ampola': ( 0.55, 0.65),
}
DELIVERY_Z_WORLD = 0.75


# ── Fallbacks (graus) — usados se o IK numérico não estiver disponível ─
_FALLBACK_TOP_DOWN_SEED = {
    'joint1':   0.0, 'joint2': -30.0, 'joint3': -90.0,
    'joint4':   0.0, 'joint5': -90.0, 'joint6':   0.0,
}
_FALLBACK_LATERAL_SEED = {
    'joint1':  +2.5, 'joint2': -21.0, 'joint3':  -84.9,
    'joint4':  -74.1, 'joint5': -177.5, 'joint6': +90.0,
}
FALLBACK_PRE_APPROACH_DEG: dict[str, dict] = {
    'frasco': dict(_FALLBACK_TOP_DOWN_SEED),
    'ampola': dict(_FALLBACK_TOP_DOWN_SEED),
    'tubo':   dict(_FALLBACK_LATERAL_SEED),
}
FALLBACK_APPROACH_DEG_BY_OBJ: dict[str, dict] = {
    'frasco': dict(_FALLBACK_TOP_DOWN_SEED),
    'ampola': dict(_FALLBACK_TOP_DOWN_SEED),
    'tubo':   dict(_FALLBACK_LATERAL_SEED),
}
FALLBACK_PICK_DEG: dict[str, dict] = {
    'frasco': dict(_FALLBACK_TOP_DOWN_SEED),
    'ampola': dict(_FALLBACK_TOP_DOWN_SEED),
    'tubo':   dict(_FALLBACK_LATERAL_SEED),
}
FALLBACK_DELIVERY_DEG = {
    'frasco': {'joint1': +108.4, 'joint2': -19.6, 'joint3': -91.3,
               'joint4':  -69.1, 'joint5': +108.4, 'joint6':  +0.0},
    'tubo':   {'joint1':  +85.6, 'joint2': -23.2, 'joint3': -86.3,
               'joint4':  -70.5, 'joint5':  +85.6, 'joint6':  +0.0},
    'ampola': {'joint1':  +65.8, 'joint2': -36.5, 'joint3': -65.6,
               'joint4':  -77.9, 'joint5':  +65.8, 'joint6':  +0.0},
}


# ── Configurações de preensão (SDD §4) ────────────────────────────────
HAND_OPEN = {j: 0 for j in HAND_JOINTS}

# HAND_GRIPS derivados de kinematics.HAND_CONFIGS (fonte de verdade do
# projeto). Conversão: slider = rad / MAX_RAD * 200, com MAX_RAD=1.6
# (dedos) e 1.0 (Rotate). Cada grip tem assinatura geométrica distinta:
#
#   palm_grip      — envoltura palmar — TODOS os dedos altos (138-156)
#                    + Rotate baixo (50) — polegar no lado oposto.
#   claw_grip      — preensão em garra — dedos médios (113-138) +
#                    Rotate médio (90) — semi-fechamento conformante.
#   fingertip_grip — pinça precisa — APENAS polegar+indicador (~100) +
#                    Rotate alto (164) — médio/anelar/mínimo NÃO usados.
HAND_GRIPS = {
    'Open': HAND_OPEN,
    # Slider 200 = curl máximo (1.6 rad). 138 / 156 / 138 (anteriores)
    # deixavam fingertips ~57mm do eixo do cilindro — fora do frasco
    # (r=45mm). Subimos a 185-195 para envolverem a geometria.
    'Palm Grip (frasco)':      {'Thumb': 185, 'Index': 195, 'Middle': 195,
                                 'Ring':  190, 'Little': 185, 'Rotate':  50},
    'Claw Grip (tubo)':        {'Thumb': 175, 'Index': 190, 'Middle': 190,
                                 'Ring':  185, 'Little': 178, 'Rotate':  90},
    'Fingertip Grip (ampola)': {'Thumb': 145, 'Index': 140, 'Middle':   0,
                                 'Ring':    0, 'Little':   0, 'Rotate': 164},
}

# Pré-shape = aprox. 50% de cada grip final. Mantém a assinatura
# distintiva (palm/claw/fingertip) mas com dedos pré-curvados o
# suficiente para não varrer o objeto durante a descida.
HAND_PRESHAPE = {
    'Palm Grip (frasco)':      {'Thumb':  70, 'Index':  80, 'Middle':  80,
                                 'Ring':   75, 'Little':  70, 'Rotate':  25},
    'Claw Grip (tubo)':        {'Thumb':  60, 'Index':  70, 'Middle':  70,
                                 'Ring':   65, 'Little':  60, 'Rotate':  45},
    'Fingertip Grip (ampola)': {'Thumb':  55, 'Index':  50, 'Middle':   0,
                                 'Ring':    0, 'Little':   0, 'Rotate':  82},
}

# ── Poses extras (gestos / formas úteis) ──────────────────────────────
# Valores 0–200 por dedo (0=aberto/relaxado, 200=curl máximo); Rotate
# segue a mesma escala (0=polegar fora, 200=polegar atravessando a palma).
# Usadas tanto no Gazebo quanto na mão COVVI real (SetDigitPosn aceita
# os mesmos 0..200 por dedo + Speed).
HAND_EXTRA_POSES = {
    'Punho':         {'Thumb': 200, 'Index': 200, 'Middle': 200,
                      'Ring':  200, 'Little': 200, 'Rotate':  60},
    'Apontar':       {'Thumb': 170, 'Index':   0, 'Middle': 195,
                      'Ring':  195, 'Little': 185, 'Rotate':  55},
    'Paz (V)':       {'Thumb': 180, 'Index':   0, 'Middle':   0,
                      'Ring':  195, 'Little': 195, 'Rotate':  60},
    'Joinha':        {'Thumb':   0, 'Index': 195, 'Middle': 195,
                      'Ring':  195, 'Little': 195, 'Rotate':   0},
    'Rock':          {'Thumb':  40, 'Index':   0, 'Middle': 200,
                      'Ring':  200, 'Little':   0, 'Rotate':  30},
    'Pistola':       {'Thumb':   0, 'Index':   0, 'Middle': 195,
                      'Ring':  195, 'Little': 185, 'Rotate':  95},
    'Gancho':        {'Thumb':   0, 'Index': 140, 'Middle': 140,
                      'Ring':  140, 'Little': 130, 'Rotate':   0},
    'Concha':        {'Thumb': 110, 'Index': 120, 'Middle': 120,
                      'Ring':  115, 'Little': 110, 'Rotate':  80},
    'Garra Aberta':  {'Thumb':  70, 'Index':  80, 'Middle':  80,
                      'Ring':   75, 'Little':  70, 'Rotate':  90},
    'Contar 3':      {'Thumb': 180, 'Index':   0, 'Middle':   0,
                      'Ring':    0, 'Little': 195, 'Rotate':  50},
}


OBJ_GRIP = {
    'frasco': 'Palm Grip (frasco)',
    'tubo':   'Claw Grip (tubo)',
    'ampola': 'Fingertip Grip (ampola)',
}

OBJ_APPROACH_DESC = {
    'frasco': 'Top-down — palma horizontal envolve o topo do frasco',
    'ampola': 'Top-down — pinça polegar+indicador sobre a ampola',
    'tubo':   'Lateral — claw frontal, mão virada (ver claw.png)',
}

# Nomes das 6 fases do ciclo de pick (SDD §6)
PHASE_NAMES = ('F1 pre-appr', 'F2 approach', 'F3 preshape',
               'F4 descend',  'F5 close',    'F6 lift')


# ── IK runtime ────────────────────────────────────────────────────────
def solve_pose(p_world, q_seed_deg, approach_vec=(0.0, 0.0, -1.0)):
    """Resolve IK para TCP em world frame com a abordagem dada. Retorna
    dict de graus por junta, ou None se IK não disponível / não convergir.

    Lazy import de numpy/kinematics: se algum dos dois falhar, devolve None
    (o chamador deve cair no fallback).
    """
    try:
        import numpy as np
        from .kinematics import (
            inverse_kinematics, forward_kinematics, approach_to_Rtcp,
            _ik_refine,
        )
    except Exception:
        return None

    base_z = 0.405
    p_base = np.array([p_world[0], p_world[1], p_world[2] - base_z])
    approach = np.array(approach_vec, dtype=float)
    q_seed = np.array([math.radians(q_seed_deg[j]) for j in ARM_JOINTS])

    q_global, ok = inverse_kinematics(
        p_base, approach, q_seed=q_seed, elbow_up=True)
    branch_ok = (ok
                 and abs(math.degrees(q_global[1])
                          - math.degrees(q_seed[1])) < 60.0
                 and abs(math.degrees(q_global[0])
                          - math.degrees(q_seed[0])) < 60.0)
    if branch_ok:
        q_final = q_global
    else:
        R_tcp = approach_to_Rtcp(approach)
        q_final, _ = _ik_refine(p_base, R_tcp, q_seed)

    T = forward_kinematics(q_final)
    err_mm = float(np.linalg.norm(T[:3, 3] - p_base)) * 1000.0
    if err_mm > 5.0:
        return None
    return {j: float(math.degrees(q_final[i])) for i, j in enumerate(ARM_JOINTS)}


def solve_pose_R(p_world, R_tcp, q_seed_deg, *,
                  check_collision: bool = True,
                  allow_object: str | None = None,
                  collision_margin: float = 0.005):
    """Resolve IK com R_tcp 3x3 explícito (orientação completa do TCP).

    Args:
        p_world: TCP em world frame (3,)
        R_tcp:   matriz 3x3 da orientação do TCP em world frame
        q_seed_deg: dict {jointN: graus} usado como semente
        check_collision: se True, filtra ramos IK que colidem com o
            mundo (esteira, paredes, caixas, câmera, prateleira).
        allow_object: nome do objeto picável tolerado no toque.
        collision_margin: folga (m) entre cada elo e os obstáculos.

    Retorna dict graus por junta ou None se nenhum ramo IK satisfizer
    posição + orientação + colisão.
    """
    try:
        import numpy as np
        from .kinematics import (
            _ik_candidates, _ik_refine, forward_kinematics, _rot_error,
        )
        from .collision import pose_is_safe
    except Exception:
        return None

    base_z = 0.405
    p_base = np.array([p_world[0], p_world[1], p_world[2] - base_z])
    R = np.asarray(R_tcp, float)
    q_seed = np.array([math.radians(q_seed_deg[j]) for j in ARM_JOINTS])

    # Coletamos TODOS os candidatos viáveis (geom + IK) e só depois
    # escolhemos o melhor — assim podemos rejeitar ramos colidentes.
    viable: list[tuple[float, "np.ndarray", float, float, bool]] = []
    for cand in _ik_candidates(p_base, R, q_seed=q_seed, elbow_up=True):
        q, _ok = _ik_refine(p_base, R, cand)
        T = forward_kinematics(q)
        err_p = float(np.linalg.norm(T[:3, 3] - p_base))
        err_o = float(np.linalg.norm(_rot_error(T[:3, :3], R)))
        if err_p > 0.010 or err_o > 0.25:
            continue
        # Penalidades adicionais:
        #   • seed_dev: cascata pre→appr→pick consistente (mesmo ramo)
        #   • joint2 extremo: punição para soluções tipo "elbow swings
        #     for trás" (q2 < -100° ou > 60°)
        seed_dev = float(np.linalg.norm(q - q_seed))
        q2_deg = math.degrees(q[1])
        q2_pen = 0.0
        if q2_deg < -100.0:
            q2_pen = (abs(q2_deg) - 100.0) / 30.0
        elif q2_deg > 60.0:
            q2_pen = (q2_deg - 60.0) / 30.0

        # Penalidade extra para joint1 longe de zero quando o TCP está
        # centrado em Y=0 (frasco/ampola). Evita o "drift à esquerda"
        # observado: para alvos em y_world≈0 com base do robô em y=0,
        # joint1 natural é ≈0; ramos com |joint1| > 10° são esquerda/
        # direita não-natural.
        q1_deg = math.degrees(q[0])
        q1_pen = 0.0
        # Detectar TCP centrado (target_y do chamador) — usamos p_world
        target_y = float(p_world[1])
        if abs(target_y) < 0.05:  # alvo essencialmente em Y=0
            q1_pen = max(0.0, (abs(q1_deg) - 5.0) / 15.0)

        safe = True
        if check_collision:
            safe, _ = pose_is_safe(q, margin_arm=collision_margin + 0.005,
                                     margin_wrist=collision_margin)

        score = (err_p + 0.10 * err_o
                 + 0.02 * seed_dev
                 + 0.50 * q2_pen
                 + 1.00 * q1_pen
                 + (0.0 if safe else 100.0))
        viable.append((score, q, err_p, err_o, safe))

    if not viable:
        return None

    # 1ª opção: candidato com menor score E livre de colisão.
    viable.sort(key=lambda t: t[0])
    chosen = next((t for t in viable if t[4]), None)
    if chosen is None:
        # Nenhum ramo livre. Se não exigíamos colisão, devolve o melhor;
        # se exigíamos, devolve None (chamador deve relaxar margem ou
        # mover o objetivo).
        if not check_collision:
            chosen = viable[0]
        else:
            return None

    _, q, _, _, _ = chosen
    return {j: float(math.degrees(q[i])) for i, j in enumerate(ARM_JOINTS)}


def compute_targets():
    """Resolve via IK (com fallbacks) as poses por objeto.

    Conforme SDD §6 v1.1: usamos R_tcp explícito (palm-down ou
    lateral-claw) em vez de approach_vec genérico. Cadeia de seeds:
        pre_approach → approach → grasp
    """
    pre, appr, pick = {}, {}, {}
    for obj in ('frasco', 'ampola', 'tubo'):
        R = RTCP_BY_OBJ.get(obj)
        seed_pre  = FALLBACK_PRE_APPROACH_DEG[obj]
        seed_appr = FALLBACK_APPROACH_DEG_BY_OBJ[obj]
        seed_pick = FALLBACK_PICK_DEG[obj]

        if R is not None:
            # Estratégia: resolve APPROACH primeiro (TCP mid-height,
            # ramo IK mais estável), depois usa appr[obj] como semente
            # de PRE e PICK — garante ramo consistente nas 3 fases.
            appr[obj] = (solve_pose_R(APPROACH_TCP_WORLD_BY_OBJ[obj], R,
                                        seed_appr, check_collision=True)
                         or solve_pose_R(APPROACH_TCP_WORLD_BY_OBJ[obj], R,
                                          seed_appr, check_collision=False)
                         or seed_appr)
            pre[obj] = (solve_pose_R(PRE_APPROACH_TCP_WORLD[obj], R,
                                       appr[obj], check_collision=True)
                        or solve_pose_R(PRE_APPROACH_TCP_WORLD[obj], R,
                                         appr[obj], check_collision=False)
                        or appr[obj])
            # Pick: a mão TOCA o objeto picável (frasco/ampola/tubo),
            # mas nenhum link deve tocar a esteira/caixas/parede.
            pick[obj] = (solve_pose_R(PICK_TCP_WORLD[obj], R, appr[obj],
                                        check_collision=True,
                                        allow_object=obj,
                                        collision_margin=0.003)
                         or solve_pose_R(PICK_TCP_WORLD[obj], R, appr[obj],
                                          check_collision=False)
                         or appr[obj])
        else:
            # Fallback: numpy/kinematics ausentes — usa seeds-base
            pre[obj]  = seed_pre
            appr[obj] = seed_appr
            pick[obj] = seed_pick

    delivery = {}
    for obj in ('frasco', 'tubo', 'ampola'):
        dx, dy = DELIVERY_XY_WORLD[obj]
        delivery[obj] = (solve_pose((dx, dy, DELIVERY_Z_WORLD),
                                     q_seed_deg=FALLBACK_DELIVERY_DEG[obj],
                                     approach_vec=(0.0, 0.0, -1.0))
                         or FALLBACK_DELIVERY_DEG[obj])
    return pre, appr, pick, delivery


# ── Cache hardcoded de poses (resolvido off-line por compute_targets) ─
# Para evitar 10s de IK em cada import. Re-gerar com:
#   from grasp_ml_pack.poses import recompute_and_print_poses
_CACHED_PRE_APPROACH_POSES_DEG = {
    'frasco': {'joint1':  20.1807, 'joint2':   4.8264, 'joint3':  -90.9614,
               'joint4': -93.8643, 'joint5': -69.8199, 'joint6':  -0.0019},
    'ampola': {'joint1':  21.1325, 'joint2':   7.0897, 'joint3': -103.1330,
               'joint4': -83.9564, 'joint5': -68.8684, 'joint6':  -0.0016},
    'tubo':   {'joint1': -15.6463, 'joint2': -27.1959, 'joint3':  -76.0081,
               'joint4': -76.7960, 'joint5': 164.3537, 'joint6':  90.0000},
}
_CACHED_APPROACH_POSES_DEG = {
    'frasco': {'joint1':  20.1807, 'joint2':   4.3877, 'joint3': -102.7067,
               'joint4': -81.6804, 'joint5': -69.8200, 'joint6':  -0.0018},
    'ampola': {'joint1':  21.1662, 'joint2':   5.9337, 'joint3': -109.2948,
               'joint4': -76.6381, 'joint5': -68.8343, 'joint6':  -0.0020},
    'tubo':   {'joint1':  -7.2996, 'joint2': -22.6308, 'joint3':  -82.9599,
               'joint4': -74.4093, 'joint5': 172.7004, 'joint6':  90.0000},
}
_CACHED_PICK_POSES_DEG = {
    'frasco': {'joint1':  20.2332, 'joint2':   3.2250, 'joint3': -107.8114,
               'joint4': -75.4123, 'joint5': -69.7669, 'joint6':  -0.0023},
    'ampola': {'joint1':  21.1326, 'joint2':   4.5673, 'joint3': -113.2245,
               'joint4': -71.3422, 'joint5': -68.8682, 'joint6':  -0.0019},
    'tubo':   {'joint1':  -2.4605, 'joint2': -20.3732, 'joint3':  -85.9378,
               'joint4': -73.6890, 'joint5': 177.5395, 'joint6':  90.0000},
}
_CACHED_DELIVERY_POSES_DEG = {
    'frasco': {'joint1': 111.43, 'joint2': -19.6, 'joint3': -91.3,
               'joint4':  -69.1, 'joint5': 108.4, 'joint6': 0.0},
    'tubo':   {'joint1':  85.6,  'joint2': -23.2, 'joint3': -86.3,
               'joint4': -70.5,  'joint5':  85.6, 'joint6': 0.0},
    'ampola': {'joint1':  65.8,  'joint2': -36.5, 'joint3': -65.6,
               'joint4': -77.9,  'joint5':  65.8, 'joint6': 0.0},
}


# Resolução em tempo de import — usa cache hardcoded para evitar IK
# pesada. Para recomputar, defina GRASP_RECOMPUTE_POSES=1 no ambiente.
import os as _os
if _os.environ.get('GRASP_RECOMPUTE_POSES'):
    (PRE_APPROACH_POSES_DEG, APPROACH_POSES_DEG,
     PICK_POSES_DEG, DELIVERY_POSES_DEG) = compute_targets()
else:
    PRE_APPROACH_POSES_DEG = dict(_CACHED_PRE_APPROACH_POSES_DEG)
    APPROACH_POSES_DEG     = dict(_CACHED_APPROACH_POSES_DEG)
    PICK_POSES_DEG         = dict(_CACHED_PICK_POSES_DEG)
    DELIVERY_POSES_DEG     = dict(_CACHED_DELIVERY_POSES_DEG)


def recompute_and_print_poses():
    """Roda compute_targets() e imprime no formato dos _CACHED_* dicts,
    para colar de volta neste arquivo após mudar TCPs ou R_tcp."""
    pre, appr, pick, _ = compute_targets()
    for label, src in [('_CACHED_PRE_APPROACH_POSES_DEG', pre),
                        ('_CACHED_APPROACH_POSES_DEG', appr),
                        ('_CACHED_PICK_POSES_DEG', pick)]:
        print(f'\n{label} = {{')
        for obj, d in src.items():
            items = ', '.join(f"'{k}': {round(d[k], 4)}"
                                for k in ARM_JOINTS)
            print(f"    '{obj}': {{{items}}},")
        print('}')

# Compatibilidade com código legacy que esperava um único APPROACH_POSE_DEG
APPROACH_POSE_DEG = APPROACH_POSES_DEG['frasco']
