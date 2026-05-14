"""
Controle manual da célula de manufatura — CR10 + COVVI + Esteira + Mão Real.

GUI Tkinter inspirada no Dobot CRStudio: tema claro, layout em abas,
indicadores de estado e botão de parada de emergência. Mantém toda a
lógica ROS / ECI / esteira do nó anterior; apenas a aparência mudou.

Parâmetros ROS:
  eci_prefix  (string, default '/covvi/hand')
      Prefixo do servidor ECI: /{namespace}/{server_name}

Uso:
  ros2 launch grasp_ml_pack conveyor_cell.launch.py no_gui:=true
  ros2 run   grasp_ml_pack manual_control
  ros2 run   grasp_ml_pack manual_control --ros-args -p eci_prefix:=/test/server_1
"""

from __future__ import annotations

import json
import math
import os
import queue
import signal
import subprocess
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

# Sistema de colisão — reusa AABBs calibrados do grasp_executor.
# Se o import falhar (rodando fora do workspace compilado), o sistema
# degrada para "sem colisão" silenciosamente.
try:
    import numpy as _np
    from .grasp_executor import (
        _arm_clears_world as _gx_arm_clears_world,
        _arm_clears_bbox as _gx_arm_clears_bbox,
        _bbox_overlap as _gx_bbox_overlap,
        _WORLD_OBSTACLES as _GX_OBSTACLES,
        _PICK_OBJ_BBOX as _GX_PICK_BBOX,
        _ROBOT_BASE_Z as _GX_BASE_Z,
    )
    from .kinematics import (
        fk_partial as _gx_fk_partial,
        finger_fk as _gx_finger_fk,
        forward_kinematics as _gx_forward_kinematics,
        inverse_kinematics as _gx_inverse_kinematics,
        approach_to_Rtcp as _gx_approach_to_Rtcp,
        _ik_refine as _gx_ik_refine,
        T_HAND_ATTACH as _GX_T_HAND_ATTACH,
        _K_P_FINGER as _GX_K_P_FINGER,
        _K_D_FINGER as _GX_K_D_FINGER,
        _K_P_THUMB as _GX_K_P_THUMB,
    )
    _COLLISION_OK = True
except Exception:
    _np = None
    _gx_arm_clears_world = None
    _gx_arm_clears_bbox = None
    _gx_bbox_overlap = None
    _gx_fk_partial = None
    _gx_finger_fk = None
    _gx_forward_kinematics = None
    _gx_inverse_kinematics = None
    _GX_OBSTACLES = {}
    _GX_PICK_BBOX = {}
    _GX_T_HAND_ATTACH = None
    _GX_BASE_Z = 0.405
    _GX_K_P_FINGER = 1.516
    _GX_K_D_FINGER = 0.718
    _GX_K_P_THUMB = 1.400
    _COLLISION_OK = False


# Origem do MCP de cada dedo em hand_base_link (extraído do URDF
# linear_covvi_hand_gazebo.urdf — joints `*_proximal_j_input_joint`).
# Para os 4 dedos longos, o eixo de curl é ≈ (1, 0, 0), então a flexão
# rotaciona +Y → +Z no frame da mão (a ponta vai para dentro da palma).
# Para o polegar, aproximamos pela posição estática (chassis em Rotate=0).
_FINGER_MCP_HAND: dict[str, tuple] = {
    'Index':  (+0.02310, +0.09136, -0.01476),
    'Middle': (+0.00336, +0.09438, -0.01554),
    'Ring':   (-0.01620, +0.09438, -0.01089),
    'Little': (-0.03292, +0.08433, -0.00395),
    # thumb: chassis_offset + proximal_offset (sem aplicar rotação Rotate)
    'Thumb':  (+0.07019, +0.04458, +0.02296),
}

# Conversão dos slider values (0-200) para o ângulo primário (rad).
_HAND_DRIVER_MAX = {'Thumb': 1.6, 'Index': 1.6, 'Middle': 1.6,
                    'Ring':  1.6, 'Little': 1.6, 'Rotate': 1.0}


def _finger_tip_hand(finger: str, primary_rad: float):
    """Ponta do dedo em hand_base_link, dado ângulo primário (rad).

    Usa o finger_fk planar do módulo kinematics e mapeia (x,0,z) do plano
    do dedo → (0, x, z) no frame da mão (curl em torno de +X aproximado).
    """
    if not _COLLISION_OK:
        return None
    k_p = _GX_K_P_THUMB if finger == 'Thumb' else _GX_K_P_FINGER
    k_d = _GX_K_D_FINGER
    tip_planar = _gx_finger_fk(primary_rad, k_p=k_p, k_d=k_d)
    # finger_fk devolve [x, 0, z]: x = extensão forward, z = curl
    fx, _, fz = float(tip_planar[0]), float(tip_planar[1]), float(tip_planar[2])
    mx, my, mz = _FINGER_MCP_HAND[finger]
    # +X axis de curl → +Y vira +Z; mantemos x_local sobre Y, z_local sobre Z
    return _np.array([mx, my + fx, mz + fz])


def _fingertips_world(q_arm, hand_state: dict):
    """Retorna dict {finger: pos_world (3,)} para as 5 pontas dos dedos."""
    if not _COLLISION_OK:
        return {}
    T_base = _np.eye(4)
    T_base[2, 3] = _GX_BASE_Z
    T_world_link6 = T_base @ _gx_fk_partial(q_arm, 6)
    T_world_hand = T_world_link6 @ _GX_T_HAND_ATTACH

    tips: dict[str, _np.ndarray] = {}
    for finger in ('Thumb', 'Index', 'Middle', 'Ring', 'Little'):
        primary = float(hand_state.get(finger, 0.0))
        tip_h = _finger_tip_hand(finger, primary)
        if tip_h is None:
            continue
        tip_world = T_world_hand @ _np.array([tip_h[0], tip_h[1], tip_h[2], 1.0])
        tips[finger] = tip_world[:3]
    return tips


def _point_in_bbox(p, bbox, margin: float = 0.0) -> bool:
    """Testa se um ponto cai dentro de um AABB world (cx,cy,cz,sx,sy,sz)."""
    cx, cy, cz, sx, sy, sz = bbox
    return (abs(p[0] - cx) <= sx / 2 + margin and
            abs(p[1] - cy) <= sy / 2 + margin and
            abs(p[2] - cz) <= sz / 2 + margin)


def _fingers_clear_objects(q_arm, hand_state: dict,
                            object_bbox=None,
                            check_world: bool = False,
                            margin: float = 0.005) -> tuple[bool, str]:
    """True se nenhuma ponta de dedo penetra os AABBs verificados.

    object_bbox: AABB do objeto na esteira (frasco/tubo/ampola) ou None.
    check_world: também testa contra `_WORLD_OBSTACLES` (exceto chapas
                 finas onde a mão precisa raspar para fazer o pick).
    """
    if not _COLLISION_OK:
        return True, 'OK'
    tips = _fingertips_world(q_arm, hand_state)
    for finger, tip in tips.items():
        if object_bbox is not None and _point_in_bbox(tip, object_bbox, margin):
            return False, f'{finger} penetra objeto'
        if check_world:
            for name, obs in _GX_OBSTACLES.items():
                if name in _HAND_OBSTACLE_SKIP:
                    continue
                if _point_in_bbox(tip, obs, margin):
                    return False, f'{finger} toca {name}'
    return True, 'OK'


# Envelope "Link6 + mão COVVI" no frame local do Link6.
# Após a fixação rpy=(π/2,0,0), o mapeamento é:
#   hand_x → Link6_x        (largura da palma)
#   hand_y → Link6_z        (DIREÇÃO DOS DEDOS quando abertos)
#   hand_z → −Link6_y       (espessura: −Y é palma-frente / +Y é palma-trás)
# Portanto a extensão dos dedos vai em +Link6_z, e a espessura da palma
# fica em Y. O envelope é dinâmico em Z para encolher quando fechados.
_HAND_ENV_X     = 0.060    # ±60 mm — largura da palma + folga
_HAND_ENV_Y_NEG = 0.080    # 80 mm — frente da palma (fingertips fechados)
_HAND_ENV_Y_POS = 0.045    # 45 mm — traseira (Link6 mesh)
_HAND_ENV_Z_NEG = 0.060    # 60 mm — atrás da palma
_PALM_BODY_Z    = 0.075    # 75 mm — palma+TCP fechado (sempre presente)
_FINGER_MAX_Z   = 0.180    # 180 mm — palma + dedos totalmente abertos


def _hand_envelope_bounds(hand_state: dict | None = None):
    """Envelope dinâmico: encolhe no eixo Z (comprimento dos dedos) com o
    fechamento. Usa o maior `primary` (rad) entre os 5 dedos como proxy.
    """
    if hand_state is None:
        close_frac = 0.0  # pessimista
    else:
        primaries = [hand_state.get(j, 0.0)
                     for j in ('Thumb', 'Index', 'Middle', 'Ring', 'Little')]
        max_p = max(primaries) if primaries else 0.0
        close_frac = max(0.0, min(1.0, max_p / 1.6))
    z_extent = _PALM_BODY_Z + (1.0 - close_frac) * (
        _FINGER_MAX_Z - _PALM_BODY_Z)
    z_extent += 0.010  # margem extra
    return (-_HAND_ENV_X, +_HAND_ENV_X,
            -_HAND_ENV_Y_NEG, +_HAND_ENV_Y_POS,
            -_HAND_ENV_Z_NEG, +z_extent)


def _hand_envelope_aabb_world(q, hand_state=None):
    """AABB world do envelope da mão+Link6 (depende do estado dos dedos)."""
    if not _COLLISION_OK:
        return None
    T_base = _np.eye(4)
    T_base[2, 3] = _GX_BASE_Z
    T_world = T_base @ _gx_fk_partial(q, 6)

    xmin, xmax, ymin, ymax, zmin, zmax = _hand_envelope_bounds(hand_state)
    pts = []
    for x in (xmin, xmax):
        for y in (ymin, ymax):
            for z in (zmin, zmax):
                p = T_world @ _np.array([x, y, z, 1.0])
                pts.append(p[:3])
    pts = _np.array(pts)
    cx = float((pts[:, 0].min() + pts[:, 0].max()) / 2)
    cy = float((pts[:, 1].min() + pts[:, 1].max()) / 2)
    cz = float((pts[:, 2].min() + pts[:, 2].max()) / 2)
    sx = float(pts[:, 0].max() - pts[:, 0].min())
    sy = float(pts[:, 1].max() - pts[:, 1].min())
    sz = float(pts[:, 2].max() - pts[:, 2].min())
    return (cx, cy, cz, sx, sy, sz)


# Obstáculos do mundo que NÃO devem bloquear o envelope da mão.
# Com o envelope corrigido (Z = direção dos dedos), pode-se checar tudo —
# mas mantemos sort_shelf_top fora porque a entrega passa baixo sobre ela.
_HAND_OBSTACLE_SKIP: set[str] = {'sort_shelf_top'}


def _hand_clears_world(q, hand_state=None, margin=0.020):
    """True se o envelope da mão não toca nenhum obstáculo estático."""
    if not _COLLISION_OK:
        return True
    bbox = _hand_envelope_aabb_world(q, hand_state=hand_state)
    if bbox is None:
        return True
    for name, obs in _GX_OBSTACLES.items():
        if name in _HAND_OBSTACLE_SKIP:
            continue
        coll, _clr = _gx_bbox_overlap(bbox, obs, margin=margin)
        if coll:
            return False
    return True


# ── ECI built-in grip IDs (CurrentGripID.value) ───────────────────────
ECI_GRIP_IDS: dict[str, int] = {
    'Tripod':       1,  'Power':        2,  'Trigger':      3,
    'Prec. Open':   4,  'Prec. Closed': 5,  'Key':          6,
    'Finger':       7,  'Cylinder':     8,  'Column':       9,
    'Relaxed':     10,  'Glove':       11,  'Tap':         12,
    'Grab':        13,  'Tripod Open': 14,
}
ECI_GRIP_NAMES: dict[int, str] = {v: k for k, v in ECI_GRIP_IDS.items()}

ECI_GRIP_GAZEBO: dict[str, dict[str, int]] = {
    'Tripod':       {'Thumb': 125, 'Index': 115, 'Middle': 115, 'Ring':   0, 'Little':  0, 'Rotate': 145},
    'Power':        {'Thumb': 155, 'Index': 165, 'Middle': 165, 'Ring': 160, 'Little': 155, 'Rotate':  40},
    'Trigger':      {'Thumb': 100, 'Index':   0, 'Middle': 140, 'Ring': 140, 'Little': 140, 'Rotate':  70},
    'Prec. Open':   {'Thumb':  50, 'Index':  50, 'Middle':   0, 'Ring':   0, 'Little':   0, 'Rotate': 155},
    'Prec. Closed': {'Thumb': 105, 'Index': 100, 'Middle':   0, 'Ring':   0, 'Little':   0, 'Rotate': 155},
    'Key':          {'Thumb': 115, 'Index': 130, 'Middle': 130, 'Ring': 125, 'Little': 115, 'Rotate':  10},
    'Finger':       {'Thumb':  60, 'Index':   0, 'Middle': 100, 'Ring': 100, 'Little': 100, 'Rotate':  60},
    'Cylinder':     {'Thumb': 130, 'Index': 150, 'Middle': 155, 'Ring': 150, 'Little': 140, 'Rotate':  35},
    'Column':       {'Thumb': 100, 'Index': 140, 'Middle': 140, 'Ring': 140, 'Little': 140, 'Rotate':  80},
    'Relaxed':      {'Thumb':  20, 'Index':  20, 'Middle':  20, 'Ring':  20, 'Little':  20, 'Rotate':   5},
    'Glove':        {'Thumb':   0, 'Index':   0, 'Middle':   0, 'Ring':   0, 'Little':   0, 'Rotate':   0},
    'Tap':          {'Thumb':   0, 'Index':   0, 'Middle': 160, 'Ring': 160, 'Little': 160, 'Rotate':  50},
    'Grab':         {'Thumb': 165, 'Index': 175, 'Middle': 175, 'Ring': 175, 'Little': 170, 'Rotate':  45},
    'Tripod Open':  {'Thumb':  60, 'Index':  50, 'Middle':  50, 'Ring':   0, 'Little':   0, 'Rotate': 145},
}

PROJECT_GRIP_ECI: dict[str, int] = {
    'Palm Grip (frasco)':      8,   # Cylinder
    'Claw Grip (tubo)':        1,   # Tripod
    'Fingertip Grip (ampola)': 5,   # Prec. Closed
}

MIMIC_JOINTS = [
    ('_lisa_j01',            'Rotate', 1.07337741974876),
    ('_thumb_chassis_j01',   'Rotate', 1.53339618284689),
    ('_thumb_proximal_j01',  'Thumb',  0.72022188617106),
    ('_thumb_distal_j01',    'Thumb',  1.06686018440504),
    ('_thumb_link_j01',      'Thumb',  0.76799454671462),
    ('_thumb_follower_j01',  'Thumb',  0.93732763826281),
    ('_index_proximal_j01',  'Index',  1.51604339913514),
    ('_index_distal_j01',    'Index',  1.33574108836936),
    ('_index_knuckle_j01',   'Index',  1.25181519799450),
    ('_index_follower_j01',  'Index',  0.26422627443924),
    ('_index_link_j01',      'Index',  1.33574038782548),
    ('_middle_proximal_j01', 'Middle', 1.51604368978713),
    ('_middle_distal_j01',   'Middle', 1.34986011532341),
    ('_middle_knuckle_j01',  'Middle', 1.25181499257525),
    ('_middle_follower_j01', 'Middle', 0.26422641895880),
    ('_middle_link_j01',     'Middle', 1.34986028913701),
    ('_ring_proximal_j01',   'Ring',   1.51604328762194),
    ('_ring_distal_j01',     'Ring',   1.34878317629563),
    ('_ring_knuckle_j01',    'Ring',   1.25181510906761),
    ('_ring_follower_j01',   'Ring',   0.26423062522385),
    ('_ring_link_j01',       'Ring',   1.34878364034377),
    ('_little_proximal_j01', 'Little', 1.51604353824541),
    ('_little_distal_j01',   'Little', 1.31664152870820),
    ('_little_knuckle_j01',  'Little', 1.25181529061989),
    ('_little_follower_j01', 'Little', 0.26422625333146),
    ('_little_link_j01',     'Little', 1.31664159359670),
]

MAX_RAD = {'Thumb': 1.6, 'Index': 1.6, 'Middle': 1.6,
           'Ring':  1.6, 'Little': 1.6, 'Rotate': 1.0}

HAND_JOINTS = ['Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Rotate']
ARM_JOINTS  = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

ARM_LIMITS_DEG = {
    'joint1': (-180, 180), 'joint2': (-260,  80),
    'joint3': (-135, 135), 'joint4': (-260,  80),
    'joint5': (-135, 135), 'joint6': (-360, 360),
}

ARM_PRESETS_BASE = {
    'Home':      {'joint1':   0, 'joint2':   0, 'joint3':  90,
                  'joint4': -90, 'joint5': -90, 'joint6':   0},
    # Vertical: braço estendido para cima com pulso recolhido.
    'Vertical':  {'joint1':   0, 'joint2':   0, 'joint3':   0,
                  'joint4': -90, 'joint5': -90, 'joint6':   0},
    'Estendido': {'joint1':   0, 'joint2':   0, 'joint3': -90,
                  'joint4':   0, 'joint5':   0, 'joint6':   0},
}

# ── Geometria do ciclo de pick (world frame, metros) — LATERAL APPROACH ─
# Pick station em (0.75, 0.0). Belt top em z=0.806.
#
# Approach LATERAL: mão se aproxima horizontalmente do lado do operador
# (-Y) com a palma virada para a frente (+Y), dedos extendidos no eixo
# +Y. Esse é o grasp natural da COVVI: cilindro deita perpendicular aos
# dedos, hand_x (largura da palma) fica vertical, e os 4 dedos longos
# se espalham ao longo do eixo do cilindro (eixo Z mundo). O polegar fica
# no lado oposto (operador) — power grasp clássico.
#
# Eixos com approach_vec=(0,1,0):
#   hand_x_world = (0, 0,+1)   thumb sobe (acima), little desce (abaixo)
#   hand_y_world = (0,+1, 0)   dedos apontam +Y (em direção ao objeto)
#   hand_z_world = (-1,0, 0)   palma normal -X (encara o operador)
#
# IK targets por objeto: hand_origin_X escolhido para que os fingertips
# ENCOSTEM a superfície do cilindro (radius-aware), sem penetração.
_PICK_XY_WORLD: tuple[float, float] = (0.75, 0.00)
_APPROACH_VEC = (0.0, 1.0, 0.0)              # lateral, de -Y para +Y
_APPROACH_TCP_WORLD: tuple = (0.75, -0.135, 0.85)  # 20 cm antes do grasp
_PICK_TCP_WORLD: dict[str, tuple] = {
    # (target_X, target_Y, target_Z) do TCP. Derivado de hand_fk:
    # • target_X = obj_X + curl_offset (≈ hand_z do fingertip mais próximo)
    # • target_Y = obj_Y + (0.115 - grasp_y_in_hand) ≈ 0.065
    # • target_Z = obj_center_Z   (hand_x vertical → spread sobre o cilindro)
    'frasco': (0.748, 0.065, 0.851),  # r=42mm, h=90mm — palm wrap front face
    'tubo':   (0.770, 0.065, 0.866),  # r=12mm, h=120mm — full cylindrical wrap
    'ampola': (0.770, 0.065, 0.844),  # r=5mm, h=75mm  — tight wrap, thin object
}


def _solve_pose(p_world: tuple[float, float, float],
                q_seed_deg: dict,
                approach_vec: tuple = _APPROACH_VEC) -> dict | None:
    """Resolve IK para TCP em `p_world` com abordagem `approach_vec` (lateral
    +Y por padrão), travando o ramo a partir da semente `q_seed_deg`.

    Se o IK global devolver um ramo diferente da semente (|Δj2|>60° ou
    |Δj1|>60°), recorre-se ao `_ik_refine` direto da semente para manter
    a coerência geométrica e evitar wrist-flipped/shoulder-back.
    """
    if not _COLLISION_OK or _gx_inverse_kinematics is None:
        return None
    p_base = _np.array([p_world[0], p_world[1], p_world[2] - _GX_BASE_Z])
    approach = _np.array(approach_vec, dtype=float)
    q_seed = _np.array([math.radians(q_seed_deg[j]) for j in ARM_JOINTS])

    q_global, ok_global = _gx_inverse_kinematics(
        p_base, approach, q_seed=q_seed, elbow_up=True)
    branch_ok = (ok_global
                 and abs(math.degrees(q_global[1]) - math.degrees(q_seed[1])) < 60.0
                 and abs(math.degrees(q_global[0]) - math.degrees(q_seed[0])) < 60.0)

    if branch_ok:
        q_final = q_global
    else:
        R_tcp = _gx_approach_to_Rtcp(approach)
        q_final, _ = _gx_ik_refine(p_base, R_tcp, q_seed)

    T = _gx_forward_kinematics(q_final)
    err_mm = float(_np.linalg.norm(T[:3, 3] - p_base)) * 1000.0
    if err_mm > 5.0:
        return None
    return {j: float(math.degrees(q_final[i])) for i, j in enumerate(ARM_JOINTS)}


# Fallbacks (graus) para approach LATERAL — sementes do IK e poses-finais
# caso o IK não esteja disponível. Calibrados com T_HAND_ATTACH atual e
# `approach_vec = (0, 1, 0)` (dedos apontando para +Y world):
_FALLBACK_APPROACH_DEG = {
    'joint1': -12.3, 'joint2': -25.5, 'joint3': -80.4,
    'joint4': -74.1, 'joint5': +167.7, 'joint6': +90.0,
}
_FALLBACK_PICK_DEG = {
    # frasco: j5≈+178, j6≈-90 — palm encara o operador, fingers wrap front
    'frasco': {'joint1':  +2.5, 'joint2': -31.2, 'joint3': -100.1,
               'joint4': +131.3, 'joint5': +177.5, 'joint6': -90.0},
    # tubo: j5≈-178, j6≈+90 — branch alternativo, equivalente em TCP
    'tubo':   {'joint1':  +2.5, 'joint2': -21.0, 'joint3':  -84.9,
               'joint4':  -74.1, 'joint5': -177.5, 'joint6': +90.0},
    # ampola: mesmo branch do frasco
    'ampola': {'joint1':  +2.4, 'joint2': -33.1, 'joint3':  -97.7,
               'joint4': +130.7, 'joint5': +177.6, 'joint6': -90.0},
}
_FALLBACK_DELIVERY_DEG = {
    'frasco': {'joint1': +108.4, 'joint2': -19.6, 'joint3': -91.3,
               'joint4':  -69.1, 'joint5': +108.4, 'joint6':  +0.0},
    'tubo':   {'joint1':  +85.6, 'joint2': -23.2, 'joint3': -86.3,
               'joint4':  -70.5, 'joint5':  +85.6, 'joint6':  +0.0},
    'ampola': {'joint1':  +65.8, 'joint2': -36.5, 'joint3': -65.6,
               'joint4':  -77.9, 'joint5':  +65.8, 'joint6':  +0.0},
}

# Caixas de entrega — mesma altura e XY que as poses originais (acima do topo).
_DELIVERY_XY_WORLD = {
    'frasco': (-0.05, 0.65),
    'tubo':   ( 0.25, 0.65),
    'ampola': ( 0.55, 0.65),
}
_DELIVERY_Z_WORLD = 0.75   # 15 cm acima do topo das caixas


def _compute_pick_targets():
    """Computa via IK as poses de approach (lateral, -Y → +Y), grasp por
    objeto e entrega por objeto. Roda uma vez no carregamento do módulo.

    Cadeia de seeds:  approach → frasco → tubo, ampola.  Cada pose usa a
    anterior como warm-start para garantir branch coerente.
    """
    approach_deg = (_solve_pose(_APPROACH_TCP_WORLD,
                                 q_seed_deg=_FALLBACK_APPROACH_DEG)
                    or _FALLBACK_APPROACH_DEG)

    pick_deg: dict[str, dict] = {}
    # Frasco (semeado do approach).
    pick_deg['frasco'] = (_solve_pose(_PICK_TCP_WORLD['frasco'],
                                       q_seed_deg=_FALLBACK_PICK_DEG['frasco'])
                          or _FALLBACK_PICK_DEG['frasco'])
    # Tubo e ampola — seeds fallback dedicados (cada um pode estar num
    # branch diferente do frasco, o que é OK).
    for obj in ('tubo', 'ampola'):
        pick_deg[obj] = (_solve_pose(_PICK_TCP_WORLD[obj],
                                      q_seed_deg=_FALLBACK_PICK_DEG[obj])
                         or _FALLBACK_PICK_DEG[obj])

    delivery_deg: dict[str, dict] = {}
    for obj in ('frasco', 'tubo', 'ampola'):
        dx, dy = _DELIVERY_XY_WORLD[obj]
        # Entrega ainda usa approach top-down (depositar nas caixas).
        delivery_deg[obj] = (_solve_pose(
                                  (dx, dy, _DELIVERY_Z_WORLD),
                                  q_seed_deg=_FALLBACK_DELIVERY_DEG[obj],
                                  approach_vec=(0.0, 0.0, -1.0))
                             or _FALLBACK_DELIVERY_DEG[obj])
    return approach_deg, pick_deg, delivery_deg


APPROACH_POSE_DEG, PICK_POSES_DEG, DELIVERY_POSES_DEG = _compute_pick_targets()


HAND_OPEN = {j: 0 for j in HAND_JOINTS}

# Grips por objeto — derivados de HAND_CONFIGS (kinematics.py) e
# convertidos para sliders (0–200). Com T_HAND_ATTACH alinhada com a
# direção dos dedos, é a CURL TOTAL que aproxima as pontas do objeto:
#   slider = rad / max_rad * 200, com max_rad=1.6 (dedos) e 1.0 (Rotate).
#
# Para os 4 dedos longos (Index/Middle/Ring/Little), curl ≥ 1.10 rad
# (slider ≥ 138) faz o tip "passar" da vertical e voltar em direção à
# palma — aperta cilindros até r~40mm. Para r<15mm a geometria da palma
# (5cm de espalhamento em hand_x) limita a precisão do contato; o grasp
# real é por fricção em vez de pinch cirúrgico.
HAND_GRIPS = {
    'Open':                     HAND_OPEN,
    # frasco r=42mm: palm grip — 4 dedos longos abraçam a frente do
    # cilindro, polegar pressiona pelo lado oposto (operador).
    'Palm Grip (frasco)':       {'Thumb': 138, 'Index': 156, 'Middle': 156,
                                 'Ring':  150, 'Little': 138, 'Rotate':  50},
    # tubo r=12mm: fechamento mais apertado, dedos curvam past o cilindro
    # para realmente "abraçar" o tubo fino.
    'Claw Grip (tubo)':         {'Thumb': 180, 'Index': 200, 'Middle': 200,
                                 'Ring':  190, 'Little': 180, 'Rotate': 140},
    # ampola r=5mm: máximo fechamento + Rotate máximo. Cylinder muito
    # fino → o grasp é por fricção lateral, não pinch exato.
    'Fingertip Grip (ampola)':  {'Thumb': 200, 'Index': 200, 'Middle': 200,
                                 'Ring':  190, 'Little': 180, 'Rotate': 200},
}

OBJ_GRIP = {
    'frasco': 'Palm Grip (frasco)',
    'tubo':   'Claw Grip (tubo)',
    'ampola': 'Fingertip Grip (ampola)',
}


# ── Paleta CRStudio-like (tema claro industrial) ──────────────────────
BG          = '#f2f3f5'   # cinza claro (fundo da janela)
PANEL       = '#ffffff'   # branco (cartões)
SURFACE     = '#fafbfc'   # cinza muito claro (linhas alternadas)
BORDER      = '#d6dae0'   # cinza linha
HEADER      = '#2c3e50'   # azul escuro corporativo
HEADER_FG   = '#ffffff'
TEXT        = '#1f2937'
TEXT_DIM    = '#6b7280'
TEXT_MUTED  = '#9ca3af'
VALUE_FG    = '#0f172a'

PRIMARY     = '#2563eb'   # azul ação principal
PRIMARY_HV  = '#1d4ed8'
ACCENT_ARM  = '#0e7490'   # ciano braço
ACCENT_HND  = '#15803d'   # verde mão
DANGER      = '#dc2626'   # vermelho E-stop
DANGER_HV   = '#b91c1c'
WARN        = '#d97706'
OK          = '#16a34a'

BTN_NEUTRAL = '#e5e7eb'   # cinza claro botão neutro
BTN_NEUTRAL_HV = '#d1d5db'
TROUGH      = '#e5e7eb'

COLOR_FRASCO, COLOR_TUBO, COLOR_AMPOLA = '#ea580c', '#0369a1', '#15803d'

FONT_TITLE  = ('Segoe UI', 13, 'bold')
FONT_H      = ('Segoe UI', 11, 'bold')
FONT_BODY   = ('Segoe UI', 10)
FONT_LBL    = ('Segoe UI', 9)
FONT_SMALL  = ('Segoe UI', 8)
FONT_MONO   = ('Consolas', 10, 'bold')
FONT_MONO_S = ('Consolas', 9)


def _flat_btn(parent, text, command, *, bg=BTN_NEUTRAL, fg=TEXT,
              hover=None, font=FONT_LBL, padx=10, pady=5, width=None):
    """Botão chato sem borda, com efeito de hover sutil."""
    hover_bg = hover or _shade(bg, -0.08)
    btn = tk.Button(parent, text=text, command=command,
                    bg=bg, fg=fg,
                    activebackground=hover_bg, activeforeground=fg,
                    relief='flat', bd=0, padx=padx, pady=pady,
                    font=font, cursor='hand2',
                    highlightthickness=1, highlightbackground=BORDER,
                    highlightcolor=BORDER)
    if width:
        btn.config(width=width)
    btn.bind('<Enter>', lambda e: btn.config(bg=hover_bg))
    btn.bind('<Leave>', lambda e: btn.config(bg=bg))
    return btn


def _shade(hex_color: str, amount: float) -> str:
    """Clareia (amount>0) ou escurece (amount<0) uma cor HEX."""
    h = hex_color.lstrip('#')
    r, g, b = int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
    if amount >= 0:
        r = int(r + (255 - r) * amount)
        g = int(g + (255 - g) * amount)
        b = int(b + (255 - b) * amount)
    else:
        r = int(r * (1 + amount))
        g = int(g * (1 + amount))
        b = int(b * (1 + amount))
    r, g, b = max(0, min(255, r)), max(0, min(255, g)), max(0, min(255, b))
    return f'#{r:02x}{g:02x}{b:02x}'


def _card(parent, title: str | None = None):
    """Cria um container 'card' com borda fina e cabeçalho opcional."""
    outer = tk.Frame(parent, bg=BORDER, bd=0)
    inner = tk.Frame(outer, bg=PANEL)
    inner.pack(fill='both', expand=True, padx=1, pady=1)
    if title:
        ttl = tk.Label(inner, text=title, font=FONT_H,
                       bg=PANEL, fg=TEXT, anchor='w',
                       padx=12, pady=8)
        ttl.pack(fill='x')
        tk.Frame(inner, bg=BORDER, height=1).pack(fill='x')
    return outer, inner


class StatusLED(tk.Canvas):
    """Pequeno LED circular usado nos badges do cabeçalho."""

    def __init__(self, parent, **kw):
        super().__init__(parent, width=12, height=12,
                         bg=kw.pop('bg', HEADER), highlightthickness=0)
        self._dot = self.create_oval(2, 2, 11, 11, fill=TEXT_MUTED, outline='')

    def set_state(self, color: str):
        self.itemconfig(self._dot, fill=color)


class ManualControlNode(Node):
    """Nó ROS 2 + GUI Tkinter para controle manual da célula (estilo CRStudio)."""

    def __init__(self, root: tk.Tk):
        super().__init__('manual_control')
        self._ready = False
        self._suppressing = False
        self._pending_hand_after: str | None = None
        self.root = root

        cb = ReentrantCallbackGroup()

        self.arm_pub = self.create_publisher(
            JointTrajectory, '/cr10_group_controller/joint_trajectory', 10)
        self.hand_pub = self.create_publisher(
            JointTrajectory, '/hand_position_controller/joint_trajectory', 10)

        self._cli = {
            'retreat':       self.create_client(
                Trigger, '/conveyor/retreat',       callback_group=cb),
            'reset':         self.create_client(
                Trigger, '/conveyor/reset',         callback_group=cb),
            'spawn_frasco':  self.create_client(
                Trigger, '/conveyor/spawn_frasco',  callback_group=cb),
            'spawn_tubo':    self.create_client(
                Trigger, '/conveyor/spawn_tubo',    callback_group=cb),
            'spawn_ampola':  self.create_client(
                Trigger, '/conveyor/spawn_ampola',  callback_group=cb),
        }

        self._conveyor_state: dict = {}
        self._last_detection: str | None = None
        self._status_q: queue.Queue = queue.Queue()

        self.create_subscription(
            String, '/conveyor/status', self._cb_conveyor, 10,
            callback_group=cb)
        self.create_subscription(
            Detection2DArray, '/detected_objects', self._cb_detection, 10,
            callback_group=cb)

        # ECI real-hand (lazy)
        self._eci_enabled = False
        eci_prefix = self.declare_parameter('eci_prefix', '/covvi/hand').value
        self._eci_prefix: str = eci_prefix
        self._eci_grip_id: int | None = None
        self._eci_posn: dict[str, int] = {}
        self._eci_last_sent: str | None = None
        self._eci_posn_after: str | None = None
        self._eci_srv = None
        self._eci_msg = None
        self._cli_eci = None
        self._cli_eci_posn = None

        # Conexão com o driver ECI (subprocesso `covvi_hand_driver server`)
        self._hand_proc: subprocess.Popen | None = None
        self._hand_ip_var: tk.StringVar | None = None  # set em _build_header

        # Estado do fechamento incremental (smart close)
        self._smart_after: str | None = None
        # Sequenciador do ciclo de pick / entrega
        self._pick_cycle_after: str | None = None

        # Sistema de colisão: rastreia a última configuração seguramente
        # publicada para usar como ponto de partida do sweep do braço.
        if _COLLISION_OK:
            self._last_safe_q = _np.array([
                math.radians(ARM_PRESETS_BASE['Home'][j]) for j in ARM_JOINTS
            ])
        else:
            self._last_safe_q = None

        self._build_ui()
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self.root.deiconify()
        self._ready = True
        self._spin_ros()
        self._poll_status()

    # ──────────────────────────────────────────────────────────────────
    # Conveyor & detection callbacks
    # ──────────────────────────────────────────────────────────────────
    def _cb_conveyor(self, msg: String):
        try:
            self._conveyor_state = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _cb_detection(self, msg: Detection2DArray):
        if msg.detections:
            try:
                cls = msg.detections[0].results[0].hypothesis.class_id
                self._last_detection = cls if cls in DELIVERY_POSES_DEG else None
            except (IndexError, AttributeError):
                self._last_detection = None
        else:
            self._last_detection = None

    def _call_conveyor(self, key: str, pending_msg: str | None = None):
        cli = self._cli[key]
        if not cli.service_is_ready():
            self._set_status(f'Serviço /conveyor/{key} indisponível', _CLR=TEXT_DIM)
            return
        if pending_msg:
            self._set_status(pending_msg, _CLR=WARN)
        future = cli.call_async(Trigger.Request())

        def _done(f):
            try:
                res = f.result()
                color = OK if res.success else DANGER
                self._status_q.put((res.message, color))
            except Exception as exc:
                self._status_q.put((str(exc), DANGER))

        future.add_done_callback(_done)

    # ──────────────────────────────────────────────────────────────────
    # ECI callbacks
    # ──────────────────────────────────────────────────────────────────
    def _cb_eci_grip(self, msg) -> None:
        self._eci_grip_id = int(msg.grip_id.value)

    def _cb_eci_posn(self, msg) -> None:
        self._eci_posn = {
            'Thumb':  msg.thumb_pos, 'Index':  msg.index_pos,
            'Middle': msg.middle_pos, 'Ring':   msg.ring_pos,
            'Little': msg.little_pos, 'Rotate': msg.rotate_pos,
        }

    def _cb_eci_touch(self, msg) -> None:
        """DigitTouchAllMsg: força de toque por dedo (0-255)."""
        self._touch_values = {
            'Thumb':  int(msg.thumb_touch),
            'Index':  int(msg.index_touch),
            'Middle': int(msg.middle_touch),
            'Ring':   int(msg.ring_touch),
            'Little': int(msg.little_touch),
        }

    def _cb_eci_current(self, msg) -> None:
        """MotorCurrentAllMsg: corrente do motor por dedo (0-255, proxy de força)."""
        self._current_values = {
            'Thumb':  int(msg.thumb_current),
            'Index':  int(msg.index_current),
            'Middle': int(msg.middle_current),
            'Ring':   int(msg.ring_current),
            'Little': int(msg.little_current),
        }

    def _send_eci_grip(self, grip_id: int, label: str) -> None:
        if not self._eci_enabled or self._cli_eci is None:
            return
        if not self._cli_eci.service_is_ready():
            self._status_q.put(
                (f'ECI SetCurrentGrip indisponível ({self._eci_prefix})', DANGER))
            return
        req = self._eci_srv.SetCurrentGrip.Request()
        req.grip_id = self._eci_msg.CurrentGripID()
        req.grip_id.value = grip_id
        self._eci_last_sent = label
        future = self._cli_eci.call_async(req)

        def _done(f):
            try:
                f.result()
                self._status_q.put(
                    (f'ECI > {label} (id={grip_id})', OK))
            except Exception as exc:
                self._status_q.put((f'ECI erro: {exc}', DANGER))

        future.add_done_callback(_done)

    def _schedule_eci_posn(self, vals: dict) -> None:
        if not self._eci_enabled or self._cli_eci_posn is None:
            return
        if self._eci_posn_after is not None:
            try:
                self.root.after_cancel(self._eci_posn_after)
            except Exception:
                pass
        self._eci_posn_after = self.root.after(
            60, lambda v=dict(vals): self._send_eci_posn_now(v))

    def _send_eci_posn_now(self, vals: dict) -> None:
        self._eci_posn_after = None
        if not self._eci_enabled or self._cli_eci_posn is None:
            return
        if not self._cli_eci_posn.service_is_ready():
            self._status_q.put(
                (f'ECI SetDigitPosn indisponível ({self._eci_prefix})', DANGER))
            return
        req = self._eci_srv.SetDigitPosn.Request()
        req.speed = self._eci_msg.Speed()
        req.speed.value = 50
        req.thumb  = int(vals.get('Thumb',  0))
        req.index  = int(vals.get('Index',  0))
        req.middle = int(vals.get('Middle', 0))
        req.ring   = int(vals.get('Ring',   0))
        req.little = int(vals.get('Little', 0))
        req.rotate = int(vals.get('Rotate', 0))
        self._eci_last_sent = (
            f"T:{req.thumb} I:{req.index} M:{req.middle} "
            f"R:{req.ring} L:{req.little} Rot:{req.rotate}"
        )
        future = self._cli_eci_posn.call_async(req)
        future.add_done_callback(lambda f: None)

    def _toggle_eci(self) -> None:
        if self._eci_enabled:
            self._eci_enabled = False
            self._eci_btn.config(text='ECI OFF', bg=BTN_NEUTRAL, fg=TEXT)
            self._led_eci.set_state(TEXT_MUTED)
            self._lbl_eci.config(text='desativada', fg=TEXT_DIM)
            if hasattr(self, '_force_src_lbl'):
                self._force_src_lbl.config(
                    text='(simulado — proxy do comando)', fg=TEXT_MUTED)
            self._set_status('Mão real desativada', _CLR=TEXT_DIM)
            return

        if self._cli_eci is None:
            try:
                import covvi_interfaces.srv as _eci_srv
                import covvi_interfaces.msg as _eci_msg
            except ImportError:
                self._set_status(
                    'covvi_interfaces não instalado — instale e recompile.',
                    _CLR=DANGER)
                return
            self._eci_srv = _eci_srv
            self._eci_msg = _eci_msg
            cb = ReentrantCallbackGroup()
            self._cli_eci = self.create_client(
                _eci_srv.SetCurrentGrip,
                f'{self._eci_prefix}/SetCurrentGrip',
                callback_group=cb)
            self._cli_eci_posn = self.create_client(
                _eci_srv.SetDigitPosn,
                f'{self._eci_prefix}/SetDigitPosn',
                callback_group=cb)
            # Sensores de força/toque da mão COVVI
            self.create_subscription(
                _eci_msg.DigitTouchAllMsg,
                f'{self._eci_prefix}/DigitTouchAllMsg',
                self._cb_eci_touch, 10, callback_group=cb)
            self.create_subscription(
                _eci_msg.MotorCurrentAllMsg,
                f'{self._eci_prefix}/MotorCurrentAllMsg',
                self._cb_eci_current, 10, callback_group=cb)
            self.create_subscription(
                _eci_msg.CurrentGripMsg,
                f'{self._eci_prefix}/CurrentGripMsg',
                self._cb_eci_grip, 10, callback_group=cb)
            self.create_subscription(
                _eci_msg.DigitPosnAllMsg,
                f'{self._eci_prefix}/DigitPosnAllMsg',
                self._cb_eci_posn, 10, callback_group=cb)

        self._eci_enabled = True
        self._eci_btn.config(text='ECI ON', bg=ACCENT_HND, fg='white')
        self._led_eci.set_state(OK)
        self._lbl_eci.config(text=self._eci_prefix, fg=HEADER_FG)
        if hasattr(self, '_force_src_lbl'):
            self._force_src_lbl.config(
                text='(real — DigitTouch + MotorCurrent)', fg=ACCENT_HND)
        self._set_status(f'Mão real ativada ({self._eci_prefix})', _CLR=OK)

    # ──────────────────────────────────────────────────────── REAL-HAND CONNECT
    def _connect_real_hand(self) -> None:
        """Sobe `covvi_hand_driver server <IP>` num subprocesso e ativa o ECI."""
        if self._hand_proc is not None and self._hand_proc.poll() is None:
            self._set_status('Driver da mão já em execução.', _CLR=WARN)
            return

        ip = (self._hand_ip_var.get() if self._hand_ip_var else '').strip()
        if not ip:
            self._set_status('Informe o IP da mão COVVI.', _CLR=DANGER)
            return

        # Quebrar o eci_prefix em (namespace, name) para o remap do ROS.
        parts = self._eci_prefix.strip('/').split('/')
        ns = '/' + parts[0]
        name = parts[1] if len(parts) > 1 else 'server'

        cmd = ['ros2', 'run', 'covvi_hand_driver', 'server', ip,
               '--ros-args',
               '--remap', f'__ns:={ns}',
               '--remap', f'__name:={name}']
        try:
            self._hand_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid)
        except FileNotFoundError:
            self._set_status(
                'ros2 não encontrado no PATH. Faça source do workspace antes.',
                _CLR=DANGER)
            self._hand_proc = None
            return
        except Exception as exc:
            self._set_status(f'Erro ao iniciar driver: {exc}', _CLR=DANGER)
            self._hand_proc = None
            return

        self._connect_btn.config(text='Conectando…', state='disabled')
        self._set_status(
            f'Iniciando driver da mão em {ip} (aguarde 2 s)…', _CLR=WARN)
        self.root.after(2200, self._post_connect_real_hand)

    def _post_connect_real_hand(self) -> None:
        """Verifica o subprocesso e ativa o toggle ECI automaticamente."""
        proc = self._hand_proc
        if proc is None or proc.poll() is not None:
            self._set_status(
                'Driver da mão falhou ao iniciar — veja o terminal.',
                _CLR=DANGER)
            self._connect_btn.config(
                text='Conectar', state='normal',
                bg=PRIMARY, fg='white',
                command=self._connect_real_hand)
            self._hand_proc = None
            return

        if not self._eci_enabled:
            self._toggle_eci()

        self._connect_btn.config(
            text='Desconectar', state='normal',
            bg=DANGER, fg='white',
            activebackground=DANGER_HV,
            command=self._disconnect_real_hand)

    def _disconnect_real_hand(self) -> None:
        """Mata o subprocesso do driver e desativa o ECI."""
        if self._eci_enabled:
            self._toggle_eci()

        proc = self._hand_proc
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=2.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
        self._hand_proc = None
        self._connect_btn.config(
            text='Conectar', state='normal',
            bg=PRIMARY, fg='white',
            activebackground=PRIMARY_HV,
            command=self._connect_real_hand)
        self._set_status('Mão real desconectada.', _CLR=TEXT_DIM)

    def _on_close(self) -> None:
        """Garante que o subprocesso do driver morra junto com a GUI."""
        try:
            if self._hand_proc is not None and self._hand_proc.poll() is None:
                os.killpg(os.getpgid(self._hand_proc.pid), signal.SIGTERM)
        except Exception:
            pass
        self.root.destroy()

    def _estop(self) -> None:
        """Parada de emergência (lado simulação): home + abrir mão + status."""
        # Mantém a mesma semântica do antigo: leva o braço a Home e abre a mão.
        # Em produção ligar também ao DisableRobot() do CR10 real.
        self._apply_arm(ARM_PRESETS_BASE['Home'])
        self._apply_hand({j: 0 for j in HAND_JOINTS})
        if self._eci_enabled:
            self._send_eci_grip(11, 'Glove (E-STOP)')   # mão aberta
        self._set_status('E-STOP — braço a Home, mão aberta.', _CLR=DANGER)

    # ──────────────────────────────────────────────────────────────────
    # UI construction
    # ──────────────────────────────────────────────────────────────────
    def _build_ui(self):
        self.root.title('CR10 + COVVI  —  Manual Control')
        self.root.configure(bg=BG)
        self.root.minsize(1080, 720)

        style = ttk.Style()
        style.theme_use('clam')

        # Notebook (abas)
        style.configure('TNotebook', background=BG, borderwidth=0)
        style.configure('TNotebook.Tab', background=BTN_NEUTRAL,
                        foreground=TEXT, padding=(18, 8), font=FONT_LBL,
                        borderwidth=0)
        style.map('TNotebook.Tab',
                  background=[('selected', PANEL)],
                  foreground=[('selected', PRIMARY)],
                  expand=[('selected', [0, 0, 0, 0])])

        # Separator
        style.configure('Sep.TFrame', background=BORDER)

        self._build_header()
        self._build_tabs()
        self._build_statusbar()

    # ──────────────────────────────────────────────────────── HEADER
    def _build_header(self):
        # Bandeira azul mais alta para caber título, badges, conexão e E-STOP
        hdr = tk.Frame(self.root, bg=HEADER, height=110)
        hdr.pack(fill='x', side='top')
        hdr.pack_propagate(False)

        # Linha 1 — título à esquerda, conexão e E-STOP à direita ─────────
        top = tk.Frame(hdr, bg=HEADER)
        top.pack(fill='x', padx=18, pady=(10, 0))

        title_box = tk.Frame(top, bg=HEADER)
        title_box.pack(side='left')
        tk.Label(title_box, text='CR10 + COVVI', font=FONT_TITLE,
                 bg=HEADER, fg=HEADER_FG).pack(anchor='w')
        tk.Label(title_box, text='Célula de Manufatura — Manual Control',
                 font=FONT_SMALL, bg=HEADER, fg='#cbd5e1').pack(anchor='w')

        # E-STOP no extremo direito
        estop = tk.Button(
            top, text='E-STOP', command=self._estop,
            bg=DANGER, fg='white',
            activebackground=DANGER_HV, activeforeground='white',
            relief='flat', bd=0, padx=22, pady=10,
            font=('Segoe UI', 11, 'bold'), cursor='hand2')
        estop.pack(side='right', padx=(12, 0))

        # Painel de conexão à mão real (rótulo + IP + Conectar + ECI)
        conn = tk.Frame(top, bg=HEADER)
        conn.pack(side='right')

        tk.Label(conn, text='MÃO COVVI', font=FONT_SMALL,
                 bg=HEADER, fg='#cbd5e1'
                 ).grid(row=0, column=0, columnspan=3, sticky='w',
                        pady=(0, 2))

        tk.Label(conn, text='IP:', font=FONT_LBL,
                 bg=HEADER, fg=HEADER_FG
                 ).grid(row=1, column=0, sticky='w', padx=(0, 6))

        self._hand_ip_var = tk.StringVar(value='192.168.1.123')
        ip_entry = tk.Entry(conn, textvariable=self._hand_ip_var,
                            width=16, font=FONT_MONO_S, bg='white',
                            fg=TEXT, relief='flat', bd=0,
                            highlightthickness=1,
                            highlightbackground=BORDER,
                            highlightcolor=PRIMARY,
                            justify='center')
        ip_entry.grid(row=1, column=1, padx=(0, 8), ipady=4, sticky='w')

        self._connect_btn = tk.Button(
            conn, text='Conectar', command=self._connect_real_hand,
            bg=PRIMARY, fg='white',
            activebackground=PRIMARY_HV, activeforeground='white',
            relief='flat', bd=0, padx=14, pady=4,
            font=FONT_LBL, cursor='hand2')
        self._connect_btn.grid(row=1, column=2, sticky='w', padx=(0, 8))

        self._eci_btn = tk.Button(
            conn, text='ECI OFF', command=self._toggle_eci,
            bg=BTN_NEUTRAL, fg=TEXT,
            activebackground=BTN_NEUTRAL_HV, activeforeground=TEXT,
            relief='flat', bd=0, padx=12, pady=4,
            font=FONT_SMALL, cursor='hand2')
        self._eci_btn.grid(row=1, column=3, sticky='w')

        # Linha 2 — badges de estado ───────────────────────────────────────
        mid = tk.Frame(hdr, bg=HEADER)
        mid.pack(fill='x', padx=18, pady=(8, 10))

        self._led_arm,  self._lbl_arm  = self._badge(mid, 'Arm',  OK, 'pronto')
        self._led_hand, self._lbl_hand = self._badge(mid, 'Hand', OK, 'pronto')
        self._led_eci,  self._lbl_eci  = self._badge(mid, 'ECI',  TEXT_MUTED,
                                                      'desativada')
        self._led_belt, self._lbl_belt = self._badge(mid, 'Belt', TEXT_MUTED,
                                                      'vazia')

        # Linha 3 — barra de sensores (força/toque por dedo) ──────────────
        self._build_sensor_bar()

    def _badge(self, parent, label: str, led_color: str, text: str):
        cell = tk.Frame(parent, bg=HEADER)
        cell.pack(side='left', padx=10)
        row = tk.Frame(cell, bg=HEADER)
        row.pack(anchor='w')
        led = StatusLED(row, bg=HEADER)
        led.set_state(led_color)
        led.pack(side='left', padx=(0, 5))
        tk.Label(row, text=label, font=FONT_LBL, bg=HEADER,
                 fg='#cbd5e1').pack(side='left')
        lbl = tk.Label(cell, text=text, font=FONT_SMALL, bg=HEADER,
                       fg=HEADER_FG, anchor='w')
        lbl.pack(anchor='w')
        return led, lbl

    # ──────────────────────────────────────────────────────── SENSOR BAR
    def _build_sensor_bar(self):
        """Faixa fina sob o header com 5 bargraphs (T/I/M/R/L).

        Cada dedo tem (label, barra horizontal de 0-255, valor numérico).
        Quando ECI está ligado, exibe DigitTouchAllMsg / MotorCurrentAllMsg
        do covvi_interfaces. Caso contrário, usa o comando atual dos sliders
        como proxy de força exercida.
        """
        strip = tk.Frame(self.root, bg=SURFACE, height=52)
        strip.pack(fill='x', side='top')
        strip.pack_propagate(False)
        tk.Frame(strip, bg=BORDER, height=1).pack(fill='x', side='top')

        inner = tk.Frame(strip, bg=SURFACE)
        inner.pack(fill='both', expand=True, padx=14, pady=4)

        tk.Label(inner, text='SENSORES', font=FONT_SMALL,
                 bg=SURFACE, fg=TEXT_MUTED).pack(side='left', padx=(0, 12))

        self._force_widgets: dict[str, tuple[tk.Canvas, tk.Label]] = {}
        # Estado interno: força "real" (toque) e "proxy" (corrente do motor).
        self._touch_values:   dict[str, int] = {j: 0 for j in
                                                ('Thumb', 'Index', 'Middle',
                                                 'Ring', 'Little')}
        self._current_values: dict[str, int] = dict(self._touch_values)

        self._force_mode_var = tk.StringVar(value='sim')

        for finger in ('Thumb', 'Index', 'Middle', 'Ring', 'Little'):
            cell = tk.Frame(inner, bg=SURFACE)
            cell.pack(side='left', padx=8, fill='y')

            tk.Label(cell, text=finger[0], font=('Segoe UI', 9, 'bold'),
                     bg=SURFACE, fg=TEXT_DIM, width=2
                     ).grid(row=0, column=0, sticky='w', padx=(0, 4))

            cv = tk.Canvas(cell, width=110, height=10,
                           bg='white', highlightthickness=1,
                           highlightbackground=BORDER, bd=0)
            cv.grid(row=0, column=1, sticky='w')
            cv.create_rectangle(0, 0, 0, 10, fill=ACCENT_HND,
                                outline='', tags='bar')

            val = tk.Label(cell, text='0', font=FONT_MONO_S,
                           bg=SURFACE, fg=VALUE_FG, width=4, anchor='e')
            val.grid(row=0, column=2, sticky='w', padx=(6, 0))

            self._force_widgets[finger] = (cv, val)

        # Indicador de fonte: real (ECI) vs simulado
        self._force_src_lbl = tk.Label(
            inner, text='(simulado — proxy do comando)',
            font=FONT_SMALL, bg=SURFACE, fg=TEXT_MUTED)
        self._force_src_lbl.pack(side='right', padx=(0, 4))

    def _update_force_bar(self, finger: str, value: int):
        """Atualiza um bargraph: largura proporcional ao valor 0..255."""
        if finger not in self._force_widgets:
            return
        cv, val_lbl = self._force_widgets[finger]
        v = max(0, min(255, int(value)))
        width_px = int(110 * v / 255)
        # cor: verde<80, amarelo 80-160, vermelho >160
        if v < 80:
            color = ACCENT_HND
        elif v < 160:
            color = WARN
        else:
            color = DANGER
        cv.coords('bar', 0, 0, width_px, 10)
        cv.itemconfig('bar', fill=color)
        val_lbl.config(text=str(v))

    # ──────────────────────────────────────────────────────── TABS
    def _build_tabs(self):
        wrapper = tk.Frame(self.root, bg=BG)
        wrapper.pack(fill='both', expand=True, padx=14, pady=12)

        nb = ttk.Notebook(wrapper)
        nb.pack(fill='both', expand=True)

        tab_jog = tk.Frame(nb, bg=BG, padx=14, pady=14)
        tab_hand = tk.Frame(nb, bg=BG, padx=14, pady=14)
        tab_cell = tk.Frame(nb, bg=BG, padx=14, pady=14)
        nb.add(tab_jog,  text='  Jog (Braço)  ')
        nb.add(tab_hand, text='  Mão COVVI  ')
        nb.add(tab_cell, text='  Célula  ')

        self._build_arm_tab(tab_jog)
        self._build_hand_tab(tab_hand)
        self._build_cell_tab(tab_cell)

    # ──────────────────────────────────────────────────────── ARM TAB
    def _build_arm_tab(self, parent):
        parent.columnconfigure(0, weight=3)
        parent.columnconfigure(1, weight=2)
        parent.rowconfigure(0, weight=1)

        # Cartão 1: sliders por junta
        card_jog, jog_body = _card(parent, 'Posição articular (graus)')
        card_jog.grid(row=0, column=0, sticky='nsew', padx=(0, 8))

        body = tk.Frame(jog_body, bg=PANEL, padx=14, pady=8)
        body.pack(fill='both', expand=True)

        self.arm_sliders: dict[str, tk.Scale] = {}
        self.arm_value_vars: dict[str, tk.StringVar] = {}
        self.arm_entries: dict[str, tk.Entry] = {}
        for i, j in enumerate(ARM_JOINTS):
            lo, hi = ARM_LIMITS_DEG[j]
            row_bg = SURFACE if i % 2 == 0 else PANEL
            row = tk.Frame(body, bg=row_bg)
            row.pack(fill='x', pady=0, ipady=6)

            tk.Label(row, text=j.upper(), font=FONT_H,
                     bg=row_bg, fg=ACCENT_ARM, width=8, anchor='w',
                     padx=8).pack(side='left')

            var = tk.StringVar(value=' +0°')
            self.arm_value_vars[j] = var
            ent = tk.Entry(row, textvariable=var, font=FONT_MONO,
                           bg=row_bg, fg=VALUE_FG, width=7,
                           justify='right', relief='flat', bd=0,
                           highlightthickness=1,
                           highlightbackground=row_bg,
                           highlightcolor=PRIMARY,
                           insertbackground=VALUE_FG)
            ent.pack(side='right', padx=8, ipady=2)
            ent.bind('<Return>',
                     lambda e, jn=j: (self._arm_entry_commit(jn),
                                       e.widget.master.focus_set()))
            ent.bind('<FocusOut>',
                     lambda e, jn=j: self._arm_entry_commit(jn))
            ent.bind('<Escape>',
                     lambda e, jn=j: (self._arm_entry_restore(jn),
                                       e.widget.master.focus_set()))
            self.arm_entries[j] = ent

            tk.Label(row, text=f'{hi:+d}', font=FONT_SMALL,
                     bg=row_bg, fg=TEXT_MUTED, width=5).pack(side='right')

            sl = tk.Scale(row, from_=lo, to=hi, resolution=1,
                          orient='horizontal', showvalue=False,
                          bg=row_bg, troughcolor=TROUGH,
                          activebackground=ACCENT_ARM, fg=row_bg,
                          highlightthickness=0, sliderrelief='flat',
                          sliderlength=18, width=10,
                          command=lambda v, jn=j:
                                  self._arm_slider_changed(v, jn))
            sl.set(0)
            sl.pack(side='left', fill='x', expand=True, padx=(4, 4))

            tk.Label(row, text=f'{lo:+d}', font=FONT_SMALL,
                     bg=row_bg, fg=TEXT_MUTED, width=5).pack(side='left')
            self.arm_sliders[j] = sl

        # Duração + velocidade
        ctrl = tk.Frame(body, bg=PANEL, pady=12)
        ctrl.pack(fill='x')
        tk.Label(ctrl, text='Duração do movimento', font=FONT_LBL,
                 bg=PANEL, fg=TEXT_DIM).pack(side='left')
        self.dur_val_lbl = tk.Label(ctrl, text='2.0 s', font=FONT_MONO,
                                     bg=PANEL, fg=PRIMARY, width=7)
        self.dur_val_lbl.pack(side='right')
        self.time_sl = tk.Scale(
            ctrl, from_=0.3, to=8.0, resolution=0.1,
            orient='horizontal', length=240, showvalue=False,
            bg=PANEL, troughcolor=TROUGH, activebackground=PRIMARY,
            highlightthickness=0, sliderrelief='flat',
            sliderlength=16, width=8,
            command=lambda v: self.dur_val_lbl.config(text=f'{float(v):.1f} s'))
        self.time_sl.set(2.0)
        self.time_sl.pack(side='right', padx=10)

        # Cartão 2: presets
        card_p, p_body = _card(parent, 'Poses pré-calculadas')
        card_p.grid(row=0, column=1, sticky='nsew', padx=(8, 0))

        pad = tk.Frame(p_body, bg=PANEL, padx=14, pady=10)
        pad.pack(fill='both', expand=True)

        tk.Label(pad, text='POSES BASE', font=FONT_SMALL,
                 bg=PANEL, fg=TEXT_MUTED).pack(anchor='w', pady=(0, 4))
        for label, preset in ARM_PRESETS_BASE.items():
            b = _flat_btn(pad, label,
                          command=lambda p=preset: self._apply_arm(p),
                          bg=PANEL, padx=12, pady=7, font=FONT_BODY)
            b.pack(fill='x', pady=2)

        tk.Label(pad, text='APROXIMAÇÃO MANUAL', font=FONT_SMALL,
                 bg=PANEL, fg=TEXT_MUTED).pack(anchor='w', pady=(14, 4))
        _flat_btn(pad, 'Approach  (15 cm acima da estação de pick)',
                  command=lambda: self._apply_arm(APPROACH_POSE_DEG),
                  bg=PANEL, padx=12, pady=7, font=FONT_BODY
                  ).pack(fill='x', pady=2)
        tk.Label(pad,
                 text='Ciclos completos de pick e entrega estão na aba '
                      'Célula.',
                 font=FONT_SMALL, bg=PANEL, fg=TEXT_MUTED,
                 wraplength=380, justify='left'
                 ).pack(anchor='w', pady=(8, 0))

    # ──────────────────────────────────────────────────────── HAND TAB
    def _build_hand_tab(self, parent):
        parent.columnconfigure(0, weight=3)
        parent.columnconfigure(1, weight=2)
        parent.rowconfigure(0, weight=1)

        # Cartão 1: sliders
        card_s, s_body = _card(parent, 'Dedos (0 = aberto · 200 = fechado)')
        card_s.grid(row=0, column=0, sticky='nsew', padx=(0, 8))

        body = tk.Frame(s_body, bg=PANEL, padx=14, pady=8)
        body.pack(fill='both', expand=True)

        self.hand_sliders: dict[str, tk.Scale] = {}
        self.hand_value_vars: dict[str, tk.StringVar] = {}
        self.hand_entries: dict[str, tk.Entry] = {}
        for i, j in enumerate(HAND_JOINTS):
            row_bg = SURFACE if i % 2 == 0 else PANEL
            row = tk.Frame(body, bg=row_bg)
            row.pack(fill='x', ipady=6)

            tk.Label(row, text=j.upper(), font=FONT_H,
                     bg=row_bg, fg=ACCENT_HND, width=8, anchor='w',
                     padx=8).pack(side='left')

            var = tk.StringVar(value='  0')
            self.hand_value_vars[j] = var
            ent = tk.Entry(row, textvariable=var, font=FONT_MONO,
                           bg=row_bg, fg=VALUE_FG, width=5,
                           justify='right', relief='flat', bd=0,
                           highlightthickness=1,
                           highlightbackground=row_bg,
                           highlightcolor=PRIMARY,
                           insertbackground=VALUE_FG)
            ent.pack(side='right', padx=8, ipady=2)
            ent.bind('<Return>',
                     lambda e, jn=j: (self._hand_entry_commit(jn),
                                       e.widget.master.focus_set()))
            ent.bind('<FocusOut>',
                     lambda e, jn=j: self._hand_entry_commit(jn))
            ent.bind('<Escape>',
                     lambda e, jn=j: (self._hand_entry_restore(jn),
                                       e.widget.master.focus_set()))
            self.hand_entries[j] = ent

            tk.Label(row, text='200', font=FONT_SMALL, bg=row_bg,
                     fg=TEXT_MUTED, width=4).pack(side='right')

            sl = tk.Scale(row, from_=0, to=200, resolution=1,
                          orient='horizontal', showvalue=False,
                          bg=row_bg, troughcolor=TROUGH,
                          activebackground=ACCENT_HND, fg=row_bg,
                          highlightthickness=0, sliderrelief='flat',
                          sliderlength=18, width=10,
                          command=lambda v, jn=j:
                                  self._hand_slider_changed(v, jn))
            sl.set(0)
            sl.pack(side='left', fill='x', expand=True, padx=(4, 4))

            tk.Label(row, text='0', font=FONT_SMALL, bg=row_bg,
                     fg=TEXT_MUTED, width=4).pack(side='left')
            self.hand_sliders[j] = sl

        # Acesso rápido
        quick = tk.Frame(body, bg=PANEL, pady=10)
        quick.pack(fill='x')
        for label, v in (('Abrir tudo', 0), ('Meio', 100), ('Fechar tudo', 200)):
            _flat_btn(quick, label,
                      command=lambda val=v: self._apply_hand_uniform(val),
                      bg=PANEL, padx=12, pady=6, font=FONT_BODY
                      ).pack(side='left', padx=4, fill='x', expand=True)

        # Cartão 2: grips
        card_g, g_body = _card(parent, 'Preensões')
        card_g.grid(row=0, column=1, sticky='nsew', padx=(8, 0))

        pad = tk.Frame(g_body, bg=PANEL, padx=14, pady=10)
        pad.pack(fill='both', expand=True)

        tk.Label(pad, text='PROJETO', font=FONT_SMALL,
                 bg=PANEL, fg=TEXT_MUTED).pack(anchor='w', pady=(0, 4))
        grip_colors = {
            'Palm Grip (frasco)':      COLOR_FRASCO,
            'Claw Grip (tubo)':        COLOR_TUBO,
            'Fingertip Grip (ampola)': COLOR_AMPOLA,
        }
        for label in ('Palm Grip (frasco)', 'Claw Grip (tubo)',
                      'Fingertip Grip (ampola)'):
            clr = grip_colors[label]
            b = tk.Button(pad, text=label, bg=clr, fg='white',
                          activebackground=_shade(clr, -0.15),
                          activeforeground='white',
                          relief='flat', bd=0, padx=12, pady=7,
                          font=FONT_BODY, cursor='hand2',
                          command=lambda v=HAND_GRIPS[label], l=label:
                                  self._smart_close(v, project_label=l, label=l))
            b.pack(fill='x', pady=2)

        tk.Label(pad, text='COVVI BUILT-IN (ECI)', font=FONT_SMALL,
                 bg=PANEL, fg=TEXT_MUTED).pack(anchor='w', pady=(14, 4))

        grid = tk.Frame(pad, bg=PANEL)
        grid.pack(fill='x')
        names = list(ECI_GRIP_IDS.keys())
        for col in range(2):
            grid.columnconfigure(col, weight=1)
        for idx, name in enumerate(names):
            r, c = divmod(idx, 2)
            b = _flat_btn(grid, name,
                          command=lambda n=name: self._apply_eci_grip(n),
                          bg=PANEL, padx=8, pady=6, font=FONT_LBL)
            b.grid(row=r, column=c, sticky='ew', padx=2, pady=2)

        # Feedback ECI
        fb_outer, fb = _card(pad, None)
        fb_outer.pack(fill='x', pady=(14, 0))
        inner = tk.Frame(fb, bg=PANEL, padx=10, pady=8)
        inner.pack(fill='x')
        tk.Label(inner, text='ECI FEEDBACK', font=FONT_SMALL,
                 bg=PANEL, fg=TEXT_MUTED).pack(anchor='w')

        def _kv(label):
            row = tk.Frame(inner, bg=PANEL)
            row.pack(fill='x', pady=1)
            tk.Label(row, text=label, font=FONT_SMALL, bg=PANEL,
                     fg=TEXT_DIM, width=12, anchor='w').pack(side='left')
            v = tk.Label(row, text='—', font=FONT_MONO_S, bg=PANEL,
                         fg=VALUE_FG, anchor='w')
            v.pack(side='left', fill='x', expand=True)
            return v

        self._eci_grip_lbl = _kv('grip atual')
        self._eci_sent_lbl = _kv('último env.')
        self._eci_posn_lbl = _kv('posições')
        self._eci_posn_lbl.config(text='T:- I:- M:- R:- L:- Rot:-')

    # ──────────────────────────────────────────────────────── CELL TAB
    def _build_cell_tab(self, parent):
        parent.columnconfigure(0, weight=1)
        parent.columnconfigure(1, weight=1)
        parent.rowconfigure(0, weight=0)
        parent.rowconfigure(1, weight=1)

        # Cartão 1: spawn
        card_sp, sp_body = _card(parent, 'Esteira — spawn de objetos')
        card_sp.grid(row=0, column=0, sticky='nsew', padx=(0, 8), pady=(0, 8))

        pad = tk.Frame(sp_body, bg=PANEL, padx=14, pady=12)
        pad.pack(fill='both', expand=True)

        tk.Label(pad, text='Coloca um objeto novo na estação de pick.',
                 font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM
                 ).pack(anchor='w', pady=(0, 8))

        row = tk.Frame(pad, bg=PANEL)
        row.pack(fill='x', pady=(0, 10))
        for label, color, srv_key in [
            ('Frasco', COLOR_FRASCO, 'spawn_frasco'),
            ('Tubo',   COLOR_TUBO,   'spawn_tubo'),
            ('Ampola', COLOR_AMPOLA, 'spawn_ampola'),
        ]:
            b = tk.Button(
                row, text=label, bg=color, fg='white',
                activebackground=_shade(color, -0.15),
                activeforeground='white',
                relief='flat', bd=0, padx=12, pady=7,
                font=FONT_BODY, cursor='hand2',
                command=lambda k=srv_key, l=label:
                        self._call_conveyor(k, f'Spawnando {l}…'))
            b.pack(side='left', padx=3, fill='x', expand=True)

        tk.Frame(pad, bg=BORDER, height=1).pack(fill='x', pady=8)

        row2 = tk.Frame(pad, bg=PANEL)
        row2.pack(fill='x')
        _flat_btn(row2, 'Remover objeto',
                  command=lambda: self._call_conveyor(
                      'retreat', 'Removendo objeto…'),
                  bg=PANEL, padx=10, pady=6, font=FONT_BODY
                  ).pack(side='left', padx=2, fill='x', expand=True)
        _flat_btn(row2, 'Resetar esteira',
                  command=lambda: self._call_conveyor(
                      'reset', 'Resetando esteira…'),
                  bg=PANEL, padx=10, pady=6, font=FONT_BODY
                  ).pack(side='left', padx=2, fill='x', expand=True)

        # Cartão 2: ciclo de pick + entrega
        card_r, r_body = _card(parent, 'Ciclo completo de pick e entrega')
        card_r.grid(row=0, column=1, sticky='nsew', padx=(8, 0), pady=(0, 8))

        pad2 = tk.Frame(r_body, bg=PANEL, padx=14, pady=12)
        pad2.pack(fill='both', expand=True)

        tk.Label(pad2,
                 text='1) Aproximação vertical · 2) Descida até o objeto · '
                      '3) Fechamento da mão · 4) Subida com o objeto.',
                 font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM,
                 justify='left', wraplength=380
                 ).pack(anchor='w', pady=(0, 10))

        for obj in ('frasco', 'tubo', 'ampola'):
            clr = {'frasco': COLOR_FRASCO, 'tubo': COLOR_TUBO,
                   'ampola': COLOR_AMPOLA}[obj]
            b = tk.Button(
                pad2, text=f'Pick: {obj.capitalize()}',
                bg=clr, fg='white',
                activebackground=_shade(clr, -0.15),
                activeforeground='white',
                relief='flat', bd=0, padx=12, pady=8,
                font=FONT_BODY, cursor='hand2',
                command=lambda o=obj: self._do_pick_cycle(o))
            b.pack(fill='x', pady=3)

        tk.Frame(pad2, bg=BORDER, height=1).pack(fill='x', pady=10)

        tk.Label(pad2, text='ENTREGA', font=FONT_SMALL,
                 bg=PANEL, fg=TEXT_MUTED).pack(anchor='w', pady=(0, 4))
        self._btn_entrega = tk.Button(
            pad2, text='Entregar na caixa correspondente',
            bg=PRIMARY, fg='white',
            activebackground=PRIMARY_HV, activeforeground='white',
            relief='flat', bd=0, padx=12, pady=8,
            font=FONT_BODY, cursor='hand2',
            command=self._do_delivery)
        self._btn_entrega.pack(fill='x', pady=3)
        self._entrega_hint = tk.Label(
            pad2, text='Aguardando detecção da câmera…',
            font=FONT_SMALL, bg=PANEL, fg=TEXT_MUTED,
            anchor='w', justify='left')
        self._entrega_hint.pack(fill='x', pady=(4, 0))

        # Cartão 3: monitor
        card_m, m_body = _card(parent, 'Monitor da célula')
        card_m.grid(row=1, column=0, columnspan=2, sticky='nsew')

        pad3 = tk.Frame(m_body, bg=PANEL, padx=14, pady=12)
        pad3.pack(fill='both', expand=True)

        def _kv(parent, k):
            row = tk.Frame(parent, bg=PANEL)
            row.pack(fill='x', pady=2)
            tk.Label(row, text=k, font=FONT_LBL, bg=PANEL,
                     fg=TEXT_DIM, width=18, anchor='w').pack(side='left')
            v = tk.Label(row, text='—', font=FONT_MONO, bg=PANEL,
                         fg=VALUE_FG, anchor='w')
            v.pack(side='left', fill='x', expand=True)
            return v

        self._mon_belt = _kv(pad3, 'Esteira (atual)')
        self._mon_det  = _kv(pad3, 'Detecção câmera')
        self._mon_eci  = _kv(pad3, 'ECI grip ativo')

    # ──────────────────────────────────────────────────────── STATUS BAR
    def _build_statusbar(self):
        sb = tk.Frame(self.root, bg=PANEL, height=28)
        sb.pack(fill='x', side='bottom')
        tk.Frame(sb, bg=BORDER, height=1).pack(fill='x', side='top')
        self.status_var = tk.StringVar(value='Pronto.')
        self.status_lbl = tk.Label(sb, textvariable=self.status_var,
                                    font=FONT_LBL, bg=PANEL,
                                    fg=TEXT, anchor='w', padx=14, pady=4)
        self.status_lbl.pack(fill='x')

    # ──────────────────────────────────────────────────────── HANDLERS
    @staticmethod
    def _fmt_arm(deg: int) -> str:
        return f'{int(deg):+4d}°'

    @staticmethod
    def _fmt_hand(v: int) -> str:
        return f'{int(v):3d}'

    def _arm_slider_changed(self, val, j):
        self.arm_value_vars[j].set(self._fmt_arm(int(float(val))))
        if not self._suppressing:
            self._publish_arm()

    def _arm_entry_commit(self, j):
        lo, hi = ARM_LIMITS_DEG[j]
        raw = self.arm_value_vars[j].get().strip().replace('°', '').strip()
        try:
            val = int(round(float(raw)))
        except (ValueError, TypeError):
            val = int(self.arm_sliders[j].get())
        val = max(lo, min(hi, val))
        self.arm_value_vars[j].set(self._fmt_arm(val))
        self._suppressing = True
        try:
            self.arm_sliders[j].set(val)
        finally:
            self._suppressing = False
        self._publish_arm()

    def _arm_entry_restore(self, j):
        self.arm_value_vars[j].set(
            self._fmt_arm(int(self.arm_sliders[j].get())))

    def _hand_slider_changed(self, val, j):
        self.hand_value_vars[j].set(self._fmt_hand(int(float(val))))
        if not self._suppressing:
            self._publish_hand()

    def _hand_entry_commit(self, j):
        raw = self.hand_value_vars[j].get().strip()
        try:
            val = int(round(float(raw)))
        except (ValueError, TypeError):
            val = int(self.hand_sliders[j].get())
        val = max(0, min(200, val))
        self.hand_value_vars[j].set(self._fmt_hand(val))
        self._suppressing = True
        try:
            self.hand_sliders[j].set(val)
        finally:
            self._suppressing = False
        self._publish_hand()
        self._schedule_eci_posn({jn: self.hand_sliders[jn].get()
                                  for jn in HAND_JOINTS})

    def _hand_entry_restore(self, j):
        self.hand_value_vars[j].set(
            self._fmt_hand(int(self.hand_sliders[j].get())))

    def _apply_arm(self, preset_deg: dict):
        positions_rad = [math.radians(preset_deg[j]) for j in ARM_JOINTS]
        self._publish_arm_positions(positions_rad)
        self.root.after_idle(self._update_arm_sliders, preset_deg)

    def _update_arm_sliders(self, preset_deg: dict):
        self._suppressing = True
        try:
            for j, deg in preset_deg.items():
                self.arm_sliders[j].set(deg)
                self.arm_value_vars[j].set(self._fmt_arm(int(deg)))
        finally:
            self._suppressing = False

    def _apply_hand_uniform(self, value: int):
        vals = {j: value for j in HAND_JOINTS}
        self._publish_hand_values(vals)
        self._schedule_eci_posn(vals)
        self.root.after_idle(self._update_hand_sliders, vals)

    def _apply_hand(self, vals: dict, project_label: str | None = None):
        """Aplicação direta (sem fechamento incremental). Usada por presets
        de abertura, recipes que já usam atraso, etc."""
        self._publish_hand_values(vals)
        self.root.after_idle(self._update_hand_sliders, dict(vals))
        if project_label and self._eci_enabled:
            eci_id = PROJECT_GRIP_ECI.get(project_label)
            if eci_id is not None:
                eci_name = ECI_GRIP_NAMES.get(eci_id, str(eci_id))
                self._send_eci_grip(eci_id, f'{project_label} > {eci_name}')

    def _apply_eci_grip(self, name: str):
        """Grips nativos COVVI usam o controlador interno do ECI no real
        (já tem detecção de stall por corrente). No sim, dispara fechamento
        incremental para a aproximação Gazebo equivalente."""
        gazebo_vals = ECI_GRIP_GAZEBO[name]
        self._send_eci_grip(ECI_GRIP_IDS[name], name)
        self._smart_close(gazebo_vals, label=name)
        self._set_status(f'ECI grip: {name} (fechamento incremental)', _CLR=OK)

    # ──────────────────────────────────────────────────────── SMART CLOSE
    def _max_sensor_value(self) -> int:
        """Valor máximo de toque/corrente disponível para detecção de contato."""
        if self._eci_enabled:
            t = max(self._touch_values.values()) if self._touch_values else 0
            c = max(self._current_values.values()) if self._current_values else 0
            return max(t, c)
        # Sim com FK real dos dedos: se há objeto na esteira, calcula a
        # distância de cada ponta ao centro do AABB do objeto. Quando uma
        # ponta entra no AABB, declara força saturada (200/255).
        obj_bbox = self._current_object_bbox()
        if obj_bbox is not None and _COLLISION_OK:
            q_arm = _np.array([
                math.radians(self.arm_sliders[j].get()) for j in ARM_JOINTS])
            hand_state = self._current_hand_state()
            tips = _fingertips_world(q_arm, hand_state)
            cx, cy, cz, sx, sy, sz = obj_bbox
            # Cada ponta: penalty pela "profundidade negativa" dentro do AABB
            # (clearance < 0 = dentro). Saturar em 200 quando dx<-2cm em
            # qualquer eixo (penetração profunda).
            max_force = 0
            for finger, p in tips.items():
                dx = abs(p[0] - cx) - sx / 2
                dy = abs(p[1] - cy) - sy / 2
                dz = abs(p[2] - cz) - sz / 2
                # Distância máxima ao envelope (negativa = dentro)
                outside = max(dx, dy, dz)
                if outside < 0.01:  # < 1 cm do envelope = contato iminente
                    # Mapa: outside=+1cm→0, outside=0→100, outside=−2cm→255
                    f = int(max(0, min(255, (0.01 - outside) * 100 / 0.01 + 50)))
                    if f > max_force:
                        max_force = f
            return max_force
        # Fallback: proxy do comando quando não há FK / sem objeto
        if self._conveyor_state.get('has_object'):
            positions = [self.hand_sliders[j].get()
                         for j in ('Thumb', 'Index', 'Middle', 'Ring', 'Little')]
            max_pos = max(positions) if positions else 0
            if max_pos > 60:
                return max(0, int((max_pos - 60) * 2.5))
        return 0

    def _smart_close(self, target_vals: dict, *,
                     n_steps: int = 12, step_ms: int = 80,
                     touch_threshold: int = 70,
                     label: str | None = None,
                     project_label: str | None = None):
        """Fechamento incremental com parada em contato.

        n_steps:          número de passos da rampa
        step_ms:          intervalo entre passos (ms)
        touch_threshold:  para o fechamento quando algum dedo ultrapassa este
                          valor de sensor (0-255). Aplicado a partir do passo 3
                          para evitar parar antes mesmo de o dedo se mover.
        project_label:    se setado, dispara o grip nativo ECI em paralelo.
        """
        start = {j: self.hand_sliders[j].get() for j in HAND_JOINTS}

        if project_label and self._eci_enabled:
            eci_id = PROJECT_GRIP_ECI.get(project_label)
            if eci_id is not None:
                eci_name = ECI_GRIP_NAMES.get(eci_id, str(eci_id))
                self._send_eci_grip(eci_id, f'{project_label} > {eci_name}')

        # Cancela qualquer fechamento em andamento
        if getattr(self, '_smart_after', None) is not None:
            try:
                self.root.after_cancel(self._smart_after)
            except Exception:
                pass
        self._smart_after = None
        self._smart_stopped_label: str | None = label

        self._smart_close_step(start, dict(target_vals),
                               1, n_steps, step_ms, touch_threshold)

    def _smart_close_step(self, start: dict, target: dict,
                           step: int, n_steps: int, step_ms: int,
                           threshold: int):
        # Detecção de contato — só após dar tempo de o dedo se mover
        if step > 3:
            sensor = self._max_sensor_value()
            if sensor >= threshold:
                self._set_status(
                    f'Contato detectado (força={sensor}) — fechamento parou '
                    f'em {int(100 * step / n_steps)}%.',
                    _CLR=OK)
                self._smart_after = None
                return

        if step > n_steps:
            self._set_status('Fechamento completo (sem contato detectado).',
                              _CLR=TEXT_DIM)
            self._smart_after = None
            return

        alpha = step / n_steps
        vals = {j: int(round(start[j] + alpha * (target[j] - start[j])))
                for j in HAND_JOINTS}
        self._publish_hand_values(vals)
        self._schedule_eci_posn(vals)
        self.root.after_idle(self._update_hand_sliders, dict(vals))

        self._smart_after = self.root.after(
            step_ms,
            lambda: self._smart_close_step(start, target, step + 1,
                                            n_steps, step_ms, threshold))

    def _update_hand_sliders(self, vals: dict):
        self._suppressing = True
        try:
            for j, v in vals.items():
                self.hand_sliders[j].set(v)
                self.hand_value_vars[j].set(self._fmt_hand(int(v)))
        finally:
            self._suppressing = False

    # ──────────────────────────────────────────────────────── PICK CYCLE
    def _cancel_pick_cycle(self):
        after_id = getattr(self, '_pick_cycle_after', None)
        if after_id is not None:
            try:
                self.root.after_cancel(after_id)
            except Exception:
                pass
        self._pick_cycle_after = None

    def _do_pick_cycle(self, obj_class: str):
        """Ciclo completo de pick em 4 fases:
          1. Mão abre + braço vai para approach (TCP em (0.75, 0, 1.05))
          2. Braço desce verticalmente até a pose de grasp do objeto
          3. Mão fecha na preensão correta (smart_close com detecção de toque)
          4. Braço sobe de volta à approach (com o objeto)
        """
        if obj_class not in PICK_POSES_DEG:
            self._set_status(f'Objeto desconhecido: {obj_class}',
                              _CLR=DANGER)
            return
        grip_key = OBJ_GRIP[obj_class]
        approach_pose = APPROACH_POSE_DEG
        grasp_pose    = PICK_POSES_DEG[obj_class]
        grip_target   = HAND_GRIPS[grip_key]

        move_ms     = int(float(self.time_sl.get()) * 1000) + 250
        descend_ms  = max(1200, move_ms)
        close_ms    = 12 * 80 + 400   # smart_close n_steps*step_ms + buffer

        self._cancel_pick_cycle()

        # Fase 1: abre a mão (sincrono) e inicia approach do braço
        self._apply_hand(HAND_OPEN)
        self._apply_arm(approach_pose)
        self._set_status(
            f'[1/4] {obj_class}: mão aberta, aproximando…', _CLR=WARN)

        def _phase_descend():
            self._set_status(
                f'[2/4] {obj_class}: descendo até a pose de grasp…',
                _CLR=WARN)
            self._apply_arm(grasp_pose)
            self._pick_cycle_after = self.root.after(
                descend_ms, _phase_close)

        def _phase_close():
            self._set_status(
                f'[3/4] {obj_class}: fechando {grip_key}…', _CLR=WARN)
            self._smart_close(grip_target, label=grip_key,
                              project_label=grip_key)
            self._pick_cycle_after = self.root.after(
                close_ms, _phase_lift)

        def _phase_lift():
            self._set_status(
                f'[4/4] {obj_class}: subindo com o objeto…', _CLR=WARN)
            self._apply_arm(approach_pose)
            self._pick_cycle_after = self.root.after(
                move_ms, _phase_done)

        def _phase_done():
            self._pick_cycle_after = None
            self._set_status(
                f'Pick completo: {obj_class} ({grip_key}).', _CLR=OK)

        self._pick_cycle_after = self.root.after(move_ms, _phase_descend)

    def _do_delivery(self):
        obj = self._last_detection
        if obj is None or obj not in DELIVERY_POSES_DEG:
            self._set_status(
                'Nenhum objeto detectado — spawne antes de entregar.',
                _CLR=DANGER)
            return
        box_color = {'frasco': 'vermelha', 'tubo': 'verde',
                     'ampola': 'azul'}[obj]
        move_ms = int(float(self.time_sl.get()) * 1000) + 250

        self._cancel_pick_cycle()
        self._set_status(
            f'[1/3] Levando {obj} → caixa {box_color}…', _CLR=WARN)
        self._apply_arm(DELIVERY_POSES_DEG[obj])

        def _phase_release():
            self._set_status(
                f'[2/3] Liberando {obj} na caixa {box_color}.',
                _CLR=WARN)
            self._apply_hand(HAND_OPEN)
            self._pick_cycle_after = self.root.after(
                900, _phase_retreat)

        def _phase_retreat():
            self._set_status(
                '[3/3] Retornando à aproximação…', _CLR=WARN)
            self._apply_arm(APPROACH_POSE_DEG)
            self._pick_cycle_after = self.root.after(
                move_ms, _phase_done)

        def _phase_done():
            self._pick_cycle_after = None
            self._set_status(
                f'Entrega completa: {obj} → caixa {box_color}.',
                _CLR=OK)

        self._pick_cycle_after = self.root.after(move_ms, _phase_release)

    # ──────────────────────────────────────────────────────── PUBLISH
    def _publish_arm(self):
        if not self._ready:
            return
        positions = [math.radians(self.arm_sliders[j].get()) for j in ARM_JOINTS]
        self._publish_arm_positions(positions)

    # ───────── Sistema de colisão (braço+mão+dedos vs cenário) ─────────
    def _current_object_bbox(self):
        """AABB world do objeto atualmente na pick station, ou None."""
        if not _COLLISION_OK:
            return None
        obj = self._conveyor_state.get('current_obj')
        if not obj or not self._conveyor_state.get('has_object'):
            return None
        return _GX_PICK_BBOX.get(obj)

    def _current_hand_state(self) -> dict:
        """Estado atual da mão (sliders) convertido para ângulos primários (rad)."""
        if not hasattr(self, 'hand_sliders'):
            return {j: 0.0 for j in HAND_JOINTS}
        return {j: float(self.hand_sliders[j].get()) / 200.0 *
                   _HAND_DRIVER_MAX[j]
                for j in HAND_JOINTS}

    def _path_max_safe_alpha(self, q_start, q_end, n: int = 20) -> float:
        """Maior alpha em [0,1] sobre o segmento `q_start→q_end` que ainda
        está livre de colisão. Verifica:
          • links 4-6 do braço vs _WORLD_OBSTACLES
          • envelope da mão COVVI (palma + dedos) vs _WORLD_OBSTACLES
          • cada ponta de dedo (FK por dedo) vs _WORLD_OBSTACLES
        """
        if not _COLLISION_OK:
            return 1.0
        hand_state = self._current_hand_state()
        for i in range(1, n + 1):
            alpha = i / n
            q_i = q_start + alpha * (q_end - q_start)
            ok_arm, _ = _gx_arm_clears_world(
                q_i, links=(4, 5, 6), margin=0.020)
            ok_hand_env = _hand_clears_world(
                q_i, hand_state=hand_state, margin=0.020)
            ok_fingers, _ = _fingers_clear_objects(
                q_i, hand_state, object_bbox=None,
                check_world=True, margin=0.010)
            if not (ok_arm and ok_hand_env and ok_fingers):
                return max(0.0, (i - 1) / n - 0.02)
        return 1.0

    def _collision_safe_arm(self, target_rad):
        """Retorna (q_seguro, foi_bloqueado). Encurta o movimento se
        necessário para que o braço pare antes da parede/obstáculo."""
        if not _COLLISION_OK:
            return target_rad, False
        start = self._last_safe_q
        if start is None:
            start = _np.array(target_rad)
        target = _np.array(target_rad)
        alpha = self._path_max_safe_alpha(start, target)
        if alpha >= 0.999:
            return target, False
        if alpha <= 0.001:
            # Mesmo um pequeno passo colide — fica onde está
            return start, True
        safe = start + alpha * (target - start)
        return safe, True

    def _publish_arm_positions(self, positions_rad: list):
        if not self._ready:
            return

        if _COLLISION_OK:
            safe_q, blocked = self._collision_safe_arm(positions_rad)
            out_positions = [float(v) for v in safe_q]
            self._last_safe_q = _np.array(out_positions)
            if blocked:
                # Sincroniza sliders/entries com a posição realmente publicada
                safe_deg = {j: int(round(math.degrees(out_positions[i])))
                            for i, j in enumerate(ARM_JOINTS)}
                self.root.after_idle(self._update_arm_sliders, safe_deg)
                self._set_status(
                    'Colisão prevista — braço parado antes do obstáculo.',
                    _CLR=WARN)
        else:
            out_positions = list(positions_rad)

        msg = JointTrajectory()
        msg.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = out_positions
        dur = float(self.time_sl.get())
        pt.time_from_start = Duration(sec=int(dur),
                                       nanosec=int((dur - int(dur)) * 1e9))
        msg.points.append(pt)
        self.arm_pub.publish(msg)

    def _publish_hand(self):
        if not self._ready:
            return
        vals = {j: self.hand_sliders[j].get() for j in HAND_JOINTS}
        self._publish_hand_values(vals)
        self._schedule_eci_posn(vals)

    def _publish_hand_values(self, vals: dict):
        if not self._ready:
            return
        primary_rad = {j: vals[j] / 200.0 * MAX_RAD[j] for j in HAND_JOINTS}
        names = list(HAND_JOINTS)
        positions = [primary_rad[j] for j in HAND_JOINTS]
        for mimic, driver, mult in MIMIC_JOINTS:
            names.append(mimic)
            positions.append(primary_rad[driver] * mult)

        msg = JointTrajectory()
        msg.joint_names = names
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(sec=0, nanosec=800_000_000)
        msg.points.append(pt)
        self.hand_pub.publish(msg)

    # ──────────────────────────────────────────────────────── STATUS
    def _set_status(self, msg: str, _CLR=TEXT):
        self.status_var.set(msg)
        self.status_lbl.configure(fg=_CLR)

    def _spin_ros(self):
        try:
            rclpy.spin_once(self, timeout_sec=0.0)
        except Exception:
            return
        self.root.after(20, self._spin_ros)

    def _poll_status(self):
        try:
            while True:
                msg, color = self._status_q.get_nowait()
                self._set_status(msg, _CLR=color)
        except queue.Empty:
            pass

        cs = self._conveyor_state
        if cs:
            obj = cs.get('current_obj', 'none')
            has = cs.get('has_object', False)
            if hasattr(self, '_led_belt'):
                self._led_belt.set_state(OK if has else TEXT_MUTED)
                self._lbl_belt.config(text=f'{obj}' if has else 'vazia')
            if hasattr(self, '_mon_belt'):
                self._mon_belt.config(text=obj if has else 'vazia')

        det = self._last_detection
        if hasattr(self, '_entrega_hint'):
            if det in DELIVERY_POSES_DEG:
                box_color = {'frasco': 'vermelha', 'tubo': 'verde',
                             'ampola': 'azul'}[det]
                clr = {'frasco': COLOR_FRASCO, 'tubo': COLOR_TUBO,
                       'ampola': COLOR_AMPOLA}[det]
                self._entrega_hint.config(
                    text=f'Pronto para entregar {det} na caixa {box_color}.',
                    fg=clr)
                self._btn_entrega.config(state='normal')
            else:
                self._entrega_hint.config(
                    text='Aguardando detecção da câmera…', fg=TEXT_MUTED)
                self._btn_entrega.config(state='disabled')
        if hasattr(self, '_mon_det'):
            self._mon_det.config(text=det if det else 'nenhum')

        if hasattr(self, '_eci_grip_lbl'):
            grip_id = self._eci_grip_id
            grip_name = (ECI_GRIP_NAMES.get(grip_id, f'id={grip_id}')
                         if grip_id else '—')
            self._eci_grip_lbl.config(text=grip_name)
            if hasattr(self, '_mon_eci'):
                self._mon_eci.config(text=grip_name)

            sent = self._eci_last_sent or '—'
            self._eci_sent_lbl.config(text=sent)

            p = self._eci_posn
            if p:
                posn_str = (
                    f"T:{p.get('Thumb','-'):3}  I:{p.get('Index','-'):3}  "
                    f"M:{p.get('Middle','-'):3}  R:{p.get('Ring','-'):3}  "
                    f"L:{p.get('Little','-'):3}  Rot:{p.get('Rotate','-'):3}"
                )
                self._eci_posn_lbl.config(text=posn_str)

        # ── Atualização do bargraph de forças ────────────────────────
        if hasattr(self, '_force_widgets'):
            sim_tips = None
            obj_bbox = None
            if (not self._eci_enabled) and _COLLISION_OK \
                    and hasattr(self, 'hand_sliders'):
                obj_bbox = self._current_object_bbox()
                if obj_bbox is not None:
                    q_arm = _np.array([
                        math.radians(self.arm_sliders[j].get())
                        for j in ARM_JOINTS])
                    sim_tips = _fingertips_world(q_arm,
                                                  self._current_hand_state())

            for finger in ('Thumb', 'Index', 'Middle', 'Ring', 'Little'):
                if self._eci_enabled:
                    t = self._touch_values.get(finger, 0)
                    c = self._current_values.get(finger, 0)
                    val = max(t, c)
                elif sim_tips is not None and finger in sim_tips and obj_bbox:
                    # Sim com FK: força do dedo ∝ proximidade/penetração no AABB
                    p = sim_tips[finger]
                    cx, cy, cz, sx, sy, sz = obj_bbox
                    dx = abs(p[0] - cx) - sx / 2
                    dy = abs(p[1] - cy) - sy / 2
                    dz = abs(p[2] - cz) - sz / 2
                    outside = max(dx, dy, dz)
                    if outside < 0.02:
                        val = int(max(0, min(255,
                                  (0.02 - outside) * 100 / 0.01)))
                    else:
                        val = 0
                else:
                    # Sem objeto / colisão indisponível: comando como proxy
                    cmd = (self.hand_sliders[finger].get()
                           if hasattr(self, 'hand_sliders')
                           and finger in self.hand_sliders else 0)
                    val = int(cmd * 255 / 200)
                self._update_force_bar(finger, val)

        self.root.after(400, self._poll_status)


def main(args=None):
    # tk.Tk() MUST be called before rclpy.init() — rclpy starts FastDDS
    # threads cujas C-extensions corrompem o alocador do Tcl se Tcl for
    # iniciado depois.
    root = tk.Tk()
    root.withdraw()

    rclpy.init(args=args)
    node = ManualControlNode(root=root)
    try:
        root.mainloop()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
