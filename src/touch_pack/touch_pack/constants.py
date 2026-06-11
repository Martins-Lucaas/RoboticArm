"""
constants.py — Constantes compartilhadas do touch_pack.

Fonte única para valores que antes viviam duplicados na GUI, no explorer,
no logger e nos nós auxiliares ("atualizei um lado só" era uma classe de
bug real: o limite de 15 N e a pose POINTING existiam em 2 cópias).

Regra: valores usados por MAIS de um módulo moram aqui; valores privados
de um único módulo ficam nele.
"""
from __future__ import annotations

import math
import os

# ── Cadeia do braço CR10 (convenção URDF) ────────────────────────────────────
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# Pose "apontar para a mesa": braço vertical com o efetuador perpendicular.
# É a home default da GUI e o seed POINTING do explorer.
POINTING_SEED_DEG = {'joint1': 0.0, 'joint2': 0.0, 'joint3': -90.0,
                     'joint4': 0.0, 'joint5': 90.0, 'joint6': 0.0}

# ── Mão COVVI — juntas primárias ─────────────────────────────────────────────
HAND_JOINTS = ['Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Rotate']

# Pose POINTING (palpação com o Index estendido).
HAND_POINT_DEG = {'Thumb': 30.0, 'Index': 0.0, 'Middle': 80.0,
                  'Ring': 80.0, 'Little': 80.0, 'Rotate': 0.0}
HAND_POINTING_RAD = {j: math.radians(v) for j, v in HAND_POINT_DEG.items()}

# ── Controle de força ────────────────────────────────────────────────────────
# Limite de segurança: a medição é CANCELADA se a compressão exceder este
# valor (explorer aborta; GUI exibe DANGER ao se aproximar).
FORCE_ABORT_LIMIT_N = 15.0
# Setpoint máximo selecionável na GUI.
FORCE_SETPOINT_MAX_N = 10.0

# ── Célula de carga (ESP32 via UDP) ──────────────────────────────────────────
LOAD_CELL_UDP_PORT = 8080

# ── Arquivos de configuração persistente (~/.config/touch_pack/) ─────────────
CONFIG_DIR            = os.path.expanduser('~/.config/touch_pack')
HOME_POSE_FILE        = os.path.join(CONFIG_DIR, 'home_pose.json')
ROBOT_CONFIG_FILE     = os.path.join(CONFIG_DIR, 'robot.json')
LC_CALIB_FILE         = os.path.join(CONFIG_DIR, 'load_cell_calib.json')
POSES_FILE            = os.path.join(CONFIG_DIR, 'poses.json')
PALPATION_PARAMS_FILE = os.path.join(CONFIG_DIR, 'palpation_params.json')

# ── Saída dos runs ───────────────────────────────────────────────────────────
RUNS_DIR = os.path.expanduser('~/touch_pack_runs')
