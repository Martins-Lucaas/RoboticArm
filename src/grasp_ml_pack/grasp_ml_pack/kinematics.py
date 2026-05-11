"""
Módulo centralizado de cinemática — CR10 + COVVI Hand.

Convenção URDF (ROS 2 / Gazebo):
    T_joint = T_origin × Rz(q)
    onde T_origin = Translation(xyz) × R_rpy(roll, pitch, yaw)

Parâmetros extraídos do URDF cr10_robot.xacro:
    joint1: xyz=[0, 0, 0.1765]   rpy=[0, 0, 0]
    joint2: xyz=[0, 0, 0]        rpy=[π/2, π/2, 0]
    joint3: xyz=[-0.607, 0, 0]   rpy=[0, 0, 0]
    joint4: xyz=[-0.568, 0, 0.191] rpy=[0, 0, -π/2]
    joint5: xyz=[0, -0.125, 0]   rpy=[π/2, 0, 0]
    joint6: xyz=[0, 0.1084, 0]   rpy=[-π/2, 0, 0]

Os ângulos de junta usados aqui são exatamente os ângulos URDF que o
controlador ros2_control / Gazebo recebe.

API pública:
    forward_kinematics(q)           → T 4×4
    fk_partial(q, n)                → T 4×4 (links 0..n-1)
    jacobian(q)                     → J 6×6
    manipulability(q)               → float ∈ [0, ∞)
    inverse_kinematics(p, av)       → (q, ok)
    reach_margin(q)                 → float ∈ [0, 1]
    singularity_distances(q)        → (shoulder, elbow, wrist)
    finger_fk(driver_angle, ...)    → [x, 0, z] ponta do dedo
    hand_ik(grasp_type, diameter)   → dict de driver joints
    approach_to_Rtcp(av)            → R 3×3
"""

from __future__ import annotations

import math
import numpy as np

_PI2 = math.pi / 2

# ──────────────────────────────────────────────────────────────────────
# Geometria do braço (constantes físicas em metros)
# ──────────────────────────────────────────────────────────────────────
_D1 = 0.1765   # base → joint1 height
_A2 = 0.6070   # joint2 → joint3 link length
_A3 = 0.5680   # joint3 → joint4 link length

# Transformação fixa flange → ponto de preensão (TCP efetivo) na palma COVVI.
#
# Acoplamento URDF Link6 → hand_base_link: xyz="0 0 0.01" rpy="π/2 0 0".
# Em configuração palm-down (q_pick padrão), Link6_y aponta para +world_Z.
#
# A correção crítica: o TCP DEVE coincidir com a região onde os dedos fecham
# em torno do objeto, não com hand_base_link nem com um ponto fantasma.
# Análise geométrica COVVI (verificada por FK):
#   • hand_base_link world Z ≈ Link6 world Z (acoplamento na lateral)
#   • Index MCP world Z = Link6 world Z + 0.015   (dedo nasce 15 mm acima)
#   • Fingertip fechado world Z = MCP − 0.067     (curl efetivo)
#                              = Link6 world Z − 0.052
#
# Portanto, com TCP definido como `(0, -0.052, 0.01)` no frame do Link6, o
# ponto IK passa pela zona de preensão real. O usuário define `pick_z` como
# o centro do objeto (belt_top + h/2) e os dedos fechados envolvem o objeto.
T_HAND_ATTACH = np.array([
    [1.0,  0.0,  0.0,  0.000],
    [0.0,  0.0, -1.0, -0.075],   # 75 mm — TCP entre palma e fingertips
    [0.0,  1.0,  0.0,  0.010],
    [0.0,  0.0,  0.0,  1.000],
], dtype=float)

# Limites articulares — convenção URDF (rad).
# Joints 2 e 4 têm offset de -π/2 em relação à convenção DH;
# os limites físicos são mapeados de ±135° (DH) → [-5π/4, π/4] (URDF).
JOINT_MIN = np.deg2rad([-180., -260., -135., -260., -135., -360.])
JOINT_MAX = np.deg2rad([ 180.,   80.,  135.,   80.,  135.,  360.])

# Distância efetiva WC→TCP ao longo do vetor de abordagem (m).
# WC→hand_base_link ≈ 0.260; novo offset palma→TCP = 0.075 → soma = 0.335.
# Apenas heurística inicial: o refinamento numérico do IK (DLS) absorve
# diferenças residuais.
_D_WC_TCP = 0.335

# ──────────────────────────────────────────────────────────────────────
# Origens URDF das juntas: (xyz, rpy)
# ──────────────────────────────────────────────────────────────────────
_URDF_ORIGINS = (
    ((0.0000,  0.0000, 0.1765), ( 0.0,   0.0,  0.0 )),  # joint1
    ((0.0000,  0.0000, 0.0000), (_PI2, _PI2,   0.0 )),  # joint2
    ((-0.6070, 0.0000, 0.0000), ( 0.0,   0.0,  0.0 )),  # joint3
    ((-0.5680, 0.0000, 0.1910), ( 0.0,   0.0, -_PI2)),  # joint4
    (( 0.0000,-0.1250, 0.0000), (_PI2,   0.0,  0.0 )),  # joint5
    (( 0.0000, 0.1084, 0.0000), (-_PI2,  0.0,  0.0 )),  # joint6
)

# ──────────────────────────────────────────────────────────────────────
# Parâmetros da mão COVVI
# ──────────────────────────────────────────────────────────────────────
_L_PROX       = 0.0450
_L_DIST       = 0.0300
_K_P_FINGER   = 1.516
_K_D_FINGER   = 0.718
_K_P_THUMB    = 1.400


# ──────────────────────────────────────────────────────────────────────
# Primitivas URDF
# ──────────────────────────────────────────────────────────────────────

def _make_T(xyz: tuple, rpy: tuple) -> np.ndarray:
    """Constrói T 4×4 a partir de origem URDF (xyz, rpy)."""
    x, y, z = xyz
    r, p, yaw = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(yaw), math.sin(yaw)
    R = np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [  -sp,            cp*sr,             cp*cr  ],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = [x, y, z]
    return T


def _Rz4(q: float) -> np.ndarray:
    """Rotação em torno de z como matriz 4×4."""
    c, s = math.cos(q), math.sin(q)
    return np.array([[c, -s, 0., 0.],
                     [s,  c, 0., 0.],
                     [0., 0., 1., 0.],
                     [0., 0., 0., 1.]])


# ──────────────────────────────────────────────────────────────────────
# Cinemática Direta (FK) — convenção URDF
# ──────────────────────────────────────────────────────────────────────

def forward_kinematics(q: np.ndarray,
                       include_hand: bool = True) -> np.ndarray:
    """
    FK completa: base → flange Link6 (opcionalmente até hand_base_link COVVI).

    Args:
        q:            ângulos das juntas URDF (6,) em rad
        include_hand: aplica T_HAND_ATTACH ao resultado se True

    Returns:
        T: pose do efetuador, matriz homogênea 4×4
    """
    T = np.eye(4)
    for (xyz, rpy), qi in zip(_URDF_ORIGINS, q):
        T = T @ _make_T(xyz, rpy) @ _Rz4(float(qi))
    if include_hand:
        T = T @ T_HAND_ATTACH
    return T


def fk_partial(q: np.ndarray, n_links: int) -> np.ndarray:
    """FK dos primeiros n_links elos — ex.: n_links=3 retorna T₀₃."""
    T = np.eye(4)
    for i in range(n_links):
        xyz, rpy = _URDF_ORIGINS[i]
        T = T @ _make_T(xyz, rpy) @ _Rz4(float(q[i]))
    return T


# ──────────────────────────────────────────────────────────────────────
# Jacobiano (diferenças finitas)
# ──────────────────────────────────────────────────────────────────────

def jacobian(q: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """Jacobiano geométrico 6×6 via diferenças finitas."""
    J = np.zeros((6, 6))
    T0 = forward_kinematics(q)
    p0, R0 = T0[:3, 3].copy(), T0[:3, :3].copy()

    for i in range(6):
        dq = q.copy(); dq[i] += eps
        T1 = forward_kinematics(dq)
        J[:3, i] = (T1[:3, 3] - p0) / eps
        dR = (T1[:3, :3] - R0) / eps
        J[3, i] = (dR[2, 1] - dR[1, 2]) / 2.0
        J[4, i] = (dR[0, 2] - dR[2, 0]) / 2.0
        J[5, i] = (dR[1, 0] - dR[0, 1]) / 2.0

    return J


def manipulability(q: np.ndarray) -> float:
    """Índice de manipulabilidade translacional de Yoshikawa."""
    Jv = jacobian(q)[:3, :]
    return float(np.sqrt(max(0.0, np.linalg.det(Jv @ Jv.T))))


# ──────────────────────────────────────────────────────────────────────
# Cinemática Inversa (IK) — CR10
# ──────────────────────────────────────────────────────────────────────

def approach_to_Rtcp(approach_vec: np.ndarray) -> np.ndarray:
    """Constrói R_tcp a partir do vetor de abordagem (eixo z do TCP)."""
    z = np.asarray(approach_vec, dtype=float)
    z = z / (np.linalg.norm(z) + 1e-12)
    ref = np.array([0., 0., 1.]) if abs(z[2]) < 0.9 else np.array([1., 0., 0.])
    y = np.cross(z, ref); y /= np.linalg.norm(y) + 1e-12
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


def _rot_error(R_curr: np.ndarray, R_des: np.ndarray) -> np.ndarray:
    """Erro de rotação robusto — fórmula de Rodrigues (vee do skew de R_err)."""
    R_err = R_des @ R_curr.T
    cos_t = float(np.clip(0.5 * (np.trace(R_err) - 1.0), -1.0, 1.0))
    s = 0.5 * np.array([
        R_err[2, 1] - R_err[1, 2],
        R_err[0, 2] - R_err[2, 0],
        R_err[1, 0] - R_err[0, 1],
    ])
    sin_t = float(np.linalg.norm(s))
    if sin_t < 1e-7:
        if cos_t >= 0.0:
            return s
        diag = np.array([R_err[0,0]+1., R_err[1,1]+1., R_err[2,2]+1.])
        idx = int(np.argmax(diag))
        ax = R_err[:, idx] + np.eye(3)[:, idx]
        return math.pi * ax / (float(np.linalg.norm(ax)) + 1e-12)
    return (math.atan2(sin_t, cos_t) / sin_t) * s


def _wrist_from_R36(R36: np.ndarray) -> tuple[float, float, float]:
    """
    Extrai (q4_u, q5_u, q6_u) de R36 em convenção URDF.

    A cadeia de pulso URDF decompõe-se como:
        R36_urdf = Rz(q4_u − π/2) · Ry(−q5_u) · Rz(q6_u)

    Extraindo: q4_u = atan2(−r12/s5, −r02/s5) + π/2
               q5_u = atan2(s5, r22)
               q6_u = atan2(−R36[2,1]/s5,  R36[2,0]/s5)
    """
    r02, r12, r22 = R36[0, 2], R36[1, 2], R36[2, 2]
    s5p = math.sqrt(r02*r02 + r12*r12)
    if s5p > 1e-6:
        q5 = math.atan2(s5p, r22)
        q4 = math.atan2(-r12/s5p, -r02/s5p) + _PI2   # +π/2 vs DH
        q6 = math.atan2(-R36[2,1]/s5p, R36[2,0]/s5p)
    else:
        q5, q4 = 0.0, 0.0
        q6 = math.atan2(-R36[0,1], R36[0,0])
    # Wrap q4 to (−π, π] so atan2 + π/2 does not exceed joint limits
    q4 = (q4 + math.pi) % (2*math.pi) - math.pi
    return q4, q5, q6


def _geometric_guess(p_tcp: np.ndarray, R_tcp: np.ndarray,
                     elbow_up: bool = True,
                     q1_force: float | None = None) -> np.ndarray:
    """
    Palpite inicial analítico geométrico em convenção URDF.

    Relação com DH:
        q2_urdf = θ2_DH − π/2   (q2=0 URDF ≡ braço vertical)
        q3_urdf = θ3_DH          (sem offset)
        q4_urdf = θ4_DH − π/2   (extraído do R36_urdf com +π/2)
        q5_urdf = θ5_DH,  q6_urdf = θ6_DH
    """
    # ── Posição do wrist center ─────────────────────────────────────
    p_wc = p_tcp - _D_WC_TCP * R_tcp[:, 2]

    q1 = float(q1_force) if q1_force is not None else math.atan2(p_wc[1], p_wc[0])

    # ── θ3_DH via lei dos cossenos ──────────────────────────────────
    r = math.sqrt(p_wc[0]**2 + p_wc[1]**2)
    s = p_wc[2] - _D1
    D = (r*r + s*s - _A2*_A2 - _A3*_A3) / (2.0 * _A2 * _A3)
    D = max(-1.0, min(1.0, D))
    sign = 1.0 if elbow_up else -1.0
    th3 = math.atan2(sign * math.sqrt(max(0.0, 1.0 - D*D)), D)   # θ3_DH

    # ── θ2_DH ───────────────────────────────────────────────────────
    th2 = math.atan2(s, r) - math.atan2(_A3*math.sin(th3), _A2 + _A3*math.cos(th3))

    # ── Converter para URDF ─────────────────────────────────────────
    q2 = th2 - _PI2                                      # q2_urdf = θ2_DH − π/2
    q3 = th3                                              # q3_urdf = θ3_DH

    # ── R36_urdf via fk_partial URDF ───────────────────────────────
    R_flange_target = R_tcp @ T_HAND_ATTACH[:3, :3].T
    q_tmp = np.array([q1, q2, q3, 0., 0., 0.])
    R03 = fk_partial(q_tmp, 3)[:3, :3]
    R36 = R03.T @ R_flange_target

    q4, q5, q6 = _wrist_from_R36(R36)

    return np.array([q1, q2, q3, q4, q5, q6])


def _set_wrist(q: np.ndarray, R_target: np.ndarray) -> np.ndarray:
    """
    Recalcula q3-q5 analiticamente dado q0-q2 (posição do braço).
    Testa ambas as soluções do pulso (±q5_u) e retorna a de menor erro.
    """
    R_flange_target = R_target @ T_HAND_ATTACH[:3, :3].T
    R03 = fk_partial(q, 3)[:3, :3]
    R36 = R03.T @ R_flange_target

    r02, r12, r22 = R36[0, 2], R36[1, 2], R36[2, 2]
    s5p = math.sqrt(r02*r02 + r12*r12)

    solutions = []
    if s5p > 1e-6:
        for sign in (+1.0, -1.0):
            s5 = sign * s5p
            q5 = math.atan2(s5, r22)
            q4_raw = math.atan2(-sign * r12/s5p, -sign * r02/s5p) + _PI2  # +π/2 URDF
            q4 = (q4_raw + math.pi) % (2*math.pi) - math.pi   # wrap to (−π, π]
            q6 = math.atan2(-sign * R36[2,1]/s5p,  sign * R36[2,0]/s5p)
            q_cand = q.copy(); q_cand[3], q_cand[4], q_cand[5] = q4, q5, q6
            T_check = forward_kinematics(q_cand)
            ang_err = float(np.linalg.norm(_rot_error(T_check[:3,:3], R_target)))
            solutions.append((ang_err, q_cand))
    else:
        q5, q4 = 0.0, 0.0
        q6 = math.atan2(-R36[0,1], R36[0,0])
        q_cand = q.copy(); q_cand[3], q_cand[4], q_cand[5] = q4, q5, q6
        T_check = forward_kinematics(q_cand)
        ang_err = float(np.linalg.norm(_rot_error(T_check[:3,:3], R_target)))
        solutions.append((ang_err, q_cand))

    solutions.sort(key=lambda x: x[0])
    return solutions[0][1]


def _ik_refine(p_target: np.ndarray, R_target: np.ndarray,
               q0: np.ndarray,
               n_iter: int = 300,
               tol_pos: float = 3e-3,
               tol_ori: float = 0.05) -> tuple[np.ndarray, bool]:
    """
    Refinamento numérico IK — abordagem desacoplada iterativa.

    Estágio 1: 4 ciclos de (DLS 3-DOF braço + _set_wrist analítico).
    Estágio 2: ajuste fino 6-DOF DLS (100 iter).
    """
    q = q0.copy().astype(float)
    I3 = np.eye(3)
    I6 = np.eye(6)
    lr = 0.40

    n_arm = 60
    for _cycle in range(4):
        lam_arm = 0.06
        for i in range(n_arm):
            lam_i = lam_arm * (0.003/lam_arm) ** (float(i)/n_arm)
            T = forward_kinematics(q)
            dp = p_target - T[:3, 3]
            if float(np.linalg.norm(dp)) < tol_pos:
                break
            J_arm = jacobian(q)[:3, :3]
            dq3 = J_arm.T @ np.linalg.solve(J_arm @ J_arm.T + lam_i*lam_i*I3, dp)
            q[:3] = np.clip(q[:3] + lr*dq3, JOINT_MIN[:3], JOINT_MAX[:3])

        q = _set_wrist(q, R_target)

        T = forward_kinematics(q)
        dp_c = float(np.linalg.norm(p_target - T[:3, 3]))
        dw_c = float(np.linalg.norm(_rot_error(T[:3, :3], R_target)))
        if dp_c < tol_pos and dw_c < tol_ori:
            return q, True

    W_ori = 0.25
    lam_fine = 0.005
    for _ in range(100):
        T = forward_kinematics(q)
        dp = p_target - T[:3, 3]
        dw_raw = _rot_error(T[:3, :3], R_target)
        if float(np.linalg.norm(dp)) < tol_pos and float(np.linalg.norm(dw_raw)) < tol_ori:
            return q, True
        dw = W_ori * dw_raw
        J = jacobian(q).copy(); J[3:, :] *= W_ori
        dq = J.T @ np.linalg.solve(J @ J.T + lam_fine*lam_fine*I6, np.concatenate([dp, dw]))
        q = np.clip(q + lr*dq, JOINT_MIN, JOINT_MAX)

    T_f = forward_kinematics(q)
    dp_f = float(np.linalg.norm(p_target - T_f[:3, 3]))
    dw_f = float(np.linalg.norm(_rot_error(T_f[:3, :3], R_target)))
    return q, dp_f < 8e-3 and dw_f < 0.25


def _ik_candidates(p_tcp: np.ndarray, R_tcp: np.ndarray,
                    q_seed: np.ndarray | None,
                    elbow_up: bool = True) -> list[np.ndarray]:
    """Gera candidatos de palpite inicial varrendo q1 ±40° e ambos os cotovelos."""
    candidates: list[np.ndarray] = []
    q1_naive = math.atan2(p_tcp[1], p_tcp[0])
    primary   = True  if elbow_up else False
    secondary = False if elbow_up else True

    for dq1 in (-0.7, -0.4, -0.2, 0.0, 0.2, 0.4, 0.7):
        candidates.append(_geometric_guess(p_tcp, R_tcp, primary,   q1_force=q1_naive+dq1))
    candidates.append(_geometric_guess(p_tcp, R_tcp, primary))

    for dq1 in (-0.7, -0.4, -0.2, 0.0, 0.2, 0.4, 0.7):
        candidates.append(_geometric_guess(p_tcp, R_tcp, secondary, q1_force=q1_naive+dq1))
    candidates.append(_geometric_guess(p_tcp, R_tcp, secondary))

    if q_seed is not None:
        candidates.insert(0, np.asarray(q_seed, dtype=float))

    return candidates


def inverse_kinematics(
        p_tcp: np.ndarray,
        approach_vec: np.ndarray,
        q_seed: np.ndarray | None = None,
        elbow_up: bool = True) -> tuple[np.ndarray, bool]:
    """
    IK completa do CR10 — retorna ângulos URDF (prontos para enviar ao Gazebo).

    Args:
        p_tcp:        posição desejada do TCP (3,) em metros [frame base]
        approach_vec: direção de abordagem unitária (TCP z-axis)
        q_seed:       palpite inicial opcional em convenção URDF
        elbow_up:     preferência de configuração de cotovelo

    Returns:
        (q, converged): ângulos URDF (6,) rad e flag de convergência
    """
    R_tcp = approach_to_Rtcp(np.asarray(approach_vec))
    p_tcp = np.asarray(p_tcp, dtype=float)

    candidates = _ik_candidates(p_tcp, R_tcp, q_seed, elbow_up)

    best_q, best_err, best_ok = candidates[0].copy(), 1e9, False

    for cand in candidates:
        q_cand = np.clip(cand, JOINT_MIN, JOINT_MAX)
        q, ok = _ik_refine(p_tcp, R_tcp, q_cand)
        T = forward_kinematics(q)
        err = float(np.linalg.norm(p_tcp - T[:3, 3]))
        if err < best_err:
            best_err = err; best_q = q; best_ok = ok
        if best_ok and best_err < 2e-3:
            break

    return best_q, best_ok and best_err < 10e-3


# ──────────────────────────────────────────────────────────────────────
# Métricas de qualidade cinemática
# ──────────────────────────────────────────────────────────────────────

def reach_margin(q: np.ndarray) -> float:
    """Margem de alcance [0, 1]: quão longe está da fronteira exterior do workspace."""
    p_wc = fk_partial(q, 3)[:3, 3]
    r = math.sqrt(p_wc[0]**2 + p_wc[1]**2)
    s = p_wc[2] - _D1
    dist_sq = r*r + s*s
    return float(max(0.0, 1.0 - dist_sq / (_A2 + _A3)**2))


def singularity_distances(q: np.ndarray) -> tuple[float, float, float]:
    """
    Distâncias normalizadas [0, 1] de cada tipo de singularidade.
    Retorna (shoulder_dist, elbow_dist, wrist_dist).
    """
    p_wc = fk_partial(q, 3)[:3, 3]
    r = math.sqrt(p_wc[0]**2 + p_wc[1]**2)
    s = p_wc[2] - _D1
    D = (r*r + s*s - _A2*_A2 - _A3*_A3) / (2.0 * _A2 * _A3)
    D = max(-1.0, min(1.0, D))

    d_shoulder = min(1.0, r / 0.10)
    d_elbow    = min(1.0, (1.0 - abs(D)) / 0.20)
    d_wrist    = min(1.0, abs(math.sin(float(q[4]))) / math.sin(math.radians(10)))

    return (d_shoulder, d_elbow, d_wrist)


# ──────────────────────────────────────────────────────────────────────
# Cinemática da Mão COVVI
# ──────────────────────────────────────────────────────────────────────

def finger_fk(driver_angle: float,
              l_prox: float = _L_PROX,
              l_dist: float = _L_DIST,
              k_p: float = _K_P_FINGER,
              k_d: float = _K_D_FINGER) -> np.ndarray:
    """FK de um dedo (cadeia 2-link planar). Retorna [x, 0, z] no frame MCP."""
    t1 = k_p * driver_angle
    t2 = k_d * driver_angle
    x = l_prox * math.cos(t1) + l_dist * math.cos(t1 + t2)
    z = l_prox * math.sin(t1) + l_dist * math.sin(t1 + t2)
    return np.array([x, 0.0, z])


def _finger_ik(target_xz: np.ndarray,
               l_prox: float, l_dist: float,
               k_p: float) -> float:
    """IK 2-link → ângulo driver. Retorna nan se fora do alcance."""
    xf, zf = float(target_xz[0]), float(target_xz[1])
    D = (xf*xf + zf*zf - l_prox*l_prox - l_dist*l_dist) / (2.0 * l_prox * l_dist)
    if abs(D) > 1.0:
        return float('nan')
    t2 = math.atan2(-math.sqrt(max(0.0, 1.0 - D*D)), D)
    t1 = math.atan2(zf, xf) - math.atan2(l_dist*math.sin(t2), l_prox + l_dist*math.cos(t2))
    return t1 / k_p


HAND_CONFIGS: dict[str, dict[str, float]] = {
    'open': {
        'Thumb': 0.00, 'Index': 0.00, 'Middle': 0.00,
        'Ring':  0.00, 'Little': 0.00, 'Rotate': 0.00,
    },
    'pinch': {
        'Thumb': 0.85, 'Index': 0.80, 'Middle': 0.05,
        'Ring':  0.05, 'Little': 0.05, 'Rotate': 0.85,
    },
    'cylindrical': {
        'Thumb': 1.20, 'Index': 1.20, 'Middle': 1.20,
        'Ring':  1.10, 'Little': 1.00, 'Rotate': 0.50,
    },
    'spherical': {
        'Thumb': 1.00, 'Index': 1.00, 'Middle': 1.00,
        'Ring':  1.00, 'Little': 0.90, 'Rotate': 0.60,
    },
    'palm_grip': {
        'Thumb': 1.10, 'Index': 1.25, 'Middle': 1.25,
        'Ring':  1.20, 'Little': 1.10, 'Rotate': 0.25,
    },
    'claw_grip': {
        'Thumb': 0.90, 'Index': 1.10, 'Middle': 1.10,
        'Ring':  1.05, 'Little': 0.95, 'Rotate': 0.45,
    },
    'fingertip_grip': {
        'Thumb': 0.75, 'Index': 0.70, 'Middle': 0.65,
        'Ring':  0.05, 'Little': 0.05, 'Rotate': 0.82,
    },
}

HAND_LIMITS: dict[str, float] = {
    'Thumb': 1.6, 'Index': 1.6, 'Middle': 1.6,
    'Ring':  1.6, 'Little': 1.6, 'Rotate': 1.0,
}


def hand_ik(grasp_type: str, obj_diameter: float = 0.0) -> dict[str, float]:
    """IK da mão COVVI: retorna driver joints (rad) para o tipo de grasp."""
    cfg = dict(HAND_CONFIGS[grasp_type])
    if obj_diameter <= 0.0 or grasp_type == 'open':
        return cfg

    _NOMINAL_DIAM = {
        'pinch': 0.010, 'cylindrical': 0.070, 'spherical': 0.060,
        'palm_grip': 0.064, 'claw_grip': 0.050, 'fingertip_grip': 0.010,
    }
    d_nom = _NOMINAL_DIAM.get(grasp_type, obj_diameter)
    if abs(d_nom) < 1e-9 or abs(obj_diameter - d_nom) < 5e-3:
        return cfg

    scale = float(np.clip(obj_diameter / d_nom, 0.50, 1.30))
    for j in ['Thumb', 'Index', 'Middle', 'Ring', 'Little']:
        cfg[j] = float(np.clip(cfg[j] * scale, 0.05, HAND_LIMITS[j]))

    return cfg
