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
# Acoplamento URDF (com acoplador da prótese, PecasProtese.stl):
#   Link6 → hand_coupler_link: xyz="0 0 0"        rpy="0 0 0"
#   hand_coupler_link → hand_base_link: xyz="0 0 0.05546" rpy="π/2 0 0"
# O acoplador (disco ⌀75×55.46 mm) desloca a mão +55.46 mm ao longo de
# +Link6_z. A rotação Rx(π/2) deixa:
#   hand_x = +Link6_x   (largura da palma — polegar↔mínimo)
#   hand_y = +Link6_z   (DIREÇÃO DOS DEDOS quando estendidos)
#   hand_z = −Link6_y   (espessura da palma — frente↔trás)
#
# Para um grasp top-down, queremos o eixo de approach (TCP_z) ALINHADO com a
# direção dos dedos (hand_y), para que a mão se aproxime do objeto "pela
# ponta dos dedos" — caso contrário a IK orienta Link6_y para cima e os
# dedos saem horizontais, sem alcançar o objeto.
#
# Logo TCP_z = +hand_y = +Link6_z. TCP_y = +Link6_y (palm-front), TCP_x =
# +Link6_x. O resultado é uma identidade de rotação + translação ao longo
# do eixo de approach até o ponto de convergência dos fingertips.
#
# Translação 170.46 mm = acoplador 55.46 mm + 115 mm da palma ao TCP
# (hand_y dos MCPs ≈ 0.091 m + alcance médio dos dedos curvados ≈ 0.024 m,
# calibrado pela FK da mão). NB: no modo tactile a palpação usa só
# T_TOUCH_TOOL_ATTACH; este transform é mantido em sincronia com
# grasp_ml_pack/kinematics.py.
T_HAND_ATTACH = np.array([
    [1.0,  0.0,  0.0,  0.00000],
    [0.0,  1.0,  0.0,  0.00000],
    [0.0,  0.0,  1.0,  0.17046],  # 55.46 mm acoplador + 115 mm palma→TCP
    [0.0,  0.0,  0.0,  1.00000],
], dtype=float)

# TCP do TouchTool Square 20×20 mm com acopladores encaixados na célula de carga.
# Cadeia desde Link6:
#   lower_coupling (0mm) → force_sensor (+7mm, célula entra 8mm no acoplador)
#   → upper_coupling (+59mm, célula entra 8mm) → touch_tool (+74mm) → tcp (+188.5mm)
T_TOUCH_TOOL_ATTACH = np.array([
    [1.0,  0.0,  0.0,  0.000],
    [0.0,  1.0,  0.0,  0.000],
    [0.0,  0.0,  1.0,  0.1885],  # 188.5 mm — ponta do probe 20×20 mm
    [0.0,  0.0,  0.0,  1.000],
], dtype=float)

# Limites articulares — convenção URDF (rad).
# Joints 2 e 4 têm offset de -π/2 em relação à convenção DH;
# os limites físicos são mapeados de ±170° (DH) → [-260°, +80°] (URDF).
# Joint 3: limite URDF real é ±2.861 rad (≈±164°) — igual ao xacro do CR10.
# Joint 5: limite físico ±135° (wrist2 do CR10).
JOINT_MIN = np.array([np.deg2rad(-180.), np.deg2rad(-260.),
                       -2.861,
                       np.deg2rad(-260.), np.deg2rad(-135.), np.deg2rad(-360.)])
JOINT_MAX = np.array([np.deg2rad( 180.), np.deg2rad(  80.),
                        2.861,
                        np.deg2rad(  80.), np.deg2rad( 135.), np.deg2rad( 360.)])

# Distância efetiva WC→TCP ao longo do vetor de abordagem (m).
# WC→hand_base_link ≈ 0.260; acoplador 0.05546 + offset palma→TCP 0.115
# → soma = 0.43046. Apenas heurística inicial: o refinamento numérico do
# IK (DLS) absorve diferenças residuais.
_D_WC_TCP = 0.43046

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

# Sinal do eixo de junta no URDF: joint1 usa axis="0 0 -1" (sentido CW positivo
# do CR10 é oposto ao +Z padrão); juntas 2-6 usam axis="0 0 +1" (original).
_JOINT_AXIS_SIGN = np.array([-1., 1., 1., 1., 1., 1.])


def forward_kinematics(q: np.ndarray,
                       include_hand: bool = True,
                       T_end: np.ndarray | None = None) -> np.ndarray:
    """
    FK completa: base → flange Link6 (opcionalmente até TCP do efector).

    Args:
        q:            ângulos das juntas URDF (6,) em rad
        include_hand: aplica T_HAND_ATTACH se True e T_end for None
        T_end:        transform fixo Link6→TCP a usar no lugar de T_HAND_ATTACH;
                      use T_TOUCH_TOOL_ATTACH para o modo palpação.

    Returns:
        T: pose do efetuador, matriz homogênea 4×4
    """
    T = np.eye(4)
    for (xyz, rpy), qi, asign in zip(_URDF_ORIGINS, q, _JOINT_AXIS_SIGN):
        T = T @ _make_T(xyz, rpy) @ _Rz4(asign * float(qi))
    if T_end is not None:
        T = T @ T_end
    elif include_hand:
        T = T @ T_HAND_ATTACH
    return T


def fk_partial(q: np.ndarray, n_links: int) -> np.ndarray:
    """FK dos primeiros n_links elos — ex.: n_links=3 retorna T₀₃."""
    T = np.eye(4)
    for i in range(n_links):
        xyz, rpy = _URDF_ORIGINS[i]
        T = T @ _make_T(xyz, rpy) @ _Rz4(_JOINT_AXIS_SIGN[i] * float(q[i]))
    return T


# ──────────────────────────────────────────────────────────────────────
# Jacobiano geométrico (forma fechada)
# ──────────────────────────────────────────────────────────────────────

def jacobian(q: np.ndarray, eps: float = 1e-6,
             T_end: np.ndarray | None = None) -> np.ndarray:
    """Jacobiano geométrico 6×6 do TCP no frame da base.

    Forma fechada — ~50× mais rápida que diferenças finitas e sem erro
    numérico de truncamento perto de singularidades. O parâmetro `eps`
    fica preservado por compatibilidade (não é usado).

    Para cada junta revoluta i com eixo local z (axis="0 0 ±1" no URDF):
        z_i_world = R_before_i · [0,0,1] · sign_i
        p_i_world = T_before_i[:3, 3]
        J_v[:, i] = z_i_world × (p_end − p_i_world)
        J_ω[:, i] = z_i_world

    `T_before_i` é o produto das transformações até a origem da junta i
    **sem aplicar Rzi(q_i)** — a rotação da própria junta não muda seu
    eixo, então qi só entra na posição de `p_end` e nas juntas seguintes.

    Args:
        T_end: transform fixo Link6→TCP; use T_TOUCH_TOOL_ATTACH para palpação.
    """
    p_end = forward_kinematics(q, include_hand=True, T_end=T_end)[:3, 3]

    J = np.zeros((6, 6))
    T_accum = np.eye(4)
    for i, ((xyz, rpy), qi, asign) in enumerate(
            zip(_URDF_ORIGINS, q, _JOINT_AXIS_SIGN)):
        T_before = T_accum @ _make_T(xyz, rpy)
        z_i = T_before[:3, 2] * asign
        p_i = T_before[:3, 3]
        J[:3, i] = np.cross(z_i, p_end - p_i)
        J[3:, i] = z_i
        T_accum = T_before @ _Rz4(asign * float(qi))

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

    # ── R36 via fk_partial (j1=−Z, j2/j3=+Z) ──────────────────────
    # j1 usa −Z: passar −q1 faz Rz(−1·(−q1))=Rz(q1) — rotação física correta.
    # j2/j3 usam +Z: q2, q3 já estão em convenção URDF, passá-los diretamente.
    R_flange_target = R_tcp @ T_HAND_ATTACH[:3, :3].T
    q_tmp = np.array([-q1, q2, q3, 0., 0., 0.])
    R03 = fk_partial(q_tmp, 3)[:3, :3]
    R36 = R03.T @ R_flange_target

    q4, q5, q6 = _wrist_from_R36(R36)

    return np.array([-q1, q2, q3, q4, q5, q6])


def _set_wrist(q: np.ndarray, R_target: np.ndarray,
                q_ref: np.ndarray | None = None) -> np.ndarray:
    """
    Recalcula q3-q5 analiticamente dado q0-q2 (posição do braço).
    Testa ambas as soluções do pulso (±q5_u) e retorna a de menor erro.

    Quando `q_ref` é fornecido (geralmente o seed da IK), o critério de
    desempate inclui a proximidade no joint-space do pulso (q3-q5):
    soluções de erro angular comparável são desambiguadas pela que
    mantém continuidade com a IK anterior. Isto resolve o "wrist flip"
    indesejado entre poses geometricamente próximas (e.g. POINTING e
    TOUCH na mesma vertical).
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
            q4_raw = math.atan2(-sign * r12/s5p, -sign * r02/s5p) + _PI2
            q4_raw = (q4_raw + math.pi) % (2*math.pi) - math.pi
            q4 = q4_raw
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

    if q_ref is not None and len(solutions) > 1:
        q_ref_arr = np.asarray(q_ref, dtype=float)
        # Custo combinado: erro angular (orientação) + bias de continuidade
        # nas juntas do pulso. O peso 0.30 garante que diferenças de orientação
        # > ~3° (≈0.05 rad → custo 0.05) ainda dominem sobre 90° de
        # diferença no pulso (custo 0.30*1.57≈0.47), mas para erros
        # angulares ≈0 (ambas soluções válidas) o pulso mais próximo vence.
        def cost(item):
            ang_err, q_cand = item
            dist_wrist = float(np.linalg.norm(q_cand[3:6] - q_ref_arr[3:6]))
            return ang_err + 0.30 * dist_wrist
        solutions.sort(key=cost)
    else:
        solutions.sort(key=lambda x: x[0])
    return solutions[0][1]


def _ik_refine(p_target: np.ndarray, R_target: np.ndarray,
               q0: np.ndarray,
               n_iter: int = 300,
               tol_pos: float = 3e-3,
               tol_ori: float = 0.05,
               q_ref: np.ndarray | None = None) -> tuple[np.ndarray, bool]:
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


# Valores re-mapeados para o cap 1.0 rad (driver, todos). Pontos
# importantes:
#   • palm_grip/claw_grip pedem fechamento próximo do máximo para
#     envelopar cilindros — ficam ~0.95-1.0 nos quatro dedos longos.
#   • fingertip_grip mantém Ring/Little em zero (pinça polegar+indicador).
#   • Rotate é cap independente (oposição do polegar), preservado.
# Saturação posterior pela URDF é garantia adicional.
HAND_CONFIGS: dict[str, dict[str, float]] = {
    'open': {
        'Thumb': 0.00, 'Index': 0.00, 'Middle': 0.00,
        'Ring':  0.00, 'Little': 0.00, 'Rotate': 0.00,
    },
    'pinch': {
        'Thumb': 0.80, 'Index': 0.75, 'Middle': 0.05,
        'Ring':  0.05, 'Little': 0.05, 'Rotate': 0.85,
    },
    'cylindrical': {
        'Thumb': 0.90, 'Index': 0.95, 'Middle': 0.95,
        'Ring':  0.90, 'Little': 0.85, 'Rotate': 0.50,
    },
    'spherical': {
        'Thumb': 0.80, 'Index': 0.85, 'Middle': 0.85,
        'Ring':  0.85, 'Little': 0.80, 'Rotate': 0.60,
    },
    'palm_grip': {
        'Thumb': 0.95, 'Index': 1.00, 'Middle': 1.00,
        'Ring':  0.98, 'Little': 0.95, 'Rotate': 0.25,
    },
    # claw_grip: posições MÁXIMAS de fechamento (slider 0..200 →
    # 75/75/80/82/87/200). O PerfectGrasp pára antes via lag-detection
    # quando os dedos encontram o tubo — estes valores são o envelope,
    # não o aperto final aplicado.
    'claw_grip': {
        'Thumb': 0.425, 'Index': 0.450, 'Middle': 0.472,
        'Ring':  0.481, 'Little': 0.503, 'Rotate': 1.000,
    },
    # fingertip_grip: pinça polegar+indicador (Middle/Ring/Little ficam
    # em MIN_RAD = rest pose). Slider 0..200 → 104/62/0/0/0/146.
    'fingertip_grip': {
        'Thumb': 0.558, 'Index': 0.393, 'Middle': 0.120,
        'Ring':  0.120, 'Little': 0.120, 'Rotate': 0.730,
    },
}

# Limites factíveis derivados de hand_pack.urdf_helpers.HAND_DRIVER_LIMITS.
# Drivers cappados em 1.0 rad para que a falange distal NÃO atravesse a
# palma (com self_collide=true bloqueando o resto fisicamente). Wrap
# resultante na ponta do dedo ≈ 163°, suficiente para envelopar objetos
# de raio ≤ 45 mm em power grip.
HAND_LIMITS: dict[str, float] = {
    'Thumb': 1.0, 'Index': 1.0, 'Middle': 1.0,
    'Ring':  1.0, 'Little': 1.0, 'Rotate': 1.0,
}

# ``lower`` calibrado — espelha hand_pack.urdf_helpers.HAND_DRIVER_LOWER.
# Equivale ao ``open_limit`` do firmware da COVVI Hand: posição de
# repouso natural com leve curvatura das falanges.
HAND_LOWER: dict[str, float] = {
    'Thumb': 0.08, 'Index': 0.12, 'Middle': 0.12,
    'Ring':  0.12, 'Little': 0.12, 'Rotate': 0.0,
}


# ──────────────────────────────────────────────────────────────────────
# FK 3D completa da mão COVVI
# ──────────────────────────────────────────────────────────────────────
# Origens dos MCPs em hand_base_link, extraídas direto do URDF
# (joints `*_proximal_j_input_joint`).
_HAND_MCP: dict[str, tuple] = {
    'Index':  (+0.02310, +0.09136, -0.01476),
    'Middle': (+0.00336, +0.09438, -0.01554),
    'Ring':   (-0.01620, +0.09438, -0.01089),
    'Little': (-0.03292, +0.08433, -0.00395),
    # thumb_proximal_j_input em thumb_chassis (rotacionado por Rotate)
    # — não diretamente em hand_base_link, ver _thumb_tip_hand abaixo.
}

# Thumb chassis pivot (Rotate joint) — URDF: parent=base_link.
_THUMB_CHASSIS_PIVOT = np.array([0.02424, 0.02292, 0.01255])
_THUMB_CHASSIS_AXIS  = np.array([0.0, -1.0, 0.0])     # rotação ≈ -hand_y
_THUMB_CHASSIS_MULT  = 1.53339618                      # mimic Rotate

# Em thumb_chassis (frame após Rotate): origem do MCP do polegar.
_THUMB_MCP_IN_CHASSIS = np.array([0.04595, 0.02166, 0.01041])
# Eixo de curl do thumb_proximal no frame thumb_chassis.
_THUMB_PROX_AXIS = np.array([0.14867, -0.13053, 0.98024])


def _rotate_axis_angle(v: np.ndarray, axis: np.ndarray, theta: float) -> np.ndarray:
    """Fórmula de Rodrigues — rotaciona v em torno de axis por theta (rad)."""
    k = axis / (np.linalg.norm(axis) + 1e-12)
    c, s = math.cos(theta), math.sin(theta)
    return v * c + np.cross(k, v) * s + k * (k @ v) * (1.0 - c)


def _finger_tip_in_hand_long(finger: str, primary: float) -> np.ndarray:
    """Ponta dos 4 dedos longos (Index/Middle/Ring/Little) em hand_base_link.

    Modelo planar 2-link: o dedo nasce no MCP e estende ao longo de +hand_y
    quando aberto; curl move o tip em +hand_z (em direção à palma).
    """
    mx, my, mz = _HAND_MCP[finger]
    tip = finger_fk(primary, k_p=_K_P_FINGER, k_d=_K_D_FINGER)
    fx, _fy, fz = float(tip[0]), 0.0, float(tip[2])
    return np.array([mx, my + fx, mz + fz])


def _thumb_tip_in_hand(thumb_primary: float, rotate_primary: float) -> np.ndarray:
    """Ponta do polegar em hand_base_link, considerando o Rotate (que gira
    o chassis do polegar) e em seguida o curl do thumb_proximal/distal.

    Aproximações: o curl 3D do polegar é tratado como planar 2-link no
    plano do chassis, alinhado com `_THUMB_PROX_AXIS` (referência URDF).
    """
    # 1) Curl planar do polegar relativo ao MCP, no plano do chassis.
    tip_planar = finger_fk(thumb_primary, k_p=_K_P_THUMB, k_d=_K_D_FINGER)
    fx, _, fz = float(tip_planar[0]), 0.0, float(tip_planar[2])
    # No frame chassis, o MCP está em _THUMB_MCP_IN_CHASSIS; o dedo
    # estende ao longo de +y_chassis e curla em +z_chassis (aproximação).
    tip_in_chassis = _THUMB_MCP_IN_CHASSIS + np.array([0.0, fx, fz])

    # 2) Rotaciona o resultado pelo ângulo Rotate em torno do pivô do chassis.
    theta = _THUMB_CHASSIS_MULT * rotate_primary
    v = tip_in_chassis - _THUMB_CHASSIS_PIVOT
    v_rot = _rotate_axis_angle(v, _THUMB_CHASSIS_AXIS, theta)
    return _THUMB_CHASSIS_PIVOT + v_rot


def hand_fk(hand_state: dict) -> dict[str, np.ndarray]:
    """FK completa da mão COVVI para um estado de juntas.

    Args:
        hand_state: dict com chaves 'Thumb','Index','Middle','Ring','Little'
                    em rad (ângulo do driver, 0=aberto), opcionalmente 'Rotate'.

    Returns:
        Dict com:
          fingertip positions: 'tip_<finger>' → (3,) em hand_base_link
          MCP positions:        'mcp_<finger>' → (3,) em hand_base_link
          'palm_center':        (3,) — centroide dos MCPs (ponto da palma)
        Todos em metros, frame hand_base_link.
    """
    out: dict[str, np.ndarray] = {}
    rotate = float(hand_state.get('Rotate', 0.0))

    for finger in ('Index', 'Middle', 'Ring', 'Little'):
        primary = float(hand_state.get(finger, 0.0))
        out['mcp_' + finger] = np.array(_HAND_MCP[finger])
        out['tip_' + finger] = _finger_tip_in_hand_long(finger, primary)

    thumb_primary = float(hand_state.get('Thumb', 0.0))
    # MCP do polegar em hand_base_link já considera o Rotate (girar pivô).
    v_mcp = _THUMB_MCP_IN_CHASSIS - _THUMB_CHASSIS_PIVOT
    theta = _THUMB_CHASSIS_MULT * rotate
    out['mcp_Thumb'] = (_THUMB_CHASSIS_PIVOT
                        + _rotate_axis_angle(v_mcp, _THUMB_CHASSIS_AXIS, theta))
    out['tip_Thumb'] = _thumb_tip_in_hand(thumb_primary, rotate)

    out['palm_center'] = np.mean(
        [out[f'mcp_{f}'] for f in ('Thumb', 'Index', 'Middle', 'Ring', 'Little')],
        axis=0)
    return out


def grasp_center_in_hand(hand_state: dict, grip_type: str) -> np.ndarray:
    """Centro do grasp (ponto de contato com o objeto) em hand_base_link.

    Para cada tipo de grasp escolhemos o aglomerado de fingertips que de
    fato encosta no objeto:
      - fingertip: centróide de Thumb+Index (pinch).
      - palm:     centróide dos 5 fingertips + palm_center (envelope).
      - claw:     centróide de Thumb+Index+Middle.
    """
    fks = hand_fk(hand_state)
    grip_type = grip_type.lower()
    if 'fingertip' in grip_type or 'pinch' in grip_type:
        pts = [fks['tip_Thumb'], fks['tip_Index']]
    elif 'claw' in grip_type or 'tripod' in grip_type:
        pts = [fks['tip_Thumb'], fks['tip_Index'], fks['tip_Middle']]
    else:  # palm / cylindrical / default
        pts = [fks[f'tip_{f}'] for f in
               ('Thumb', 'Index', 'Middle', 'Ring', 'Little')]
        pts.append(fks['palm_center'])
    return np.mean(pts, axis=0)


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
        cfg[j] = float(np.clip(cfg[j] * scale, HAND_LOWER[j], HAND_LIMITS[j]))

    return cfg


# ─── Mimic joints da mão COVVI ─────────────────────────────────────────────
# 26 juntas escravas extraídas de linear_covvi_hand_gazebo.urdf.
# Formato: (nome_da_junta_mimic, junta_primária, multiplicador)
# Usado pelo tactile_explorer e palpation_gui para expandir a pose primária
# (6 graus de liberdade) nas 31 juntas da mão antes de publicar no controller.
MIMIC_LIST: list[tuple[str, str, float]] = [
    ('_lisa_j01',            'Rotate', 1.07338),
    ('_thumb_chassis_j01',   'Rotate', 1.53340),
    ('_thumb_proximal_j01',  'Thumb',  0.72022),
    ('_thumb_distal_j01',    'Thumb',  1.06686),
    ('_thumb_link_j01',      'Thumb',  0.76799),
    ('_thumb_follower_j01',  'Thumb',  0.93733),
    ('_index_proximal_j01',  'Index',  1.51604),
    ('_index_distal_j01',    'Index',  1.33574),
    ('_index_knuckle_j01',   'Index',  1.25182),
    ('_index_follower_j01',  'Index',  0.26423),
    ('_index_link_j01',      'Index',  1.33574),
    ('_middle_proximal_j01', 'Middle', 1.51604),
    ('_middle_distal_j01',   'Middle', 1.34986),
    ('_middle_knuckle_j01',  'Middle', 1.25181),
    ('_middle_follower_j01', 'Middle', 0.26423),
    ('_middle_link_j01',     'Middle', 1.34986),
    ('_ring_proximal_j01',   'Ring',   1.51604),
    ('_ring_distal_j01',     'Ring',   1.34878),
    ('_ring_knuckle_j01',    'Ring',   1.25182),
    ('_ring_follower_j01',   'Ring',   0.26423),
    ('_ring_link_j01',       'Ring',   1.34878),
    ('_little_proximal_j01', 'Little', 1.51604),
    ('_little_distal_j01',   'Little', 1.31664),
    ('_little_knuckle_j01',  'Little', 1.25182),
    ('_little_follower_j01', 'Little', 0.26423),
    ('_little_link_j01',     'Little', 1.31664),
]


# ─── Conversão URDF ↔ DOBOT ────────────────────────────────────────────────
# joint1 do URDF tem axis "0 0 -1" (gira em −Z), mas o J1 do CR10 real
# positivo gira em +Z → sinal INVERTIDO na junta 1 (verificado empiricamente:
# sem o flip, sim e real giram a base em direções opostas). joints 2–6 têm a
# mesma convenção nos dois lados. Não há offset angular.
_URDF_DOBOT_SIGN   = np.array([-1., 1., 1., 1., 1., 1.])
_URDF_DOBOT_OFFSET = np.zeros(6)


def urdf_to_dobot(q_urdf: np.ndarray) -> np.ndarray:
    """Converte ângulos do URDF para a convenção do controlador CR10."""
    return (_URDF_DOBOT_SIGN * np.asarray(q_urdf, dtype=np.float64)
            + _URDF_DOBOT_OFFSET)


def dobot_to_urdf(q_dobot: np.ndarray) -> np.ndarray:
    """Converte ângulos lidos do controlador CR10 para a convenção URDF."""
    return _URDF_DOBOT_SIGN * (np.asarray(q_dobot, dtype=np.float64)
                               - _URDF_DOBOT_OFFSET)
