"""
Módulo centralizado de cinemática — CR10 + COVVI Hand.

Convenção DH padrão (Denavit-Hartenberg):
    T_i = Rot_z(θᵢ) · Trans_z(dᵢ) · Trans_x(aᵢ) · Rot_x(αᵢ)

    ⎡ cθ  -sθcα   sθsα   a·cθ ⎤
    ⎢ sθ   cθcα  -cθsα   a·sθ ⎥
    ⎢  0    sα     cα    d    ⎥
    ⎣  0    0      0     1    ⎦

Parâmetros DH do CR10 (User Guide v1.8, Fig. 5.13):
    d1=176.5mm, a2=607mm, a3=568mm, d4=193mm, d5=125mm, d6=111.4mm

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

# ──────────────────────────────────────────────────────────────────────
# Parâmetros DH — Standard DH, colunas [a (m), alpha (rad), d (m)]
# ──────────────────────────────────────────────────────────────────────
_PI2 = math.pi / 2

DH_CR10 = np.array([
    #   a        alpha      d
    [0.0000,   _PI2,    0.1765],   # J1 — rotação da base (waist)
    [0.6070,   0.000,   0.0000],   # J2 — ombro (shoulder)
    [0.5680,   0.000,   0.0000],   # J3 — cotovelo (elbow)
    [0.0000,   _PI2,    0.1930],   # J4 — rotação do pulso (wrist roll)
    [0.0000,  -_PI2,    0.1250],   # J5 — inclinação do pulso (wrist pitch)
    [0.0000,   0.000,   0.1114],   # J6 — flange (wrist yaw / spin)
], dtype=float)

# Transformação fixa flange → base da mão COVVI (hand_attach_joint)
#   xyz = [0, 0, 0.01],  rpy = [π/2, 0, 0]  →  Rx(π/2)
T_HAND_ATTACH = np.array([
    [1.0,  0.0,  0.0, 0.00],
    [0.0,  0.0, -1.0, 0.00],
    [0.0,  1.0,  0.0, 0.01],
    [0.0,  0.0,  0.0, 1.00],
], dtype=float)

# Limites articulares do CR10 (rad)
JOINT_MIN = np.deg2rad(np.array([-180.0, -135.0, -135.0, -135.0, -135.0, -360.0]))
JOINT_MAX = np.deg2rad(np.array([ 180.0,  135.0,  135.0,  135.0,  135.0,  360.0]))

# Distância efetiva WC→TCP ao longo da direção de abordagem (m).
# Inclui contribuições de d5, d6 e hand_attach para grasps verticais.
# Valor calibrado para minimizar erro de posição do IK geométrico.
_D_WC_TCP = 0.260

# ──────────────────────────────────────────────────────────────────────
# Parâmetros da mão COVVI
# ──────────────────────────────────────────────────────────────────────
_L_PROX       = 0.0450   # comprimento da falange proximal (m)
_L_DIST       = 0.0300   # comprimento da falange distal   (m)
_K_P_FINGER   = 1.516    # razão mimic proximal — dedos normais
_K_D_FINGER   = 0.718    # razão mimic distal   — dedos normais
_K_P_THUMB    = 1.400    # razão mimic proximal — polegar


# ──────────────────────────────────────────────────────────────────────
# Primitiva DH — matriz homogênea de um elo
# ──────────────────────────────────────────────────────────────────────

def _dh(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa,  a * ct],
        [st,  ct * ca, -ct * sa,  a * st],
        [0.0,       sa,      ca,  d     ],
        [0.0,      0.0,     0.0,  1.0   ],
    ])


# ──────────────────────────────────────────────────────────────────────
# Cinemática Direta (FK)
# ──────────────────────────────────────────────────────────────────────

def forward_kinematics(q: np.ndarray,
                       include_hand: bool = True) -> np.ndarray:
    """
    FK completa: base → flange J6 (opcionalmente até a base da mão COVVI).

    Args:
        q:            ângulos das juntas (6,) em rad
        include_hand: aplica T_HAND_ATTACH ao resultado se True

    Returns:
        T: pose do efetuador, matriz homogênea 4×4
    """
    T = np.eye(4)
    for (a, alpha, d), qi in zip(DH_CR10, q):
        T = T @ _dh(a, alpha, d, float(qi))
    if include_hand:
        T = T @ T_HAND_ATTACH
    return T


def fk_partial(q: np.ndarray, n_links: int) -> np.ndarray:
    """FK dos primeiros n_links elos — ex.: n_links=3 retorna T₀₃."""
    T = np.eye(4)
    for i in range(n_links):
        a, alpha, d = DH_CR10[i]
        T = T @ _dh(a, alpha, d, float(q[i]))
    return T


# ──────────────────────────────────────────────────────────────────────
# Jacobiano (diferenças finitas)
# ──────────────────────────────────────────────────────────────────────

def jacobian(q: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """
    Jacobiano geométrico 6×6 (translação nas 3 primeiras linhas,
    rotação nas 3 últimas) via diferenças finitas.
    """
    J = np.zeros((6, 6))
    T0 = forward_kinematics(q)
    p0, R0 = T0[:3, 3].copy(), T0[:3, :3].copy()

    for i in range(6):
        dq = q.copy()
        dq[i] += eps
        T1 = forward_kinematics(dq)
        J[:3, i] = (T1[:3, 3] - p0) / eps
        # Erro angular escalarizado via vetorização anti-simétrica
        dR = (T1[:3, :3] - R0) / eps
        J[3, i] = (dR[2, 1] - dR[1, 2]) / 2.0
        J[4, i] = (dR[0, 2] - dR[2, 0]) / 2.0
        J[5, i] = (dR[1, 0] - dR[0, 1]) / 2.0

    return J


def manipulability(q: np.ndarray) -> float:
    """
    Índice de manipulabilidade translacional de Yoshikawa:
        w = sqrt(det(Jv · Jvᵀ))

    Valores maiores indicam configuração mais afastada de singularidades.
    """
    J = jacobian(q)
    Jv = J[:3, :]
    val = float(np.sqrt(max(0.0, np.linalg.det(Jv @ Jv.T))))
    return val


# ──────────────────────────────────────────────────────────────────────
# Cinemática Inversa (IK) — CR10
# ──────────────────────────────────────────────────────────────────────

def approach_to_Rtcp(approach_vec: np.ndarray) -> np.ndarray:
    """
    Constrói a matriz de rotação do TCP a partir do vetor de abordagem.

    O vetor de abordagem define o eixo z do TCP (aponta para o objeto).
    Os eixos x e y são escolhidos para minimizar o rolamento do efetuador.
    """
    z = np.asarray(approach_vec, dtype=float)
    z = z / (np.linalg.norm(z) + 1e-12)
    ref = np.array([0.0, 0.0, 1.0]) if abs(z[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    y = np.cross(z, ref)
    y /= np.linalg.norm(y) + 1e-12
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


def _rot_error(R_curr: np.ndarray, R_des: np.ndarray) -> np.ndarray:
    """Erro de rotação como vetor (3,) — eixo × sen(ângulo)."""
    return 0.5 * (
        np.cross(R_curr[:, 0], R_des[:, 0]) +
        np.cross(R_curr[:, 1], R_des[:, 1]) +
        np.cross(R_curr[:, 2], R_des[:, 2])
    )


def _geometric_guess(p_tcp: np.ndarray, R_tcp: np.ndarray,
                     elbow_up: bool = True,
                     q1_force: float | None = None) -> np.ndarray:
    """
    Estimativa inicial analítica geométrica (pulso esférico aproximado).

    Passos:
      1. WC = p_tcp − d_eff·ẑ_tcp
      2. θ₁ = atan2(WCy, WCx)  [ou q1_force se fornecido]
      3. θ₂, θ₃ via lei dos cossenos
      4. θ₄, θ₅, θ₆ via R₃₆ = R₀₃ᵀ·R_tcp  [com q1 correto!]

    q1_force: forçar q1 a um valor específico (para multi-start varredura).
      Sem isso, substituir g[0] após o cálculo deixa q4-q6 inconsistentes,
      pois R03 (que depende de q1) foi calculada com o q1 original.
    """
    a2 = DH_CR10[1, 0]   # 0.607
    a3 = DH_CR10[2, 0]   # 0.568
    d1 = DH_CR10[0, 2]   # 0.1765

    p_wc = p_tcp - _D_WC_TCP * R_tcp[:, 2]

    # θ₁ — usa q1_force se fornecido, caso contrário cálculo analítico
    q1 = float(q1_force) if q1_force is not None else math.atan2(p_wc[1], p_wc[0])

    # θ₃ — lei dos cossenos sobre a projeção do WC
    r = math.sqrt(p_wc[0] ** 2 + p_wc[1] ** 2)
    s = p_wc[2] - d1
    D = (r * r + s * s - a2 * a2 - a3 * a3) / (2.0 * a2 * a3)
    D = max(-1.0, min(1.0, D))
    sign = 1.0 if elbow_up else -1.0
    q3 = math.atan2(sign * math.sqrt(max(0.0, 1.0 - D * D)), D)

    # θ₂
    q2 = math.atan2(s, r) - math.atan2(a3 * math.sin(q3),
                                         a2 + a3 * math.cos(q3))

    # θ₄, θ₅, θ₆ via R₃₆ = R₀₃ᵀ · R_tcp  (R03 usa o q1 correto)
    q_tmp = np.array([q1, q2, q3, 0.0, 0.0, 0.0])
    R03 = fk_partial(q_tmp, 3)[:3, :3]
    R36 = R03.T @ R_tcp

    r02, r12, r22 = R36[0, 2], R36[1, 2], R36[2, 2]
    s5 = math.sqrt(r02 * r02 + r12 * r12)
    q5 = math.atan2(s5, r22)

    if s5 > 1e-6:
        q4 = math.atan2(r12 / s5, r02 / s5)
        q6 = math.atan2(-R36[2, 1] / s5, R36[2, 0] / s5)
    else:
        q4 = 0.0
        q6 = math.atan2(-R36[0, 1], R36[0, 0])

    return np.array([q1, q2, q3, q4, q5, q6])


def _ik_refine(p_target: np.ndarray, R_target: np.ndarray,
               q0: np.ndarray,
               n_iter: int = 150,
               tol: float = 5e-4,
               lr: float = 0.45,
               lam: float = 0.08) -> tuple[np.ndarray, bool]:
    """
    Refinamento numérico via pseudo-inverso amortecido (DLS) com lambda adaptativo.

    Lambda decai exponencialmente de lam (0.08) até lam_min (0.003) ao longo
    das iterações: começo estável (previne oscilações em poses de alta extensão)
    e final preciso (permite convergência fina).

    Returns (q, converged).
    """
    q = q0.copy().astype(float)
    I6 = np.eye(6)
    lam_min = 0.003

    for i in range(n_iter):
        lam_i = lam * (lam_min / lam) ** (i / n_iter)
        T = forward_kinematics(q)
        dp = p_target - T[:3, 3]
        dw = _rot_error(T[:3, :3], R_target)
        err = np.concatenate([dp, dw])
        if np.linalg.norm(err) < tol:
            return q, True
        J = jacobian(q)
        JJt = J @ J.T
        dq = J.T @ np.linalg.solve(JJt + lam_i * lam_i * I6, err)
        q = np.clip(q + lr * dq, JOINT_MIN, JOINT_MAX)

    T_f = forward_kinematics(q)
    return q, float(np.linalg.norm(p_target - T_f[:3, 3])) < 10e-3


def _ik_candidates(p_tcp: np.ndarray, R_tcp: np.ndarray,
                    q_seed: np.ndarray | None) -> list[np.ndarray]:
    """
    Gera lista de candidatos de palpite inicial para o IK.

    Varre q1 em torno do atan2 ingênuo e gera variantes de
    cotovelo-acima/abaixo para cada valor, compensando o offset d4.
    """
    candidates: list[np.ndarray] = []

    q1_naive = math.atan2(p_tcp[1], p_tcp[0])

    # Varre q1 num intervalo de ±40° em torno do valor ingênuo
    for dq1 in (-0.7, -0.4, -0.2, 0.0, 0.2, 0.4, 0.7):
        q1 = q1_naive + dq1
        for elbow in (True, False):
            g = _geometric_guess(p_tcp, R_tcp, elbow)
            g = g.copy()
            g[0] = q1   # forçar q1 varrido
            candidates.append(g)

    # Sempre incluir a estimativa geométrica pura
    candidates.append(_geometric_guess(p_tcp, R_tcp, True))
    candidates.append(_geometric_guess(p_tcp, R_tcp, False))

    # Incluir seed externo se disponível
    if q_seed is not None:
        candidates.insert(0, np.asarray(q_seed, dtype=float))

    return candidates


def inverse_kinematics(
        p_tcp: np.ndarray,
        approach_vec: np.ndarray,
        q_seed: np.ndarray | None = None,
        elbow_up: bool = True) -> tuple[np.ndarray, bool]:
    """
    IK completa do CR10 dada posição do TCP e vetor de abordagem.

    Estratégia multi-start: gera ~16 candidatos de palpite inicial
    (varrendo q1 ±40° e ambas as configurações de cotovelo) e refina
    numericamente cada um.  Retorna a solução com menor erro residual.

    Args:
        p_tcp:        posição desejada do TCP (3,) em metros [frame base]
        approach_vec: direção de abordagem unitária (TCP z-axis)
        q_seed:       palpite inicial opcional (e.g., solução anterior)
        elbow_up:     preferência de configuração de cotovelo (guia a varredura)

    Returns:
        (q, converged): ângulos (6,) rad e flag de convergência
    """
    R_tcp = approach_to_Rtcp(np.asarray(approach_vec))
    p_tcp = np.asarray(p_tcp, dtype=float)

    candidates = _ik_candidates(p_tcp, R_tcp, q_seed)

    best_q   = candidates[0].copy()
    best_err = 1e9
    best_ok  = False

    for cand in candidates:
        q_cand = np.clip(cand, JOINT_MIN, JOINT_MAX)
        q, ok = _ik_refine(p_tcp, R_tcp, q_cand)
        T = forward_kinematics(q)
        err = float(np.linalg.norm(p_tcp - T[:3, 3]))
        if err < best_err:
            best_err = err
            best_q   = q
            best_ok  = ok
        if best_ok and best_err < 2e-3:
            break   # solução boa o suficiente — não procurar mais

    return best_q, best_ok and best_err < 10e-3


# ──────────────────────────────────────────────────────────────────────
# Métricas de qualidade cinemática
# ──────────────────────────────────────────────────────────────────────

def reach_margin(q: np.ndarray) -> float:
    """
    Margem de alcance [0, 1]: quão longe está da fronteira exterior
    do espaço de trabalho. 0 = limite externo, 1 = centro do workspace.
    """
    p_wc = fk_partial(q, 3)[:3, 3]
    a2, a3 = DH_CR10[1, 0], DH_CR10[2, 0]
    d1 = DH_CR10[0, 2]
    r = math.sqrt(p_wc[0] ** 2 + p_wc[1] ** 2)
    s = p_wc[2] - d1
    dist_sq = r * r + s * s
    r_max_sq = (a2 + a3) ** 2
    return float(max(0.0, 1.0 - dist_sq / r_max_sq))


def singularity_distances(q: np.ndarray) -> tuple[float, float, float]:
    """
    Distâncias normalizadas [0, 1] de cada tipo de singularidade.
    Retorna (shoulder_dist, elbow_dist, wrist_dist).
    Valores próximos de 0 indicam singularidade iminente.
    """
    T03 = fk_partial(q, 3)
    p_wc = T03[:3, 3]
    a2, a3 = DH_CR10[1, 0], DH_CR10[2, 0]
    d1 = DH_CR10[0, 2]

    # Singularidade de ombro: WC sobre o eixo J1 (r ≈ 0)
    r = math.sqrt(p_wc[0] ** 2 + p_wc[1] ** 2)
    d_shoulder = min(1.0, r / 0.10)

    # Singularidade de cotovelo: braço totalmente estendido/recolhido
    s = p_wc[2] - d1
    D = (r * r + s * s - a2 * a2 - a3 * a3) / (2.0 * a2 * a3)
    D = max(-1.0, min(1.0, D))
    d_elbow = min(1.0, (1.0 - abs(D)) / 0.20)

    # Singularidade de pulso: sin(q5) ≈ 0
    d_wrist = min(1.0, abs(math.sin(float(q[4]))) / math.sin(math.radians(10)))

    return (d_shoulder, d_elbow, d_wrist)


# ──────────────────────────────────────────────────────────────────────
# Cinemática da Mão COVVI
# ──────────────────────────────────────────────────────────────────────

def finger_fk(driver_angle: float,
              l_prox: float = _L_PROX,
              l_dist: float = _L_DIST,
              k_p: float = _K_P_FINGER,
              k_d: float = _K_D_FINGER) -> np.ndarray:
    """
    FK de um dedo (cadeia 2-link planar no plano sagital).

    Returns: posição da ponta (x, 0, z) no referencial MCP do dedo.
    """
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
    D = (xf * xf + zf * zf - l_prox * l_prox - l_dist * l_dist) / (
        2.0 * l_prox * l_dist)
    if abs(D) > 1.0:
        return float('nan')
    t2 = math.atan2(-math.sqrt(max(0.0, 1.0 - D * D)), D)
    t1 = math.atan2(zf, xf) - math.atan2(
        l_dist * math.sin(t2), l_prox + l_dist * math.cos(t2))
    return t1 / k_p


# Configurações nominais de mão por tipo de grasp (driver joints, rad)
HAND_CONFIGS: dict[str, dict[str, float]] = {
    'open': {
        'Thumb': 0.00, 'Index': 0.00, 'Middle': 0.00,
        'Ring':  0.00, 'Little': 0.00, 'Rotate': 0.00,
    },
    'pinch': {
        # Polegar + indicador em precisão; demais dedos recolhidos
        'Thumb': 0.85, 'Index': 0.80, 'Middle': 0.05,
        'Ring':  0.05, 'Little': 0.05, 'Rotate': 0.85,
    },
    'cylindrical': {
        # Todos os dedos em arco envolvendo o cilindro
        'Thumb': 1.20, 'Index': 1.20, 'Middle': 1.20,
        'Ring':  1.10, 'Little': 1.00, 'Rotate': 0.50,
    },
    'spherical': {
        # Power grasp esférico — todos os dedos curvados
        'Thumb': 1.00, 'Index': 1.00, 'Middle': 1.00,
        'Ring':  1.00, 'Little': 0.90, 'Rotate': 0.60,
    },
}

# Limites máximos dos driver joints (rad) — clamp de segurança
HAND_LIMITS: dict[str, float] = {
    'Thumb': 1.6, 'Index': 1.6, 'Middle': 1.6,
    'Ring':  1.6, 'Little': 1.6, 'Rotate': 1.0,
}


def hand_ik(grasp_type: str, obj_diameter: float = 0.0) -> dict[str, float]:
    """
    IK da mão COVVI: retorna driver joints (rad) para o tipo de grasp.
    Usa configurações nominais de HAND_CONFIGS como base e escala
    levemente pela abertura necessária para o diâmetro do objeto.

    A escala é linear:
      - objeto muito grande (≥ diâmetro máximo) → 100% da config nominal
      - objeto muito pequeno (≤ diâmetro mínimo) → escala reduzida
    A direção segura é MANTER os valores nominais pré-calibrados.

    Args:
        grasp_type:   'open' | 'pinch' | 'cylindrical' | 'spherical'
        obj_diameter: diâmetro do objeto em metros (≤0 usa config padrão)

    Returns:
        dict {joint_name: angle_rad} para as 6 juntas primárias
    """
    cfg = dict(HAND_CONFIGS[grasp_type])
    if obj_diameter <= 0.0 or grasp_type == 'open':
        return cfg

    # Diâmetros de calibração (m): objeto para qual a config nominal é ideal
    _NOMINAL_DIAM = {'pinch': 0.010, 'cylindrical': 0.070, 'spherical': 0.060}
    d_nom = _NOMINAL_DIAM.get(grasp_type, obj_diameter)

    if abs(d_nom) < 1e-9 or abs(obj_diameter - d_nom) < 5e-3:
        return cfg   # dentro de 5 mm do nominal — não ajustar

    # Escala proporcional: objetos menores precisam de menos abertura
    # (os dedos ficam mais fechados para objetos menores)
    scale = float(np.clip(obj_diameter / d_nom, 0.50, 1.30))

    # Aplica escala apenas nas juntas de flexão (não no Rotate)
    for j in ['Thumb', 'Index', 'Middle', 'Ring', 'Little']:
        cfg[j] = float(np.clip(cfg[j] * scale, 0.05, HAND_LIMITS[j]))

    return cfg
