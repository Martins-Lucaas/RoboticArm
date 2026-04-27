"""
GraspQualityNet — classificador de qualidade de grasp com features cinemáticas.

Feature vector (26 dimensões):
  [0:3]   obj_onehot       — pencil / cup / ball
  [3:6]   grasp_pos_rel    — posição do grasp relativa ao centróide (m)
  [6:9]   grasp_euler      — orientação do objeto (roll/pitch/yaw, rad)
  [9]     aperture_norm    — abertura normalizada [0, 1]
  [10:13] cfg_onehot       — pinch / cylindrical / spherical
  [13:16] approach_vec     — vetor de abordagem unitário (ẑ_tcp)
  [16:22] q_arm            — ângulos das juntas do braço IK (rad) ← cinemática
  [22]    manipulability   — índice de Yoshikawa (translacional)  ← cinemática
  [23]    reach_margin     — margem de alcance [0, 1]             ← cinemática
  [24]    elbow_up         — 1=cotovelo acima, 0=abaixo           ← cinemática
  [25]    ik_converged     — 1=IK convergiu, 0=falhou             ← cinemática

Uso:
    net = GraspQualityNet.load('models/grasp_quality.pkl')
    score = net.predict(features)   # features: np.ndarray (26,)
"""

from __future__ import annotations

import os
import pickle
import numpy as np

from .kinematics import (
    inverse_kinematics, manipulability, reach_margin,
    HAND_CONFIGS,
)

# ─────────────────────────────────────────────────────────────────────
# Metadados das features
# ─────────────────────────────────────────────────────────────────────
_FEATURE_NAMES = [
    # objeto
    'obj_pencil', 'obj_cup', 'obj_ball',
    # posição e orientação
    'gp_x', 'gp_y', 'gp_z',
    'go_roll', 'go_pitch', 'go_yaw',
    # abertura da mão
    'aperture',
    # configuração de dedos
    'cfg_pinch', 'cfg_cylindrical', 'cfg_spherical',
    # vetor de abordagem
    'av_x', 'av_y', 'av_z',
    # cinemática do braço (IK)
    'q1', 'q2', 'q3', 'q4', 'q5', 'q6',
    # métricas cinemáticas
    'manipulability',
    'reach_margin',
    'elbow_up',
    'ik_converged',
]
N_FEATURES = len(_FEATURE_NAMES)   # 26

_OBJ_IDX = {'pencil': 0, 'cup': 1, 'ball': 2}
_CFG_IDX = {'pinch': 0, 'cylindrical': 1, 'spherical': 2}

# Heurísticas base antes do modelo ML estar treinado
_HEURISTIC_BASE = {
    (0, 0): 0.80,   # pencil  + pinch
    (1, 1): 0.85,   # cup     + cylindrical
    (2, 2): 0.82,   # ball    + spherical
}


def build_feature_vector(
        object_type: str,
        grasp_pos_rel: np.ndarray,
        grasp_euler: np.ndarray,
        aperture: float,
        finger_config: str,
        approach_vec: np.ndarray,
        q_arm: np.ndarray | None = None,
        manip: float = 0.0,
        r_margin: float = 0.5,
        elbow_up: bool = True,
        ik_ok: bool = False) -> np.ndarray:
    """
    Monta o vetor de features (26,) para a GraspQualityNet.

    Se q_arm for None, as features cinemáticas são zero/default.
    """
    obj_oh = np.zeros(3)
    obj_oh[_OBJ_IDX[object_type]] = 1.0

    cfg_oh = np.zeros(3)
    cfg_oh[_CFG_IDX[finger_config]] = 1.0

    q_feat = np.zeros(6) if q_arm is None else np.asarray(q_arm, dtype=float)

    return np.concatenate([
        obj_oh,                              # [0:3]
        np.asarray(grasp_pos_rel, float),    # [3:6]
        np.asarray(grasp_euler, float),      # [6:9]
        [float(aperture)],                   # [9]
        cfg_oh,                              # [10:13]
        np.asarray(approach_vec, float),     # [13:16]
        q_feat,                              # [16:22]
        [float(manip)],                      # [22]
        [float(r_margin)],                   # [23]
        [1.0 if elbow_up else 0.0],          # [24]
        [1.0 if ik_ok else 0.0],             # [25]
    ])


def build_feature_vector_with_ik(
        object_type: str,
        grasp_pos_world: np.ndarray,
        obj_pos_world: np.ndarray,
        grasp_euler: np.ndarray,
        aperture: float,
        finger_config: str,
        approach_vec: np.ndarray,
        q_seed: np.ndarray | None = None) -> np.ndarray:
    """
    Versão completa: calcula automaticamente o IK e popula as features
    cinemáticas a partir da posição do grasp no referencial do mundo.
    """
    approach_vec = np.asarray(approach_vec, float)
    approach_unit = approach_vec / (np.linalg.norm(approach_vec) + 1e-12)

    q_arm, ik_ok = inverse_kinematics(
        np.asarray(grasp_pos_world, float),
        approach_unit,
        q_seed=q_seed,
        elbow_up=True,
    )

    manip = manipulability(q_arm) if ik_ok else 0.0
    r_mg  = reach_margin(q_arm)   if ik_ok else 0.0
    elbow_up_flag = bool(q_arm[2] >= 0) if ik_ok else True

    grasp_pos_rel = np.asarray(grasp_pos_world) - np.asarray(obj_pos_world)

    return build_feature_vector(
        object_type, grasp_pos_rel, grasp_euler, aperture,
        finger_config, approach_unit,
        q_arm=q_arm, manip=manip, r_margin=r_mg,
        elbow_up=elbow_up_flag, ik_ok=ik_ok,
    )


# ─────────────────────────────────────────────────────────────────────
# Modelo
# ─────────────────────────────────────────────────────────────────────

class GraspQualityNet:
    """
    Wrapper de um RandomForestClassifier (sklearn) para scoring de grasps.
    Antes do treinamento usa heurísticas analíticas + bônus cinemático.
    """

    def __init__(self, model=None):
        self._model = model

    # ------------------------------------------------------------------
    def predict(self, features: np.ndarray) -> float:
        """Retorna probabilidade de sucesso [0, 1]."""
        x = np.asarray(features, float).reshape(1, -1)
        if self._model is not None:
            if x.shape[1] != N_FEATURES:
                raise ValueError(
                    f'Feature vector tem {x.shape[1]} dimensões; esperado {N_FEATURES}')
            return float(self._model.predict_proba(x)[0, 1])
        return self._heuristic_score(x[0])

    # ------------------------------------------------------------------
    @staticmethod
    def _heuristic_score(features: np.ndarray) -> float:
        """
        Score analítico pré-treinamento.

        Prioridades:
          1. IK inviável → score muito baixo
          2. Par (objeto, config) errado → penalidade forte
          3. Par correto → bônus cinemático (manipulabilidade, alcance)
          4. Penalidade por offset grande do centróide
        """
        obj_oh = features[0:3]
        gp     = features[3:6]
        cfg_oh = features[10:13]
        manip  = features[22]
        r_mg   = features[23]
        ik_ok  = features[25]

        if ik_ok < 0.5:
            return 0.05   # posição inatingível

        obj_idx = int(np.argmax(obj_oh))
        cfg_idx = int(np.argmax(cfg_oh))

        if (obj_idx, cfg_idx) not in _HEURISTIC_BASE:
            return 0.12   # par objeto/config incorreto — desaconselhar fortemente

        base = _HEURISTIC_BASE[(obj_idx, cfg_idx)]

        # Penalidade de distância ao centróide (offset de preensão grande → falha)
        dist_pen = min(float(np.linalg.norm(gp)) * 1.5, 0.20)

        # Bônus cinemático moderado: manipulabilidade e margem de alcance
        kin_bonus = 0.06 * min(manip / 0.10, 1.0) + 0.04 * r_mg

        return float(np.clip(base - dist_pen + kin_bonus, 0.0, 1.0))

    # ------------------------------------------------------------------
    @classmethod
    def load(cls, path: str) -> 'GraspQualityNet':
        if os.path.exists(path):
            with open(path, 'rb') as f:
                model = pickle.load(f)
            return cls(model)
        return cls(None)

    def save(self, path: str):
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        with open(path, 'wb') as f:
            pickle.dump(self._model, f)

    # ------------------------------------------------------------------
    @staticmethod
    def train(X: np.ndarray, y: np.ndarray,
              save_path: str | None = None) -> 'GraspQualityNet':
        """
        Treina RandomForest com validação cruzada 5-fold.

        X: (N, 26) features
        y: (N,)    labels binários (1=sucesso, 0=falha)
        """
        from sklearn.ensemble import RandomForestClassifier, GradientBoostingClassifier
        from sklearn.model_selection import cross_val_score
        from sklearn.pipeline import Pipeline
        from sklearn.preprocessing import StandardScaler

        if X.shape[1] != N_FEATURES:
            raise ValueError(f'X deve ter {N_FEATURES} colunas, tem {X.shape[1]}')

        # RandomForest robusto para features mistas (one-hot + contínuas)
        rf = RandomForestClassifier(
            n_estimators=300,
            max_depth=14,
            min_samples_leaf=2,
            max_features='sqrt',
            class_weight='balanced',
            random_state=42,
            n_jobs=-1,
        )

        cv_auc = cross_val_score(rf, X, y, cv=5, scoring='roc_auc')
        print(f'  RandomForest  CV AUC-ROC: {cv_auc.mean():.3f} ± {cv_auc.std():.3f}')

        rf.fit(X, y)
        net = GraspQualityNet(rf)
        if save_path:
            net.save(save_path)
            print(f'  Modelo salvo em: {save_path}')

        # Importância das features
        imp = rf.feature_importances_
        top5 = np.argsort(imp)[::-1][:5]
        print('  Top-5 features mais importantes:')
        for idx in top5:
            print(f'    [{idx:2d}] {_FEATURE_NAMES[idx]:20s} {imp[idx]:.3f}')

        return net
