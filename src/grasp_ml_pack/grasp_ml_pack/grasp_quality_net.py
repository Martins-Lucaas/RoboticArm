"""
grasp_quality_net.py — modelo de qualidade de grasp.

API pública:
    N_FEATURES                    = 26
    _FEATURE_NAMES                — lista dos 26 nomes de features
    build_feature_vector_with_ik(obj, grasp_pos, obj_pos, euler,
                                  aperture, gtype, av, q_seed=None)
                                  → np.ndarray (26,)
    GraspQualityNet.train(X, y, save_path) → GraspQualityNet
    GraspQualityNet.load(path)             → GraspQualityNet
    net.predict_batch(X)                   → np.ndarray (N,)
    net._backend                           → 'rf' | 'nn'
    net._model                             → modelo interno
"""
from __future__ import annotations

import os
import pickle

import numpy as np

# ── Categorias ────────────────────────────────────────────────────────────────

_OBJECTS     = ['pencil', 'cup', 'ball']
_GRASP_TYPES = ['pinch', 'cylindrical', 'spherical']

# ── Nomes das 26 features ─────────────────────────────────────────────────────
# [0:16]  features originais (geometria + identidade do grasp)
# [16:26] features cinemáticas (IK + qualidade da configuração)

_FEATURE_NAMES: list[str] = [
    # originais [0–15]
    'obj_pencil', 'obj_cup', 'obj_ball',            # one-hot objeto        [0-2]
    'gt_pinch', 'gt_cylindrical', 'gt_spherical',   # one-hot grasp type    [3-5]
    'obj_x', 'obj_y', 'obj_z',                      # posição do objeto     [6-8]
    'euler_r', 'euler_p', 'euler_y',                # orientação Euler      [9-11]
    'aperture',                                      # abertura da mão       [12]
    'av_x', 'av_y', 'av_z',                         # vetor de approach     [13-15]
    # cinemáticas [16–25]
    'grasp_x', 'grasp_y', 'grasp_z',               # posição do TCP grasp  [16-18]
    'grasp_offset_norm',                             # ||grasp - obj||       [19]
    'shoulder_dist',                                 # dist. sing. ombro     [20]
    'elbow_dist',                                    # dist. sing. cotovelo  [21]
    'manipulability',                                # índice de Yoshikawa   [22]
    'reach_margin',                                  # margem de alcance     [23]
    'wrist_dist',                                    # dist. sing. pulso     [24]
    'ik_ok',                                         # IK convergiu (0/1)    [25]
]

N_FEATURES: int = len(_FEATURE_NAMES)   # 26


# ── Construção do vetor de features ──────────────────────────────────────────

def build_feature_vector_with_ik(
        obj: str,
        grasp_pos: np.ndarray,
        obj_pos: np.ndarray,
        euler: np.ndarray,
        aperture: float,
        gtype: str,
        av: np.ndarray,
        q_seed: np.ndarray | None = None,
) -> np.ndarray:
    """Constrói vetor de 26 features para um grasp candidato.

    Se q_seed for None (IK não convergiu), as features cinemáticas [16:26]
    ficam zeradas e ik_ok = 0.
    """
    feat = np.zeros(N_FEATURES, dtype=float)

    # one-hot objeto
    if obj in _OBJECTS:
        feat[_OBJECTS.index(obj)] = 1.0

    # one-hot tipo de grasp
    if gtype in _GRASP_TYPES:
        feat[3 + _GRASP_TYPES.index(gtype)] = 1.0

    feat[6:9]   = np.asarray(obj_pos,   dtype=float)
    feat[9:12]  = np.asarray(euler,     dtype=float)
    feat[12]    = float(aperture)
    feat[13:16] = np.asarray(av,        dtype=float)

    # features cinemáticas
    feat[16:19] = np.asarray(grasp_pos, dtype=float)
    feat[19]    = float(np.linalg.norm(
        np.asarray(grasp_pos, dtype=float) - np.asarray(obj_pos, dtype=float)))

    if q_seed is not None:
        from .kinematics import manipulability, reach_margin, singularity_distances
        q           = np.asarray(q_seed, dtype=float)
        s, e, w     = singularity_distances(q)
        feat[20]    = float(s)
        feat[21]    = float(e)
        feat[22]    = float(manipulability(q))
        feat[23]    = float(reach_margin(q))
        feat[24]    = float(w)
        feat[25]    = 1.0   # ik_ok

    return feat


# ── Modelo ────────────────────────────────────────────────────────────────────

class GraspQualityNet:
    """Classificador binário de qualidade de grasp.

    Backends suportados:
      'rf' — RandomForestClassifier (scikit-learn), preferido quando disponível.
      'nn' — MLP simples (torch), usado quando scikit-learn não está instalado.
    """

    def __init__(self, backend: str, model):
        self._backend = backend
        self._model   = model

    # ── Inferência ────────────────────────────────────────────────────────
    def predict_batch(self, X: np.ndarray) -> np.ndarray:
        """Probabilidades de sucesso para cada linha de X (shape N×26)."""
        X = np.asarray(X, dtype=float)
        if self._backend == 'rf':
            return self._model.predict_proba(X)[:, 1]
        import torch
        self._model.eval()
        with torch.no_grad():
            t = torch.tensor(X, dtype=torch.float32)
            return self._model(t).squeeze(-1).numpy()

    def predict(self, x: np.ndarray) -> float:
        return float(self.predict_batch(x.reshape(1, -1))[0])

    # ── Treinamento ───────────────────────────────────────────────────────
    @classmethod
    def train(cls, X: np.ndarray, y: np.ndarray,
              save_path: str = 'models/grasp_quality.pkl') -> 'GraspQualityNet':
        X = np.asarray(X, dtype=float)
        y = np.asarray(y, dtype=float)
        try:
            net = cls._train_rf(X, y)
        except ImportError:
            print('[INFO] scikit-learn não encontrado — usando backend nn (torch).')
            net = cls._train_nn(X, y)
        net._save(save_path)
        return net

    @classmethod
    def _train_rf(cls, X: np.ndarray, y: np.ndarray) -> 'GraspQualityNet':
        from sklearn.ensemble import RandomForestClassifier
        from sklearn.model_selection import StratifiedKFold, cross_val_score

        model = RandomForestClassifier(
            n_estimators=200, max_depth=None,
            min_samples_leaf=2, class_weight='balanced',
            random_state=42, n_jobs=-1)

        n_pos, n_neg = int(y.sum()), int((y == 0).sum())
        n_splits = min(5, n_pos, n_neg)
        if n_splits >= 2:
            cv = StratifiedKFold(n_splits=n_splits, shuffle=True, random_state=42)
            scores = cross_val_score(model, X, y.astype(int),
                                     cv=cv, scoring='roc_auc')
            print(f'  CV AUC-ROC: {scores.mean():.3f} ± {scores.std():.3f}')

        model.fit(X, y.astype(int))
        print(f'  RandomForest treinado — {model.n_estimators} árvores, '
              f'{X.shape[1]} features')
        return cls(backend='rf', model=model)

    @classmethod
    def _train_nn(cls, X: np.ndarray, y: np.ndarray) -> 'GraspQualityNet':
        import torch
        import torch.nn as nn
        import torch.optim as optim

        n_in = X.shape[1]

        # Modelo com logits para BCEWithLogitsLoss durante treino
        model_logit = nn.Sequential(
            nn.Linear(n_in, 64), nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(64, 32),   nn.ReLU(),
            nn.Linear(32, 1),
        )

        n_pos = float((y == 1).sum())
        n_neg = float((y == 0).sum())
        pos_weight = torch.tensor([n_neg / max(n_pos, 1)], dtype=torch.float32)
        criterion  = nn.BCEWithLogitsLoss(pos_weight=pos_weight)
        optimizer  = optim.Adam(model_logit.parameters(), lr=1e-3, weight_decay=1e-4)

        Xt = torch.tensor(X, dtype=torch.float32)
        yt = torch.tensor(y, dtype=torch.float32).unsqueeze(1)

        for epoch in range(300):
            model_logit.train()
            optimizer.zero_grad()
            loss = criterion(model_logit(Xt), yt)
            loss.backward()
            optimizer.step()
            if (epoch + 1) % 100 == 0:
                print(f'  epoch {epoch+1}/300  loss={loss.item():.4f}')

        # Modelo de inferência com Sigmoid
        model_infer = nn.Sequential(
            *[layer for layer in model_logit
              if not isinstance(layer, nn.Dropout)],
            nn.Sigmoid(),
        )
        model_infer.eval()
        print(f'  MLP treinado — {n_in}→64→32→1')
        return cls(backend='nn', model=model_infer)

    # ── Persistência ──────────────────────────────────────────────────────
    def _save(self, path: str):
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        if self._backend == 'rf':
            with open(path, 'wb') as f:
                pickle.dump(self, f)
        else:
            import torch
            pt_path = os.path.splitext(path)[0] + '.pt'
            torch.save(self._model.state_dict(), pt_path)

    @classmethod
    def load(cls, path: str) -> 'GraspQualityNet':
        if path.endswith('.pt'):
            import torch
            import torch.nn as nn
            model = nn.Sequential(
                nn.Linear(N_FEATURES, 64), nn.ReLU(),
                nn.Linear(64, 32),         nn.ReLU(),
                nn.Linear(32, 1),          nn.Sigmoid(),
            )
            model.load_state_dict(torch.load(path, map_location='cpu',
                                             weights_only=True))
            model.eval()
            return cls(backend='nn', model=model)
        with open(path, 'rb') as f:
            return pickle.load(f)
