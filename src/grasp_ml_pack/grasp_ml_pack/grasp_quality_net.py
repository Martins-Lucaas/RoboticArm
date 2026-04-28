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

Backends:
  MLP  (padrão se PyTorch disponível) — treina na GPU (ROCm ou CUDA)
  RF   (fallback)                     — RandomForestClassifier sklearn, CPU

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
# PyTorch — importado sob demanda para não quebrar sem GPU
# ─────────────────────────────────────────────────────────────────────
try:
    import torch
    import torch.nn as nn
    _TORCH_AVAILABLE = True
except ImportError:
    _TORCH_AVAILABLE = False


def _get_device() -> 'torch.device':
    """Retorna GPU (ROCm ou CUDA) se disponível, caso contrário CPU."""
    if not _TORCH_AVAILABLE:
        return None
    if torch.cuda.is_available():   # ROCm expõe a mesma API via HIP
        dev = torch.device('cuda')
        name = torch.cuda.get_device_name(0)
        print(f'  GPU detectada: {name}')
        return dev
    print('  GPU não detectada — usando CPU')
    return torch.device('cpu')


# ─────────────────────────────────────────────────────────────────────
# Metadados das features
# ─────────────────────────────────────────────────────────────────────
_FEATURE_NAMES = [
    'obj_pencil', 'obj_cup', 'obj_ball',
    'gp_x', 'gp_y', 'gp_z',
    'go_roll', 'go_pitch', 'go_yaw',
    'aperture',
    'cfg_pinch', 'cfg_cylindrical', 'cfg_spherical',
    'av_x', 'av_y', 'av_z',
    'q1', 'q2', 'q3', 'q4', 'q5', 'q6',
    'manipulability',
    'reach_margin',
    'elbow_up',
    'ik_converged',
]
N_FEATURES = len(_FEATURE_NAMES)   # 26

_OBJ_IDX = {'pencil': 0, 'cup': 1, 'ball': 2}
_CFG_IDX = {'pinch': 0, 'cylindrical': 1, 'spherical': 2}

_HEURISTIC_BASE = {
    (0, 0): 0.80,
    (1, 1): 0.85,
    (2, 2): 0.82,
}


# ─────────────────────────────────────────────────────────────────────
# Arquitetura MLP
# ─────────────────────────────────────────────────────────────────────
if _TORCH_AVAILABLE:
    class _MLP(nn.Module):
        """MLP 26→128→64→32→1 com BatchNorm e Dropout para classificação binária."""

        def __init__(self, n_features: int = N_FEATURES):
            super().__init__()
            self.net = nn.Sequential(
                nn.Linear(n_features, 128),
                nn.BatchNorm1d(128),
                nn.ReLU(),
                nn.Dropout(0.30),
                nn.Linear(128, 64),
                nn.BatchNorm1d(64),
                nn.ReLU(),
                nn.Dropout(0.20),
                nn.Linear(64, 32),
                nn.ReLU(),
                nn.Linear(32, 1),
                nn.Sigmoid(),
            )

        def forward(self, x: 'torch.Tensor') -> 'torch.Tensor':
            return self.net(x).squeeze(-1)


# ─────────────────────────────────────────────────────────────────────
# Construtores de features
# ─────────────────────────────────────────────────────────────────────

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

    obj_oh = np.zeros(3)
    obj_oh[_OBJ_IDX[object_type]] = 1.0
    cfg_oh = np.zeros(3)
    cfg_oh[_CFG_IDX[finger_config]] = 1.0
    q_feat = np.zeros(6) if q_arm is None else np.asarray(q_arm, dtype=float)

    return np.concatenate([
        obj_oh,
        np.asarray(grasp_pos_rel, float),
        np.asarray(grasp_euler, float),
        [float(aperture)],
        cfg_oh,
        np.asarray(approach_vec, float),
        q_feat,
        [float(manip)],
        [float(r_margin)],
        [1.0 if elbow_up else 0.0],
        [1.0 if ik_ok else 0.0],
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

    approach_vec = np.asarray(approach_vec, float)
    approach_unit = approach_vec / (np.linalg.norm(approach_vec) + 1e-12)

    q_arm, ik_ok = inverse_kinematics(
        np.asarray(grasp_pos_world, float),
        approach_unit,
        q_seed=q_seed,
        elbow_up=True,
    )

    manip     = manipulability(q_arm) if ik_ok else 0.0
    r_mg      = reach_margin(q_arm)   if ik_ok else 0.0
    elbow_up_flag = bool(q_arm[2] >= 0) if ik_ok else True
    grasp_pos_rel = np.asarray(grasp_pos_world) - np.asarray(obj_pos_world)

    return build_feature_vector(
        object_type, grasp_pos_rel, grasp_euler, aperture,
        finger_config, approach_unit,
        q_arm=q_arm, manip=manip, r_margin=r_mg,
        elbow_up=elbow_up_flag, ik_ok=ik_ok,
    )


# ─────────────────────────────────────────────────────────────────────
# Modelo principal
# ─────────────────────────────────────────────────────────────────────

class GraspQualityNet:
    """
    Wrapper unificado para MLP PyTorch (GPU) ou RandomForestClassifier (CPU).

    O backend é selecionado automaticamente:
      - PyTorch disponível + GPU (ROCm/CUDA) → MLP treina na GPU
      - PyTorch disponível sem GPU            → MLP treina na CPU (ainda útil)
      - PyTorch não instalado                 → RandomForest sklearn
    """

    _BACKEND_MLP = 'mlp'
    _BACKEND_RF  = 'rf'

    def __init__(self, model=None, backend: str = _BACKEND_RF):
        self._model   = model
        self._backend = backend
        self._device  = None
        if backend == self._BACKEND_MLP and _TORCH_AVAILABLE and model is not None:
            self._device = _get_device()
            self._model  = model.to(self._device)

    # ------------------------------------------------------------------
    def predict(self, features: np.ndarray) -> float:
        """Retorna probabilidade de sucesso [0, 1]."""
        if self._model is None:
            return self._heuristic_score(np.asarray(features, float))
        if self._backend == self._BACKEND_MLP:
            return self._predict_mlp(features)
        x = np.asarray(features, float).reshape(1, -1)
        return float(self._model.predict_proba(x)[0, 1])

    def predict_batch(self, X: np.ndarray) -> np.ndarray:
        """Retorna array de probabilidades para um batch (N, 26)."""
        if self._model is None:
            return np.array([self._heuristic_score(x) for x in X])
        if self._backend == self._BACKEND_MLP:
            t = torch.tensor(X, dtype=torch.float32).to(self._device)
            self._model.eval()
            with torch.no_grad():
                return self._model(t).cpu().numpy()
        return self._model.predict_proba(X)[:, 1]

    def _predict_mlp(self, features: np.ndarray) -> float:
        t = torch.tensor(
            np.asarray(features, dtype=np.float32).reshape(1, -1),
            dtype=torch.float32,
        ).to(self._device)
        self._model.eval()
        with torch.no_grad():
            return float(self._model(t).item())

    # ------------------------------------------------------------------
    @staticmethod
    def _heuristic_score(features: np.ndarray) -> float:
        obj_oh = features[0:3]
        gp     = features[3:6]
        cfg_oh = features[10:13]
        manip  = features[22]
        r_mg   = features[23]
        ik_ok  = features[25]

        if ik_ok < 0.5:
            return 0.05
        obj_idx = int(np.argmax(obj_oh))
        cfg_idx = int(np.argmax(cfg_oh))
        if (obj_idx, cfg_idx) not in _HEURISTIC_BASE:
            return 0.12
        base     = _HEURISTIC_BASE[(obj_idx, cfg_idx)]
        dist_pen = min(float(np.linalg.norm(gp)) * 1.5, 0.20)
        kin_bonus = 0.06 * min(manip / 0.10, 1.0) + 0.04 * r_mg
        return float(np.clip(base - dist_pen + kin_bonus, 0.0, 1.0))

    # ------------------------------------------------------------------
    @classmethod
    def load(cls, path: str) -> 'GraspQualityNet':
        """Carrega MLP (.pt) ou RandomForest (.pkl), o que existir."""
        pt_path = os.path.splitext(path)[0] + '.pt'
        if _TORCH_AVAILABLE and os.path.exists(pt_path):
            mlp = _MLP(N_FEATURES)
            mlp.load_state_dict(
                torch.load(pt_path, map_location='cpu', weights_only=True))
            return cls(mlp, backend=cls._BACKEND_MLP)
        if os.path.exists(path):
            with open(path, 'rb') as f:
                model = pickle.load(f)
            return cls(model, backend=cls._BACKEND_RF)
        return cls(None, backend=cls._BACKEND_RF)

    def save(self, path: str):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        if self._backend == self._BACKEND_MLP:
            pt_path = os.path.splitext(path)[0] + '.pt'
            torch.save(self._model.cpu().state_dict(), pt_path)
            # Devolve o modelo para o device correto após salvar
            if self._device is not None:
                self._model = self._model.to(self._device)
        else:
            with open(path, 'wb') as f:
                pickle.dump(self._model, f)

    # ------------------------------------------------------------------
    @staticmethod
    def train(X: np.ndarray, y: np.ndarray,
              save_path: str | None = None) -> 'GraspQualityNet':
        if _TORCH_AVAILABLE:
            return GraspQualityNet._train_mlp(X, y, save_path)
        return GraspQualityNet._train_rf(X, y, save_path)

    # ------------------------------------------------------------------
    @staticmethod
    def _train_mlp(X: np.ndarray, y: np.ndarray,
                   save_path: str | None = None) -> 'GraspQualityNet':
        from sklearn.model_selection import StratifiedKFold
        from sklearn.metrics import roc_auc_score
        from torch.utils.data import DataLoader, TensorDataset
        import torch.optim as optim

        device = _get_device()
        X32 = X.astype(np.float32)
        y32 = y.astype(np.float32)

        # ── Cross-validation 5-fold ─────────────────────────────────
        skf = StratifiedKFold(n_splits=5, shuffle=True, random_state=42)
        auc_scores = []
        for tr, va in skf.split(X32, y.astype(int)):
            m   = _MLP(N_FEATURES).to(device)
            opt = optim.Adam(m.parameters(), lr=3e-3, weight_decay=1e-4)
            crit = nn.BCELoss()
            Xt = torch.tensor(X32[tr]).to(device)
            yt = torch.tensor(y32[tr]).to(device)
            Xv = torch.tensor(X32[va]).to(device)
            for _ in range(200):
                m.train()
                opt.zero_grad()
                crit(m(Xt), yt).backward()
                opt.step()
            m.eval()
            with torch.no_grad():
                probs = m(Xv).cpu().numpy()
            auc_scores.append(roc_auc_score(y[va], probs))

        print(f'  MLP  CV AUC-ROC: {np.mean(auc_scores):.3f} ± {np.std(auc_scores):.3f}')

        # ── Treino final em todos os dados ──────────────────────────
        model = _MLP(N_FEATURES).to(device)
        opt   = optim.Adam(model.parameters(), lr=3e-3, weight_decay=1e-4)
        sched = optim.lr_scheduler.ReduceLROnPlateau(opt, patience=20, factor=0.5)
        crit  = nn.BCELoss()

        Xt = torch.tensor(X32).to(device)
        yt = torch.tensor(y32).to(device)
        bs = max(min(64, len(X32) // 4), 1)
        loader = DataLoader(TensorDataset(Xt, yt), batch_size=bs, shuffle=True)

        print(f'  Treinando MLP (device={device}, batch={bs}) ...')
        for epoch in range(300):
            model.train()
            total = 0.0
            for xb, yb in loader:
                opt.zero_grad()
                loss = crit(model(xb), yb)
                loss.backward()
                opt.step()
                total += loss.item()
            avg = total / len(loader)
            sched.step(avg)
            if epoch % 50 == 0:
                print(f'    epoch {epoch:3d}  loss={avg:.4f}  '
                      f'lr={opt.param_groups[0]["lr"]:.2e}')

        net = GraspQualityNet(model, backend=GraspQualityNet._BACKEND_MLP)
        if save_path:
            net.save(save_path)
            pt_path = os.path.splitext(save_path)[0] + '.pt'
            print(f'  Modelo salvo em: {pt_path}')
        return net

    # ------------------------------------------------------------------
    @staticmethod
    def _train_rf(X: np.ndarray, y: np.ndarray,
                  save_path: str | None = None) -> 'GraspQualityNet':
        from sklearn.ensemble import RandomForestClassifier
        from sklearn.model_selection import cross_val_score

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
        imp  = rf.feature_importances_
        top5 = np.argsort(imp)[::-1][:5]
        print('  Top-5 features mais importantes:')
        for idx in top5:
            print(f'    [{idx:2d}] {_FEATURE_NAMES[idx]:22s} {imp[idx]:.3f}')

        net = GraspQualityNet(rf, backend=GraspQualityNet._BACKEND_RF)
        if save_path:
            net.save(save_path)
            print(f'  Modelo salvo em: {save_path}')
        return net
