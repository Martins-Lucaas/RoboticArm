"""
Treina a GraspQualityNet com os dados coletados pelo Gazebo.

Backend selecionado automaticamente:
  - PyTorch + GPU (ROCm/CUDA) → MLP treina na GPU
  - PyTorch sem GPU           → MLP treina na CPU
  - Sem PyTorch               → RandomForestClassifier (sklearn)

Uso:
    ros2 run grasp_ml_pack train_model
    ros2 run grasp_ml_pack train_model \
        --ros-args -p data_path:=models/training_data.npz \
                   -p model_path:=models/grasp_quality.pkl
ou:
    python -m grasp_ml_pack.scripts.train_grasp_model \
        --data models/training_data.npz \
        --out  models/grasp_quality.pkl
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np


def _print_gpu_info():
    try:
        import torch
        if torch.cuda.is_available():
            name  = torch.cuda.get_device_name(0)
            total = torch.cuda.get_device_properties(0).total_memory / 1024**3
            print(f'  GPU : {name}  ({total:.1f} GB VRAM)')
            hip = getattr(torch.version, 'hip', None)
            print(f'  API : {"ROCm/HIP " + hip if hip else "CUDA " + torch.version.cuda}')
        else:
            print('  GPU : não detectada — treinando na CPU')
        print(f'  PyTorch: {torch.__version__}')
    except ImportError:
        print('  PyTorch não instalado — usando RandomForest (sklearn)')


def main(args=None):
    parser = argparse.ArgumentParser(description='Treina GraspQualityNet')
    parser.add_argument('--data', default='models/training_data.npz')
    parser.add_argument('--out',  default='models/grasp_quality.pkl')
    parser.add_argument('--min-samples', type=int, default=50)
    parsed = parser.parse_args(sys.argv[1:] if args is None else args)

    # ── Informações de hardware ────────────────────────────────────────
    print('\n=== HARDWARE ===')
    _print_gpu_info()

    # ── 1. Carregar dados ──────────────────────────────────────────────
    if not os.path.exists(parsed.data):
        print(f'\n[ERRO] Arquivo não encontrado: {parsed.data}')
        print('Execute primeiro: ros2 run grasp_ml_pack generate_data')
        sys.exit(1)

    data = np.load(parsed.data)
    X, y = data['X'].astype(float), data['y'].astype(float)
    print(f'\n=== DADOS CARREGADOS ===')
    print(f'  Arquivo : {parsed.data}')
    print(f'  Amostras: {X.shape[0]} × {X.shape[1]} features')
    print(f'  Sucessos: {int(y.sum())} ({100*y.mean():.1f}%)')

    from grasp_ml_pack.grasp_quality_net import N_FEATURES, _FEATURE_NAMES
    if X.shape[1] != N_FEATURES:
        print(f'[AVISO] Dimensão das features ({X.shape[1]}) ≠ {N_FEATURES}.')
        sys.exit(1)

    if X.shape[0] < parsed.min_samples:
        print(f'[AVISO] Poucos dados ({X.shape[0]} < {parsed.min_samples}).')
        if X.shape[0] < 20:
            print('[ERRO] Mínimo absoluto de 20 amostras.')
            sys.exit(1)

    # ── 2. Estatísticas cinemáticas ────────────────────────────────────
    print(f'\n=== FEATURES CINEMÁTICAS ===')
    ik_mask = X[:, 25] > 0.5
    print(f'  IK convergido: {ik_mask.sum()}/{len(ik_mask)} '
          f'({100*ik_mask.mean():.1f}%)')
    if ik_mask.sum() > 0:
        print(f'  Manipulabilidade: μ={X[ik_mask, 22].mean():.4f}')
        print(f'  Margem de alcance: μ={X[ik_mask, 23].mean():.3f}')

    # ── 3. Treinar ─────────────────────────────────────────────────────
    print(f'\n=== TREINAMENTO ===')
    from grasp_ml_pack.grasp_quality_net import GraspQualityNet
    net = GraspQualityNet.train(X, y, save_path=parsed.out)

    # ── 4. Avaliação ───────────────────────────────────────────────────
    from sklearn.metrics import classification_report, roc_auc_score

    net_loaded = GraspQualityNet.load(parsed.out)
    probs = net_loaded.predict_batch(X)
    preds = (probs > 0.5).astype(int)

    print(f'\n=== AVALIAÇÃO (conjunto de treino) ===')
    print(classification_report(y.astype(int), preds,
                                target_names=['falha', 'sucesso']))
    print(f'  AUC-ROC (treino): {roc_auc_score(y, probs):.3f}')

    if net_loaded._backend == GraspQualityNet._BACKEND_RF:
        imp  = net_loaded._model.feature_importances_
        print(f'\n=== IMPORTÂNCIA DAS FEATURES (RF) ===')
        print(f'  Originais  [0:16]: {100*imp[:16].sum():.1f}%')
        print(f'  Cinemáticas [16:]: {100*imp[16:].sum():.1f}%')
        for i, (name, val) in enumerate(zip(_FEATURE_NAMES, imp)):
            print(f'  [{i:2d}] {name:22s} {val:.4f}  {"█" * int(val * 200)}')

    if net_loaded._backend == GraspQualityNet._BACKEND_MLP:
        succ = probs[y > 0.5]
        fail = probs[y < 0.5]
        print(f'\n=== DISTRIBUIÇÃO DE SCORES (MLP) ===')
        if len(succ):
            print(f'  SUCESSO: μ={succ.mean():.3f}  '
                  f'[{succ.min():.3f}, {succ.max():.3f}]')
        if len(fail):
            print(f'  FALHA:   μ={fail.mean():.3f}  '
                  f'[{fail.min():.3f}, {fail.max():.3f}]')

    pt_path = os.path.splitext(parsed.out)[0] + '.pt'
    final_path = pt_path if os.path.exists(pt_path) else parsed.out
    print(f'\n[OK] Modelo salvo em: {final_path}')


if __name__ == '__main__':
    main()
