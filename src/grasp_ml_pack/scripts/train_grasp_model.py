"""
Treina a GraspQualityNet com os dados coletados pelo Gazebo.

Faz pré-processamento, validação cruzada, treinamento completo e avalia
a importância das features cinemáticas vs. features originais.

Uso:
    ros2 run grasp_ml_pack train_model
    ros2 run grasp_ml_pack train_model --ros-args -p data:=models/training_data.npz
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


def main(args=None):
    parser = argparse.ArgumentParser(description='Treina GraspQualityNet')
    parser.add_argument('--data', default='models/training_data.npz',
                        help='Arquivo .npz gerado por generate_data')
    parser.add_argument('--out',  default='models/grasp_quality.pkl',
                        help='Destino do modelo treinado')
    parser.add_argument('--min-samples', type=int, default=50,
                        help='Mínimo de amostras para treinar')
    parsed = parser.parse_args(sys.argv[1:] if args is None else args)

    # ── 1. Carregar dados ──────────────────────────────────────────────
    if not os.path.exists(parsed.data):
        print(f'[ERRO] Arquivo não encontrado: {parsed.data}')
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
        print(f'[AVISO] Dimensão das features ({X.shape[1]}) ≠ {N_FEATURES}. '
              f'Dados podem ter sido gerados com versão anterior.')
        sys.exit(1)

    if X.shape[0] < parsed.min_samples:
        print(f'[AVISO] Poucos dados ({X.shape[0]} < {parsed.min_samples}). '
              f'Resultados podem ser instáveis.')
        if X.shape[0] < 20:
            print('[ERRO] Mínimo absoluto de 20 amostras. Abortar.')
            sys.exit(1)

    # ── 2. Estatísticas das features cinemáticas ───────────────────────
    print(f'\n=== FEATURES CINEMÁTICAS (amostras com IK convergido) ===')
    ik_mask = X[:, 25] > 0.5
    print(f'  IK convergido: {ik_mask.sum()}/{len(ik_mask)} '
          f'({100*ik_mask.mean():.1f}%)')
    if ik_mask.sum() > 0:
        manip_vals = X[ik_mask, 22]
        reach_vals = X[ik_mask, 23]
        print(f'  Manipulabilidade: μ={manip_vals.mean():.4f} '
              f'σ={manip_vals.std():.4f}')
        print(f'  Margem de alcance: μ={reach_vals.mean():.3f} '
              f'σ={reach_vals.std():.3f}')

    # ── 3. Treinar ─────────────────────────────────────────────────────
    print(f'\n=== TREINAMENTO ===')
    from grasp_ml_pack.grasp_quality_net import GraspQualityNet
    net = GraspQualityNet.train(X, y, save_path=parsed.out)

    # ── 4. Avaliação no conjunto completo ──────────────────────────────
    from sklearn.metrics import classification_report, roc_auc_score

    probs = net.predict_batch(X)
    preds = (probs > 0.5).astype(int)

    print(f'\n=== AVALIAÇÃO (conjunto de treino) ===')
    print(classification_report(y.astype(int), preds,
                                target_names=['falha', 'sucesso']))
    print(f'  AUC-ROC (treino): {roc_auc_score(y, probs):.3f}')

    # ── 5. Importância de features (apenas RandomForest) ───────────────
    if net._backend == 'rf' and net._model is not None:
        print(f'\n=== IMPORTÂNCIA: cinemáticas vs. originais ===')
        imp = net._model.feature_importances_
        original_imp  = imp[:16].sum()
        kinematic_imp = imp[16:].sum()
        print(f'  Features originais  [0:16]: {100*original_imp:.1f}%')
        print(f'  Features cinemáticas [16:]: {100*kinematic_imp:.1f}%')
        print(f'\n  Importância por feature:')
        for i, (name, val) in enumerate(zip(_FEATURE_NAMES, imp)):
            bar = '█' * int(val * 200)
            print(f'  [{i:2d}] {name:22s} {val:.4f}  {bar}')

    pt_path = os.path.splitext(parsed.out)[0] + '.pt'
    saved = pt_path if os.path.exists(pt_path) else parsed.out
    print(f'\n[OK] Modelo salvo em: {saved}')


if __name__ == '__main__':
    main()
