"""
Coleta de dados de treinamento no Gazebo com features cinemáticas completas.

Para cada amostra:
  1. Amostra configuração aleatória de grasp (objeto, tipo, posição, abordagem)
  2. Resolve IK → descarta se inalcançável (label automático = falha)
  3. Computa features cinemáticas (manipulabilidade, margens)
  4. Envia comando ao Gazebo e aguarda /grasp_result
  5. Salva vetor de 26 features + label (1=sucesso / 0=falha)

Saída: training_data.npz com arrays X (N×26) e y (N,)

Uso:
    ros2 run grasp_ml_pack generate_data
    ros2 run grasp_ml_pack generate_data --ros-args -p n_samples:=1000
"""

from __future__ import annotations

import json
import os
import random
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from grasp_ml_pack.grasp_quality_net import (
    build_feature_vector_with_ik, N_FEATURES)
from grasp_ml_pack.kinematics import (
    inverse_kinematics, manipulability, reach_margin,
    singularity_distances, HAND_CONFIGS)

# ─────────────────────────────────────────────────────────────────────
# Espaço de amostragem
# ─────────────────────────────────────────────────────────────────────
_OBJ_GRASP = {
    'pencil': ['pinch'],
    'cup':    ['cylindrical'],
    'ball':   ['spherical'],
}
_OBJ_DIAMETERS = {'pencil': 0.007, 'cup': 0.070, 'ball': 0.064}

# Posições no frame do BASE_LINK do robô (robô spawnado em world Z=0.375 m).
# Bancada: face frontal x=0.38 m, objetos reais em x=0.61-0.81 m
# Z dos objetos: world Z ≈ 0.800 m  →  base-frame Z = 0.800 - 0.375 = 0.425 m
_X_RANGE = (0.55, 0.85)
_Y_RANGE = (-0.18, 0.18)
_Z_OBJ   = 0.425   # base-frame: world Z (0.800) - pedestal (0.375)

# Variações de parâmetros
_APERTURE_RANGE   = (0.30, 0.95)
_OFFSET_STD       = 0.018   # m — perturbação gaussiana no offset de preensão
_APPROACH_PERTURB = 0.18    # rad — perturbação no vetor de abordagem padrão

# Descarta grasp se singularidade de pulso < este limiar
_WRIST_SING_MIN = 0.20


class DataCollectorNode(Node):

    def __init__(self):
        super().__init__('data_collector')
        self.declare_parameter('n_samples',   500)
        self.declare_parameter('output_path', '')
        self.declare_parameter('ik_only',     False)

        n_samples = self.get_parameter('n_samples').value
        out_path  = self.get_parameter('output_path').value
        self._ik_only = self.get_parameter('ik_only').value

        if not out_path:
            pkg = get_package_share_directory('grasp_ml_pack')
            models_dir = os.path.join(os.path.dirname(pkg), 'models')
            out_path = os.path.join(models_dir, 'training_data.npz')

        self._n_target   = n_samples
        self._out_path   = out_path
        self._X: list    = []
        self._y: list    = []
        self._n_ik_skip  = 0    # amostras descartadas por IK inviável
        self._pending    = False
        self._cur_feat: np.ndarray | None = None

        self._pub_grasp = self.create_publisher(String, '/selected_grasp', 10)
        self._sub_result = self.create_subscription(
            String, '/grasp_result', self._cb_result, 10)

        self.get_logger().info(
            f'DataCollector: {n_samples} amostras → {out_path} '
            f'| ik_only={self._ik_only}')
        self.create_timer(2.0, self._send_next)

    # ──────────────────────────────────────────────────────────────────
    def _send_next(self):
        if self._pending or len(self._X) >= self._n_target:
            return

        obj       = random.choice(list(_OBJ_GRASP.keys()))
        gtype     = random.choice(_OBJ_GRASP[obj])
        aperture  = random.uniform(*_APERTURE_RANGE)
        diam      = _OBJ_DIAMETERS[obj]

        # Posição do objeto na bancada
        obj_pos = np.array([
            random.uniform(*_X_RANGE),
            random.uniform(*_Y_RANGE),
            _Z_OBJ,
        ])

        # Offset de preensão relativo ao centróide (pequena perturbação)
        offset = np.random.normal(0.0, _OFFSET_STD, 3)
        offset[2] = abs(offset[2]) + 0.01   # sempre levemente acima

        # Vetor de abordagem (base: de cima para baixo)
        av = np.array([0.0, 0.0, -1.0])
        av += np.random.normal(0.0, _APPROACH_PERTURB, 3)
        av /= np.linalg.norm(av) + 1e-9

        # Orientação do objeto (perturbação em Euler)
        euler = np.random.normal(0.0, 0.12, 3)

        grasp_pos = obj_pos + offset

        # ── Verificação IK (pré-filtragem cinemática) ─────────────────
        q_arm, ik_ok = inverse_kinematics(grasp_pos, av, elbow_up=True)

        if not ik_ok:
            self._n_ik_skip += 1
            # Registra como falha com features cinemáticas zeradas
            if self._ik_only:
                return   # descarta completamente se ik_only=True
            feat = build_feature_vector_with_ik(
                obj, grasp_pos, obj_pos, euler, aperture, gtype, av)
            self._X.append(feat)
            self._y.append(0)
            return

        # Verifica singularidade de pulso
        _, _, wrist_d = singularity_distances(q_arm)
        if wrist_d < _WRIST_SING_MIN:
            self._n_ik_skip += 1
            return   # configuração singular — pula sem registrar

        # ── Feature vector completo ───────────────────────────────────
        self._cur_feat = build_feature_vector_with_ik(
            obj, grasp_pos, obj_pos, euler, aperture, gtype, av,
            q_seed=q_arm)

        # ── Envia ao Gazebo para execução real ────────────────────────
        cmd = {
            'object_label':    obj,
            'object_position': obj_pos.tolist(),
            'grasp_type':      gtype,
            'approach_vector': av.tolist(),
            'grasp_offset':    offset.tolist(),
            'hand_config':     {'aperture_norm': aperture},
            'q_arm_seed':      q_arm.tolist(),
            'score':           0.5,
        }
        self._pub_grasp.publish(String(data=json.dumps(cmd)))
        self._pending = True

    # ──────────────────────────────────────────────────────────────────
    def _cb_result(self, msg: String):
        result = json.loads(msg.data)
        if self._cur_feat is not None:
            self._X.append(self._cur_feat)
            self._y.append(1 if result['success'] else 0)
            n = len(self._X)
            if n % 50 == 0:
                pos = sum(self._y)
                self.get_logger().info(
                    f'Amostras: {n}/{self._n_target} | '
                    f'sucesso: {100*pos/n:.1f}% | '
                    f'IK skip: {self._n_ik_skip}')

        self._cur_feat = None
        self._pending  = False

        if len(self._X) >= self._n_target:
            self._save()

    # ──────────────────────────────────────────────────────────────────
    def _save(self):
        X = np.array(self._X, dtype=float)
        y = np.array(self._y, dtype=float)
        os.makedirs(os.path.dirname(self._out_path) or '.', exist_ok=True)
        np.savez(self._out_path, X=X, y=y)

        pos = int(y.sum())
        neg = len(y) - pos
        self.get_logger().info(
            f'\n=== COLETA CONCLUÍDA ===\n'
            f'  Total:      {len(y)} amostras\n'
            f'  Sucessos:   {pos} ({100*pos/len(y):.1f}%)\n'
            f'  Falhas:     {neg} ({100*neg/len(y):.1f}%)\n'
            f'  IK skip:    {self._n_ik_skip}\n'
            f'  Shape X:    {X.shape}\n'
            f'  Salvo em:   {self._out_path}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectorNode()
    rclpy.spin(node)
