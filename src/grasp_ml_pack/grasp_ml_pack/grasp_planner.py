"""
Nó de planejamento de grasp com verificação de viabilidade cinemática.

Para cada objeto detectado:
  1. Carrega candidatos do banco de dados YAML.
  2. Verifica IK (elimina candidatos inalcançáveis).
  3. Pontua cada candidato viável com GraspQualityNet (26 features).
  4. Seleciona o melhor e publica em /selected_grasp.

Topologia:
  /object_poses   (PoseArray + rótulos no frame_id) → entrada
  /selected_grasp (String JSON)                      → saída
"""

from __future__ import annotations

import json
import math
import os
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String

from .grasp_quality_net import GraspQualityNet, build_feature_vector_with_ik
from .kinematics import inverse_kinematics, manipulability, reach_margin


def _euler_from_quat(q) -> np.ndarray:
    """q = [x, y, z, w]  →  [roll, pitch, yaw]"""
    x, y, z, w = q
    roll  = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw   = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


class GraspPlannerNode(Node):

    def __init__(self):
        super().__init__('grasp_planner')

        self.declare_parameter('grasp_db_path',  '')
        self.declare_parameter('model_path',     '')
        self.declare_parameter('n_candidates',   6)
        self.declare_parameter('score_threshold', 0.40)
        self.declare_parameter('ik_check',       True)

        pkg = get_package_share_directory('grasp_ml_pack')
        db_path = self.get_parameter('grasp_db_path').value or \
                  os.path.join(pkg, 'config', 'grasp_database.yaml')
        model_path = self.get_parameter('model_path').value or \
                     os.path.join(pkg, 'models', 'grasp_quality.pkl')

        self._n_cand  = self.get_parameter('n_candidates').value
        self._thr     = self.get_parameter('score_threshold').value
        self._ik_chk  = self.get_parameter('ik_check').value

        self._db  = self._load_db(db_path)
        self._net = GraspQualityNet.load(model_path)

        mode = 'treinado (ML)' if self._net._model else 'heurístico'
        self.get_logger().info(
            f'GraspPlanner pronto | DB: {len(self._db)} objetos | modelo: {mode}')

        self._sub = self.create_subscription(
            PoseArray, '/object_poses', self._cb_poses, 10)
        self._pub = self.create_publisher(String, '/selected_grasp', 10)

    # ──────────────────────────────────────────────────────────────────
    @staticmethod
    def _load_db(path: str) -> dict:
        with open(path) as f:
            raw = yaml.safe_load(f)
        return raw.get('objects', {})

    # ──────────────────────────────────────────────────────────────────
    def _cb_poses(self, msg: PoseArray):
        parts = msg.header.frame_id.split('|', 1)
        if len(parts) < 2:
            return
        labels = json.loads(parts[1])

        best_score = -1.0
        best_grasp = None

        for pose, label in zip(msg.poses, labels):
            if label not in self._db:
                self.get_logger().warn(f'Label desconhecido: {label}')
                continue

            obj_pos  = np.array([pose.position.x,
                                  pose.position.y,
                                  pose.position.z])
            obj_quat = [pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w]
            euler    = _euler_from_quat(obj_quat)
            diam     = self._db[label].get('obj_diameter_m', 0.05)

            for cand in self._db[label].get('candidates', []):
                score, q_arm, ik_ok = self._score(
                    label, obj_pos, euler, diam, cand)

                if ik_ok and score >= self._thr and score > best_score:
                    best_score = score
                    best_grasp = {
                        'object_label':    label,
                        'object_position': obj_pos.tolist(),
                        'grasp_type':      cand['grasp_type'],
                        'approach_vector': cand['approach_vector'],
                        'grasp_offset':    cand['grasp_offset'],
                        'hand_config':     cand['hand_config'],
                        'q_arm_seed':      q_arm.tolist(),
                        'score':           score,
                    }

        if best_grasp:
            self._pub.publish(String(data=json.dumps(best_grasp)))
            self.get_logger().info(
                f'Selecionado: {best_grasp["object_label"]} '
                f'({best_grasp["grasp_type"]}) score={best_score:.3f}')
        else:
            self.get_logger().warn(
                'Nenhum grasp viável encontrado (IK + threshold).')

    # ──────────────────────────────────────────────────────────────────
    def _score(self, label: str, obj_pos: np.ndarray,
               euler: np.ndarray, diam: float,
               cand: dict) -> tuple[float, np.ndarray, bool]:
        """
        Pontua um candidato. Retorna (score, q_arm, ik_ok).
        Se ik_check=True e IK falhar → score = 0, ik_ok = False.
        """
        offset   = np.array(cand['grasp_offset'], float)
        av       = np.array(cand['approach_vector'], float)
        av       = av / (np.linalg.norm(av) + 1e-9)
        aperture = cand['hand_config'].get('aperture_norm', 0.5)
        gtype    = cand['grasp_type']
        grasp_pos = obj_pos + offset

        q_arm = np.zeros(6)
        ik_ok = True

        if self._ik_chk:
            q_arm, ik_ok = inverse_kinematics(grasp_pos, av)
            if not ik_ok:
                return 0.0, q_arm, False

        feats = build_feature_vector_with_ik(
            label, grasp_pos, obj_pos, euler, aperture, gtype, av,
            q_seed=q_arm if ik_ok else None)

        score = self._net.predict(feats)

        # Bônus de manipulabilidade: favorece configurações bem condicionadas
        if ik_ok:
            manip_bonus = min(manipulability(q_arm) / 0.05, 1.0) * 0.08
            score = float(np.clip(score + manip_bonus, 0.0, 1.0))

        return score, q_arm, ik_ok


def main(args=None):
    rclpy.init(args=args)
    node = GraspPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
