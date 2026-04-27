"""
Nó de estimativa de pose 6D.

Combina:
  - Detecções 2D (/detected_objects) com
  - Imagem de profundidade (/camera/depth/image_raw) e
  - Parâmetros intrínsecos (/camera/color/camera_info)

para estimar a pose 3D de cada objeto via RANSAC + ajuste de primitivas
geométricas (cilindro → lápis/copo, esfera → bola).

Publica: /object_poses  (geometry_msgs/PoseArray com campos extras como JSON
          no frame_id para transportar rótulo + confiança)
         /pose_estimator/debug_markers  (visualization_msgs/MarkerArray)
"""

import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
from builtin_interfaces.msg import Duration


def _rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Convert 3×3 rotation matrix to quaternion [x, y, z, w]."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


class PoseEstimatorNode(Node):

    def __init__(self):
        super().__init__('pose_estimator')

        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('depth_scale', 0.001)   # mm → m para RealSense
        self._camera_frame = self.get_parameter('camera_frame').value
        self._depth_scale = self.get_parameter('depth_scale').value

        self._bridge = CvBridge()
        self._K = None        # matriz intrínseca 3×3
        self._depth_img = None
        self._latest_detections: Detection2DArray | None = None

        qos = QoSProfile(depth=5,
                         reliability=ReliabilityPolicy.BEST_EFFORT)

        self._sub_info = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._info_cb, 10)
        self._sub_depth = self.create_subscription(
            Image, '/camera/depth/image_raw', self._depth_cb, qos)
        self._sub_det = self.create_subscription(
            Detection2DArray, '/detected_objects', self._det_cb, 10)

        self._pub_poses = self.create_publisher(PoseArray, '/object_poses', 10)
        self._pub_markers = self.create_publisher(
            MarkerArray, '/pose_estimator/debug_markers', 10)

        self.get_logger().info('PoseEstimator pronto.')

    # ------------------------------------------------------------------
    def _info_cb(self, msg: CameraInfo):
        if self._K is None:
            self._K = np.array(msg.k).reshape(3, 3)

    def _depth_cb(self, msg: Image):
        self._depth_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def _det_cb(self, msg: Detection2DArray):
        self._latest_detections = msg
        self._process()

    # ------------------------------------------------------------------
    def _process(self):
        if (self._K is None or self._depth_img is None
                or self._latest_detections is None):
            return

        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = self._camera_frame

        markers = MarkerArray()
        marker_id = 0

        for det in self._latest_detections.detections:
            label = det.results[0].hypothesis.class_id
            cx = int(det.bbox.center.position.x)
            cy = int(det.bbox.center.position.y)
            bw = int(det.bbox.size_x)
            bh = int(det.bbox.size_y)

            # Extrair ROI de profundidade
            x0 = max(cx - bw // 2, 0)
            y0 = max(cy - bh // 2, 0)
            x1 = min(cx + bw // 2, self._depth_img.shape[1])
            y1 = min(cy + bh // 2, self._depth_img.shape[0])
            depth_roi = self._depth_img[y0:y1, x0:x1].astype(float) * self._depth_scale

            valid = depth_roi[depth_roi > 0.05]
            if valid.size < 20:
                continue
            z_med = float(np.median(valid))

            # Back-project centro da bbox para 3D
            px = (cx - self._K[0, 2]) * z_med / self._K[0, 0]
            py = (cy - self._K[1, 2]) * z_med / self._K[1, 1]

            position = np.array([px, py, z_med])
            orientation, dims = self._fit_primitive(
                label, depth_roi, x0, y0, x1, y1, z_med)

            pose = Pose()
            pose.position = Point(x=position[0], y=position[1], z=position[2])
            pose.orientation = Quaternion(
                x=orientation[0], y=orientation[1],
                z=orientation[2], w=orientation[3])

            # Codifica label no frame_id do pose array via comentário indexado
            pose_array.poses.append(pose)
            pose_array.header.frame_id = self._camera_frame  # restaura

            markers.markers.append(
                self._make_marker(marker_id, label, position, dims,
                                  pose_array.header))
            marker_id += 1

        # Transporta rótulos como JSON no frame_id (workaround sem msg custom)
        labels = [d.results[0].hypothesis.class_id
                  for d in self._latest_detections.detections
                  if any(True for _ in [None])]
        pose_array.header.frame_id = (
            self._camera_frame + '|' + json.dumps(labels))

        self._pub_poses.publish(pose_array)
        self._pub_markers.publish(markers)

    # ------------------------------------------------------------------
    def _fit_primitive(self, label: str, depth_roi: np.ndarray,
                       x0, y0, x1, y1, z_med: float):
        """
        Ajusta primitiva geométrica à nuvem de pontos da ROI.
        Retorna (quaternion [x,y,z,w], dimensões dict).
        """
        h, w = depth_roi.shape
        ys, xs = np.mgrid[0:h, 0:w]
        zs = depth_roi.copy()
        zs[zs < 0.05] = np.nan

        px = ((xs + x0) - self._K[0, 2]) * zs / self._K[0, 0]
        py = ((ys + y0) - self._K[1, 2]) * zs / self._K[1, 1]

        mask = ~np.isnan(zs)
        if mask.sum() < 10:
            return np.array([0.0, 0.0, 0.0, 1.0]), {}

        pts = np.stack([px[mask], py[mask], zs[mask]], axis=1)

        if label == 'ball':
            return self._fit_sphere(pts)
        else:
            return self._fit_cylinder(pts, label)

    # ------------------------------------------------------------------
    @staticmethod
    def _fit_sphere(pts: np.ndarray):
        """PCA da nuvem → orientação identidade, raio estimado."""
        center = pts.mean(axis=0)
        centered = pts - center
        radius = float(np.percentile(np.linalg.norm(centered, axis=1), 90))
        return np.array([0.0, 0.0, 0.0, 1.0]), {'radius': radius}

    @staticmethod
    def _fit_cylinder(pts: np.ndarray, label: str):
        """
        PCA 3D → eixo principal do cilindro.
        O primeiro componente é o eixo longitudinal.
        """
        center = pts.mean(axis=0)
        centered = pts - center
        _, _, Vt = np.linalg.svd(centered, full_matrices=False)
        axis = Vt[0]  # eixo principal

        # Alinha eixo com Z do mundo (câmera olha para baixo → Z é profundidade)
        z_ref = np.array([0.0, 0.0, 1.0])
        v = np.cross(axis, z_ref)
        s = np.linalg.norm(v)
        c = np.dot(axis, z_ref)
        if s < 1e-6:
            return np.array([0.0, 0.0, 0.0, 1.0]), {}

        vx = np.array([[0, -v[2], v[1]],
                       [v[2], 0, -v[0]],
                       [-v[1], v[0], 0]])
        R = np.eye(3) + vx + vx @ vx * ((1 - c) / (s * s))
        quat = _rotation_matrix_to_quaternion(R)

        # Estima dimensões
        proj = centered @ axis
        length = float(proj.max() - proj.min())
        radial = centered - np.outer(proj, axis)
        radius = float(np.percentile(np.linalg.norm(radial, axis=1), 90))
        return quat, {'length': length, 'radius': radius}

    # ------------------------------------------------------------------
    def _make_marker(self, mid: int, label: str, pos: np.ndarray,
                     dims: dict, header) -> Marker:
        m = Marker()
        m.header = header
        m.id = mid
        m.action = Marker.ADD
        m.pose.position = Point(x=pos[0], y=pos[1], z=pos[2])
        m.pose.orientation.w = 1.0
        m.lifetime = Duration(sec=1)

        if label == 'ball':
            m.type = Marker.SPHERE
            r = dims.get('radius', 0.04)
            m.scale.x = m.scale.y = m.scale.z = r * 2
            m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.1, 0.1, 0.8
        elif label == 'cup':
            m.type = Marker.CYLINDER
            m.scale.x = m.scale.y = dims.get('radius', 0.035) * 2
            m.scale.z = dims.get('length', 0.10)
            m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.9, 0.9, 0.8
        else:  # pencil
            m.type = Marker.CYLINDER
            m.scale.x = m.scale.y = dims.get('radius', 0.006) * 2
            m.scale.z = dims.get('length', 0.15)
            m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.9, 0.1, 0.8
        return m


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
