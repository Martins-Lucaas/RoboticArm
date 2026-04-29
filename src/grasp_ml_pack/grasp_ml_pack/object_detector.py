"""
Nó de detecção de objetos — Célula de Manufatura Biomédica.

Objetos monitorados na pick station da esteira:
  frasco — frasco de medicamento (cilindro largo, âmbar/laranja)
  tubo   — tubo de ensaio / centrífuga (cilindro estreito, azul)
  ampola — ampola farmacêutica (cilindro muito fino, verde)

Pipeline:
  Modo simulação (use_yolo=false): segmentação HSV por cor, confiável no Gazebo.
  Modo real (use_yolo=true): YOLOv8 para deploy no robô físico.

Publica:
  /detected_objects      (vision_msgs/Detection2DArray)
  /detector/debug_image  (sensor_msgs/Image, opcional)
  /detector/status       (std_msgs/String)
"""

import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import String as DetStatus
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose


# ── Câmera RGBD — parâmetros extraídos do conveyor_cell.world ──────────────
#
#   Pose SDF: x=1.15  y=0.0  z=1.70  roll=0  pitch=1.05  yaw=π
#   Coluna montada atrás da esteira (x=1.35) — fora do alcance do braço.
#   Camera olha de volta para a pick station (yaw=π = 180°).
#   Sensor:   hfov=1.2217 rad, resolução 848×480
#
_CAM_POS   = np.array([1.15, 0.0, 1.70])           # posição world (m)
_CAM_PITCH = 1.05                                   # rad — inclinação em torno de Y
_BELT_Z    = 0.806                                  # m — topo da correia (world)

_IMG_W, _IMG_H = 848, 480
_HFOV = 1.2217                                      # rad
_FX   = (_IMG_W / 2.0) / math.tan(_HFOV / 2.0)    # ≈ 603.5 px
_FY   = _FX
_CX   = _IMG_W / 2.0                               # 424.0 px
_CY   = _IMG_H / 2.0                               # 240.0 px

# Rotação world ← frame óptico ROS (Z = eixo óptico, X = direita, Y = baixo).
# Derivada de: R_z(π) @ R_y(pitch) aplicado à convenção Gazebo→ROS.
# yaw=π inverte os eixos X e Y do mundo relativos ao frame óptico.
_s = math.sin(_CAM_PITCH)  # ≈ 0.8674
_c = math.cos(_CAM_PITCH)  # ≈ 0.4976
_R_W_CAM = np.array([
    [ 0.0,  _s, -_c],   # Z_cam aponta em -x (volta à pick station)
    [ 1.0,  0.0, 0.0],
    [ 0.0, -_c, -_s],
])

# Semialturas dos objetos sobre a correia (metros)
_OBJ_HALF_HEIGHTS: dict[str, float] = {
    'frasco': 0.045,    # h=90 mm
    'tubo':   0.060,    # h=120 mm
    'ampola': 0.0375,   # h=75 mm
}


def _pixel_to_world(u_px: float, v_px: float, z_plane: float) -> np.ndarray:
    """
    Projeção inversa: pixel (u,v) → ponto world no plano horizontal z=z_plane.

    Retorna np.array([x, y, z_plane]) em metros (world frame).
    Retorna zeros se o raio não intersecta o plano (câmera olhando para cima).
    """
    d_cam   = np.array([(u_px - _CX) / _FX, (v_px - _CY) / _FY, 1.0])
    d_world = _R_W_CAM @ d_cam
    if abs(d_world[2]) < 1e-6:
        return np.zeros(3)
    t = (z_plane - _CAM_POS[2]) / d_world[2]
    if t < 0:
        return np.zeros(3)
    p = _CAM_POS + t * d_world
    # Sanidade: ponto deve estar dentro da área de trabalho esperada
    if not (0.2 < p[0] < 1.2 and -0.4 < p[1] < 0.4):
        return np.zeros(3)
    return np.array([float(p[0]), float(p[1]), float(z_plane)])


# ── Faixas HSV (OpenCV 0-180) para cada objeto na simulação Gazebo ─────────
#
#  frasco: diffuse (0.9, 0.50, 0.0) → âmbar/laranja   H≈10-26, S>120, V>80
#  tubo:   diffuse (0.1, 0.30, 0.9) → azul rico        H≈100-135, S>80, V>50
#  ampola: diffuse (0.1, 0.90, 0.2) → verde brilhante  H≈38-85, S>110, V>80
#
_HSV_RANGES = {
    'frasco': {'lower': np.array([8,  120,  80]),  'upper': np.array([26, 255, 255])},
    'tubo':   {'lower': np.array([100, 80,  50]),  'upper': np.array([135, 255, 255])},
    'ampola': {'lower': np.array([38, 110,  80]),  'upper': np.array([85,  255, 255])},
}

_MIN_AREA = 150   # px² — filtra ruído e reflexos pontuais

# Área máxima por objeto — proporcional ao tamanho real
_MAX_AREA = {
    'frasco': 18000,   # frasco largo → maior projeção na imagem
    'tubo':   6000,
    'ampola': 2500,    # ampola é muito fina → projeção pequena
}

# Filtros de forma (circularidade e aspect ratio)
_SHAPE = {
    'frasco': {'min_circ': 0.20, 'asp_min': 0.25, 'asp_max': 4.0},
    'tubo':   {'min_circ': 0.08, 'asp_min': 0.10, 'asp_max': 8.0},
    'ampola': {'min_circ': 0.05, 'asp_min': 0.05, 'asp_max': 12.0},
}


def _contour_shape_ok(cnt: np.ndarray, min_circ: float,
                      asp_min: float, asp_max: float) -> bool:
    area  = cv2.contourArea(cnt)
    perim = cv2.arcLength(cnt, True)
    if perim < 1.0:
        return False
    if 4 * np.pi * area / (perim ** 2) < min_circ:
        return False
    _, _, w, h = cv2.boundingRect(cnt)
    asp = w / max(h, 1)
    return asp_min <= asp <= asp_max


class ObjectDetectorNode(Node):

    def __init__(self):
        super().__init__('object_detector')

        self.declare_parameter('use_yolo', False,
            ParameterDescriptor(description='Use YOLOv8 instead of HSV'))
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.45)
        self.declare_parameter('publish_debug', True)

        self._use_yolo  = self.get_parameter('use_yolo').value
        self._conf_thr  = self.get_parameter('confidence_threshold').value
        self._pub_debug = self.get_parameter('publish_debug').value
        self._bridge    = CvBridge()
        self._model     = None

        if self._use_yolo:
            self._load_yolo(self.get_parameter('yolo_model').value)

        self._sub = self.create_subscription(
            Image, '/camera/color/image_raw', self._image_cb, 10)

        self._pub_det = self.create_publisher(
            Detection2DArray, '/detected_objects', 10)
        self._pub_status = self.create_publisher(DetStatus, '/detector/status', 10)

        if self._pub_debug:
            self._pub_img = self.create_publisher(
                Image, '/detector/debug_image', 10)

        cv2.namedWindow('Pick Station — Visão do Robô', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Pick Station — Visão do Robô', 800, 600)
        cv2.waitKey(1)

        self.get_logger().info(
            f'ObjectDetector pronto — modo: '
            f'{"YOLOv8" if self._use_yolo else "HSV-simulação"} | '
            f'objetos: frasco / tubo / ampola')

    # ------------------------------------------------------------------
    def _load_yolo(self, model_name: str):
        try:
            from ultralytics import YOLO
            self._model = YOLO(model_name)
            self.get_logger().info(f'YOLOv8 carregado: {model_name}')
        except ImportError:
            self.get_logger().error(
                'ultralytics não instalado. pip install ultralytics')
            raise

    # ------------------------------------------------------------------
    def _image_cb(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')

        if self._use_yolo:
            detections = self._detect_yolo(frame)
        else:
            detections = self._detect_hsv(frame)

        det_array = Detection2DArray()
        det_array.header = msg.header
        det_array.detections = detections
        self._pub_det.publish(det_array)

        mode_str = 'YOLO' if self._use_yolo else 'HSV'
        labels = [d.results[0].hypothesis.class_id for d in detections]
        status_txt = (f'DETECTADO: {", ".join(sorted(set(labels)))} [{mode_str}]'
                      if labels else f'SEM_DETECCAO [{mode_str}]')
        self._pub_status.publish(DetStatus(data=status_txt))

        if self._pub_debug:
            debug = self._draw_detections(frame.copy(), detections)
            self._pub_img.publish(self._bridge.cv2_to_imgmsg(debug, 'bgr8'))

    # ------------------------------------------------------------------
    def _detect_hsv(self, frame: np.ndarray) -> list:
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        detections = []

        kernel_open  = np.ones((7, 7), np.uint8)
        kernel_close = np.ones((9, 9), np.uint8)

        for label, rng in _HSV_RANGES.items():
            mask = cv2.inRange(hsv, rng['lower'], rng['upper'])
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel_open)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            sh = _SHAPE[label]
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < _MIN_AREA or area > _MAX_AREA[label]:
                    continue
                if not _contour_shape_ok(
                        cnt, sh['min_circ'], sh['asp_min'], sh['asp_max']):
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                score = min(area / 2500.0, 1.0)
                detections.append(self._make_detection(label, x, y, w, h, score))

        return detections

    # ------------------------------------------------------------------
    def _detect_yolo(self, frame: np.ndarray) -> list:
        # Mapeamento COCO → rótulos do projeto.
        # Frasco e ampola precisam de treinamento custom; bottle é fallback para tubo.
        _COCO_MAP = {
            'bottle': 'tubo',    # cilindro alto → tubo de ensaio
            'cup':    'frasco',  # recipiente largo → frasco
            'vase':   'frasco',  # vaso → frasco (silhueta similar)
        }
        results = self._model(frame, conf=self._conf_thr, verbose=False)
        detections = []
        for r in results:
            for box in r.boxes:
                coco_label = self._model.names[int(box.cls)]
                label = _COCO_MAP.get(coco_label)
                if label is None:
                    continue
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                detections.append(self._make_detection(
                    label, x1, y1, x2 - x1, y2 - y1, float(box.conf)))
        return detections

    # ------------------------------------------------------------------
    @staticmethod
    def _make_detection(label: str, x: int, y: int,
                        w: int, h: int, score: float) -> Detection2D:
        cx_px = float(x + w / 2)
        cy_px = float(y + h / 2)

        # Centro do objeto na correia: plano z = topo da correia + semialtura
        z_plane   = _BELT_Z + _OBJ_HALF_HEIGHTS.get(label, 0.0)
        world_pos = _pixel_to_world(cx_px, cy_px, z_plane)

        det = Detection2D()
        det.bbox.center.position.x = cx_px
        det.bbox.center.position.y = cy_px
        det.bbox.size_x = float(w)
        det.bbox.size_y = float(h)
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = label
        hyp.hypothesis.score    = score
        # Posição 3D world frame: usada pelo grasp_executor para IK
        hyp.pose.pose.position.x = world_pos[0]
        hyp.pose.pose.position.y = world_pos[1]
        hyp.pose.pose.position.z = world_pos[2]
        det.results.append(hyp)
        return det

    # ------------------------------------------------------------------
    def _draw_detections(self, frame: np.ndarray, detections: list) -> np.ndarray:
        # Paleta de cores por objeto (BGR)
        colors = {
            'frasco': (0, 120, 220),   # laranja
            'tubo':   (200, 80,  10),  # azul
            'ampola': (30,  200,  60), # verde
        }
        h_img, w_img = frame.shape[:2]
        img_cx, img_cy = w_img // 2, h_img // 2

        # Barra de status
        n_det     = len(detections)
        bar_color = (40, 200, 80) if n_det > 0 else (60, 60, 180)
        cv2.rectangle(frame, (0, 0), (w_img, 36), (20, 20, 30), -1)
        mode_label = 'YOLO' if self._use_yolo else 'HSV'
        status_txt = (f'DETECCAO  {n_det} obj  [{mode_label}]'
                      if n_det > 0 else f'AGUARDANDO  [{mode_label}]')
        cv2.putText(frame, status_txt, (8, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, bar_color, 2)
        cv2.circle(frame, (w_img - 20, 18), 9, bar_color, -1)
        cv2.circle(frame, (w_img - 20, 18), 9, (255, 255, 255), 1)

        # Indicador da pick station (mira)
        cross_c = (160, 160, 200)
        ps_x, ps_y = img_cx, img_cy   # aprox. projeção da pick station
        cv2.line(frame, (ps_x - 18, ps_y), (ps_x + 18, ps_y), cross_c, 1)
        cv2.line(frame, (ps_x, ps_y - 18), (ps_x, ps_y + 18), cross_c, 1)
        cv2.circle(frame, (ps_x, ps_y), 6, cross_c, 1)
        cv2.putText(frame, 'PICK', (ps_x + 10, ps_y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, cross_c, 1)

        # Contagem por classe
        counts: dict = {}
        for det in detections:
            lbl = det.results[0].hypothesis.class_id
            counts[lbl] = counts.get(lbl, 0) + 1
        y_cnt = 56
        for lbl, cnt in counts.items():
            cv2.putText(frame, f'{lbl}: {cnt}', (8, y_cnt),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50,
                        colors.get(lbl, (180, 180, 180)), 2)
            y_cnt += 22

        # Por detecção: bbox + label
        for det in detections:
            label = det.results[0].hypothesis.class_id
            score = det.results[0].hypothesis.score
            cx    = int(det.bbox.center.position.x)
            cy    = int(det.bbox.center.position.y)
            bw    = int(det.bbox.size_x)
            bh    = int(det.bbox.size_y)
            color = colors.get(label, (180, 180, 180))
            dim_c = tuple(max(v - 80, 0) for v in color)

            x1, y1 = cx - bw // 2, cy - bh // 2
            x2, y2 = cx + bw // 2, cy + bh // 2

            cv2.line(frame, (ps_x, ps_y), (cx, cy), dim_c, 1, cv2.LINE_AA)
            cv2.circle(frame, (cx, cy), 5, color, -1)
            cv2.line(frame, (0, cy), (w_img, cy), dim_c, 1)
            cv2.line(frame, (cx, 36), (cx, h_img), dim_c, 1)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

            # Cantos estilo HUD
            cl = max(min(bw, bh) // 5, 8)
            for px, py, dx, dy in [(x1, y1, 1, 1), (x2, y1, -1, 1),
                                    (x1, y2, 1, -1), (x2, y2, -1, -1)]:
                cv2.line(frame, (px, py), (px + dx * cl, py), color, 3)
                cv2.line(frame, (px, py), (px, py + dy * cl), color, 3)

            label_txt = f'{label}  {score:.2f}'
            (tw, th), _ = cv2.getTextSize(
                label_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.60, 2)
            lx = max(x1, 4)
            ly = max(y1 - th - 10, 38)
            cv2.rectangle(frame, (lx, ly), (lx + tw + 8, ly + th + 8), color, -1)
            cv2.putText(frame, label_txt, (lx + 4, ly + th + 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 0, 0), 2)

            bar_w = int((x2 - x1) * score)
            cv2.rectangle(frame, (x1, y2 + 3), (x1 + bar_w, y2 + 9), color, -1)
            cv2.rectangle(frame, (x1, y2 + 3), (x2, y2 + 9), color, 1)

        cv2.imshow('Pick Station — Visão do Robô', frame)
        cv2.waitKey(1)
        return frame


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
