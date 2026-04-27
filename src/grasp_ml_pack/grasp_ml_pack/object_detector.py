"""
Nó de detecção de objetos.

Pipeline:
  - Modo simulação (padrão): segmentação HSV por cor — lápis=amarelo,
    copo=branco, bola=vermelho — confiável no Gazebo sem necessidade de GPU.
  - Modo real (--use-yolo): YOLOv8 (ultralytics) para deploy no robô físico.

Publica: /detected_objects  (vision_msgs/Detection2DArray)
         /detector/debug_image  (sensor_msgs/Image, opcional)
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose


# Faixas HSV para cada objeto na simulação Gazebo
_HSV_RANGES = {
    'pencil': {'lower': np.array([20, 100, 100]),   'upper': np.array([35, 255, 255])},
    'cup':    {'lower': np.array([0,   0,   200]),   'upper': np.array([180, 30,  255])},
    'ball':   {'lower': np.array([0,  120, 70]),     'upper': np.array([10, 255, 255])},
}
# A bola pode ter matiz "wrapping" (vermelho vai de 170-180 também)
_BALL_UPPER2 = {'lower': np.array([170, 120, 70]), 'upper': np.array([180, 255, 255])}

_MIN_AREA = 300   # px² — ignora ruído


class ObjectDetectorNode(Node):

    def __init__(self):
        super().__init__('object_detector')

        self.declare_parameter('use_yolo', False,
            ParameterDescriptor(description='Use YOLOv8 instead of HSV (requires ultralytics)'))
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.45)
        self.declare_parameter('publish_debug', True)

        self._use_yolo = self.get_parameter('use_yolo').value
        self._conf_thr = self.get_parameter('confidence_threshold').value
        self._pub_debug = self.get_parameter('publish_debug').value
        self._bridge = CvBridge()
        self._model = None

        if self._use_yolo:
            self._load_yolo(self.get_parameter('yolo_model').value)

        self._sub = self.create_subscription(
            Image, '/camera/color/image_raw', self._image_cb, 10)

        self._pub_det = self.create_publisher(
            Detection2DArray, '/detected_objects', 10)

        if self._pub_debug:
            self._pub_img = self.create_publisher(
                Image, '/detector/debug_image', 10)

        self.get_logger().info(
            f'ObjectDetector pronto — modo: {"YOLOv8" if self._use_yolo else "HSV-simulação"}')

    # ------------------------------------------------------------------
    def _load_yolo(self, model_name: str):
        try:
            from ultralytics import YOLO
            self._model = YOLO(model_name)
            self.get_logger().info(f'YOLOv8 carregado: {model_name}')
        except ImportError:
            self.get_logger().error(
                'ultralytics não instalado. Instale com: pip install ultralytics')
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

        if self._pub_debug:
            debug = self._draw_detections(frame.copy(), detections)
            self._pub_img.publish(self._bridge.cv2_to_imgmsg(debug, 'bgr8'))

    # ------------------------------------------------------------------
    def _detect_hsv(self, frame: np.ndarray) -> list:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detections = []

        for label, rng in _HSV_RANGES.items():
            mask = cv2.inRange(hsv, rng['lower'], rng['upper'])
            if label == 'ball':
                mask |= cv2.inRange(hsv, _BALL_UPPER2['lower'], _BALL_UPPER2['upper'])

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                    np.ones((5, 5), np.uint8))
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < _MIN_AREA:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                det = self._make_detection(label, x, y, w, h,
                                           score=min(area / 5000.0, 1.0))
                detections.append(det)

        return detections

    # ------------------------------------------------------------------
    def _detect_yolo(self, frame: np.ndarray) -> list:
        # Mapeamento COCO → rótulos do projeto
        _COCO_MAP = {
            'cup': 'cup',          # classe 41
            'sports ball': 'ball', # classe 32
            'bottle': 'pencil',    # fallback — treinar classe custom para lápis
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
                det = self._make_detection(
                    label, x1, y1, x2 - x1, y2 - y1, float(box.conf))
                detections.append(det)
        return detections

    # ------------------------------------------------------------------
    @staticmethod
    def _make_detection(label: str, x: int, y: int,
                        w: int, h: int, score: float) -> Detection2D:
        det = Detection2D()
        det.bbox.center.position.x = float(x + w / 2)
        det.bbox.center.position.y = float(y + h / 2)
        det.bbox.size_x = float(w)
        det.bbox.size_y = float(h)
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = label
        hyp.hypothesis.score = score
        det.results.append(hyp)
        return det

    # ------------------------------------------------------------------
    @staticmethod
    def _draw_detections(frame: np.ndarray, detections: list) -> np.ndarray:
        colors = {'pencil': (0, 220, 220), 'cup': (220, 220, 0), 'ball': (0, 60, 220)}
        for det in detections:
            label = det.results[0].hypothesis.class_id
            score = det.results[0].hypothesis.score
            cx = int(det.bbox.center.position.x)
            cy = int(det.bbox.center.position.y)
            w = int(det.bbox.size_x)
            h = int(det.bbox.size_y)
            color = colors.get(label, (180, 180, 180))
            cv2.rectangle(frame, (cx - w//2, cy - h//2),
                          (cx + w//2, cy + h//2), color, 2)
            cv2.putText(frame, f'{label} {score:.2f}',
                        (cx - w//2, cy - h//2 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        return frame


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
