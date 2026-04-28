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
from std_msgs.msg import String as DetStatus
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose


# Faixas HSV para cada objeto na simulação Gazebo
#
# Análise das cores Gazebo (diffuse → HSV OpenCV 0-180):
#   ball   (0.9, 0.05, 0.05)  → H≈0,  S≈241, V≈230  — vermelho; reflexos especulares
#                                                        reduzem S para ~150 localmente
#   cup    (0.95,0.95,0.95)   → H=*, S≈0-50, V≈242  — branco/cinza quase neutro
#   pencil (1.0, 0.95, 0.0)   → H≈29, S≈255, V≈255  — amarelo puro
#   table  (0.72,0.55,0.38)   → H≈15, S≈120, V≈184  — marrom → S>85 exclui do copo
#
# Câmera a 60° de pitch: objetos na metade superior/central da imagem.
_HSV_RANGES = {
    'pencil': {'lower': np.array([18,  60,  80]),  'upper': np.array([40, 255, 255])},
    'cup':    {'lower': np.array([0,    0, 155]),  'upper': np.array([180, 85, 255])},
    'ball':   {'lower': np.array([0,  140,  60]),  'upper': np.array([12, 255, 255])},
}
# Vermelho tem wrapping no HSV: H=165-180 também é vermelho
_BALL_UPPER2 = {'lower': np.array([165, 140, 60]), 'upper': np.array([180, 255, 255])}

_MIN_AREA = 250   # px² — ignora ruído e reflexos pontuais

# Área máxima por objeto — evita detectar a mesa inteira como copo
_MAX_AREA = {'pencil': 8000, 'cup': 14000, 'ball': 9000}

# Filtros de forma para a bola (elimina falsos positivos retangulares/irregulares)
_BALL_MIN_CIRCULARITY = 0.35
_BALL_ASPECT_MIN = 0.30
_BALL_ASPECT_MAX = 3.0

# Filtros de forma para o copo (visão de cima ≈ elipse; exclui mesa grande/plana)
# Com câmera a 60° de pitch, o topo circular do copo (∅70mm) aparece como
# elipse com aspect ≈ w/h ≈ 1/cos(60°) = 2.0
_CUP_MIN_CIRCULARITY = 0.20
_CUP_ASPECT_MIN = 0.30
_CUP_ASPECT_MAX = 3.5


def _contour_shape_ok(cnt: np.ndarray,
                      min_circ: float,
                      asp_min: float,
                      asp_max: float) -> bool:
    area = cv2.contourArea(cnt)
    perim = cv2.arcLength(cnt, True)
    if perim < 1.0:
        return False
    if 4 * np.pi * area / (perim ** 2) < min_circ:
        return False
    _, _, w, h = cv2.boundingRect(cnt)
    asp = w / max(h, 1)
    return asp_min <= asp <= asp_max


def _is_ball_shaped(cnt: np.ndarray) -> bool:
    return _contour_shape_ok(cnt, _BALL_MIN_CIRCULARITY, _BALL_ASPECT_MIN, _BALL_ASPECT_MAX)


def _is_cup_shaped(cnt: np.ndarray) -> bool:
    return _contour_shape_ok(cnt, _CUP_MIN_CIRCULARITY, _CUP_ASPECT_MIN, _CUP_ASPECT_MAX)


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

        self._pub_status = self.create_publisher(DetStatus, '/detector/status', 10)

        # Cria a janela antes de receber a primeira imagem para garantir que
        # ela apareça mesmo em ambientes onde a janela pode ser lazy-initialized
        cv2.namedWindow('Camera — O que o robo ve', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Camera — O que o robo ve', 800, 600)
        cv2.waitKey(1)

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

        # Feedback de status textual
        mode_str = 'YOLO' if self._use_yolo else 'HSV'
        labels = [d.results[0].hypothesis.class_id for d in detections]
        if labels:
            status_txt = f'DETECTADO: {", ".join(sorted(set(labels)))} [{mode_str}]'
        else:
            status_txt = f'SEM_DETECCAO [{mode_str}]'
        self._pub_status.publish(DetStatus(data=status_txt))

        if self._pub_debug:
            debug = self._draw_detections(frame.copy(), detections)
            self._pub_img.publish(self._bridge.cv2_to_imgmsg(debug, 'bgr8'))

    # ------------------------------------------------------------------
    def _detect_hsv(self, frame: np.ndarray) -> list:
        # Leve desfoque gaussiano reduz ruído de textura antes da segmentação HSV
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        detections = []

        kernel_open  = np.ones((7, 7), np.uint8)   # remove ruído pequeno
        kernel_close = np.ones((9, 9), np.uint8)   # preenche buracos internos

        for label, rng in _HSV_RANGES.items():
            mask = cv2.inRange(hsv, rng['lower'], rng['upper'])
            if label == 'ball':
                mask |= cv2.inRange(hsv, _BALL_UPPER2['lower'], _BALL_UPPER2['upper'])

            # Open: elimina manchas pequenas; Close: une fragmentos do mesmo objeto
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel_open)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < _MIN_AREA:
                    continue
                if area > _MAX_AREA[label]:
                    continue

                if label == 'ball' and not _is_ball_shaped(cnt):
                    continue
                if label == 'cup' and not _is_cup_shaped(cnt):
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                det = self._make_detection(label, x, y, w, h,
                                           score=min(area / 3000.0, 1.0))
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
    def _draw_detections(self, frame: np.ndarray, detections: list) -> np.ndarray:
        colors = {'pencil': (0, 220, 220), 'cup': (220, 220, 0), 'ball': (0, 60, 220)}
        h_img, w_img = frame.shape[:2]
        img_cx, img_cy = w_img // 2, h_img // 2   # boresight da câmera

        # ── Barra de status no topo ─────────────────────────────────
        n_det     = len(detections)
        bar_color = (20, 160, 60) if n_det > 0 else (160, 60, 20)
        cv2.rectangle(frame, (0, 0), (w_img, 34), (20, 20, 20), -1)
        mode_label = 'YOLO' if self._use_yolo else 'HSV'
        status_txt = (f'DETECCAO  {n_det} obj  [{mode_label}]'
                      if n_det > 0 else f'AGUARDANDO  [{mode_label}]')
        cv2.putText(frame, status_txt, (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.60, bar_color, 2)
        cv2.circle(frame, (w_img - 20, 17), 9, bar_color, -1)
        cv2.circle(frame, (w_img - 20, 17), 9, (255, 255, 255), 1)

        # ── Mira central da câmera (boresight) ──────────────────────
        cross_c = (180, 180, 180)
        cv2.line(frame, (img_cx - 15, img_cy), (img_cx + 15, img_cy), cross_c, 1)
        cv2.line(frame, (img_cx, img_cy - 15), (img_cx, img_cy + 15), cross_c, 1)
        cv2.circle(frame, (img_cx, img_cy), 5, cross_c, 1)

        # ── Contagem por classe ──────────────────────────────────────
        counts: dict = {}
        for det in detections:
            lbl = det.results[0].hypothesis.class_id
            counts[lbl] = counts.get(lbl, 0) + 1
        y_cnt = 58
        for lbl, cnt in counts.items():
            cv2.putText(frame, f'{lbl}: {cnt}', (8, y_cnt),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                        colors.get(lbl, (180, 180, 180)), 2)
            y_cnt += 20

        # ── Por detecção: linhas + bbox + label ─────────────────────
        for det in detections:
            label = det.results[0].hypothesis.class_id
            score = det.results[0].hypothesis.score
            cx = int(det.bbox.center.position.x)
            cy = int(det.bbox.center.position.y)
            bw = int(det.bbox.size_x)
            bh = int(det.bbox.size_y)
            color  = colors.get(label, (180, 180, 180))
            dim_c  = tuple(max(v - 80, 0) for v in color)   # versão mais escura

            x1, y1 = cx - bw // 2, cy - bh // 2
            x2, y2 = cx + bw // 2, cy + bh // 2

            # -- Linha de detecção: boresight → centro do objeto ------
            cv2.line(frame, (img_cx, img_cy), (cx, cy), dim_c, 1,
                     cv2.LINE_AA)
            # Pequeno ponto onde a linha toca o objeto
            cv2.circle(frame, (cx, cy), 5, color, -1)

            # -- Crosshair completo passando pelo centro do objeto ----
            cv2.line(frame, (0, cy), (w_img, cy), dim_c, 1)
            cv2.line(frame, (cx, 34), (cx, h_img), dim_c, 1)

            # -- Bounding box ----------------------------------------
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

            # Cantos destacados (estilo HUD)
            cl = max(min(bw, bh) // 5, 8)
            for px, py, dx, dy in [(x1, y1, 1, 1), (x2, y1, -1, 1),
                                    (x1, y2, 1, -1), (x2, y2, -1, -1)]:
                cv2.line(frame, (px, py), (px + dx * cl, py), color, 3)
                cv2.line(frame, (px, py), (px, py + dy * cl), color, 3)

            # -- Label com fundo sólido -------------------------------
            label_txt = f'{label}  {score:.2f}'
            (tw, th), _ = cv2.getTextSize(
                label_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.60, 2)
            lx = max(x1, 4)
            ly = max(y1 - th - 10, 36)
            cv2.rectangle(frame, (lx, ly), (lx + tw + 8, ly + th + 8),
                          color, -1)
            cv2.putText(frame, label_txt, (lx + 4, ly + th + 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 0, 0), 2)

            # -- Barra de confiança ----------------------------------
            bar_w = int((x2 - x1) * score)
            cv2.rectangle(frame, (x1, y2 + 3), (x1 + bar_w, y2 + 9),
                          color, -1)
            cv2.rectangle(frame, (x1, y2 + 3), (x2, y2 + 9), color, 1)

        # Mostra em janela local para "ver o que o robô vê"
        cv2.imshow('Camera — O que o robo ve', frame)
        cv2.waitKey(1)

        return frame


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
