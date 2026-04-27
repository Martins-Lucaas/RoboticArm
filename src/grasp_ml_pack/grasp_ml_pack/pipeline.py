"""
Nó orquestrador do pipeline de grasp autônomo.

Máquina de estados:
  IDLE → DETECTING → ESTIMATING → PLANNING → EXECUTING → VERIFYING → IDLE

Monitora o sistema inteiro, registra métricas e aplica watchdogs.
"""

import json
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseArray


class State(Enum):
    IDLE       = auto()
    DETECTING  = auto()
    ESTIMATING = auto()
    PLANNING   = auto()
    EXECUTING  = auto()
    VERIFYING  = auto()
    ERROR      = auto()


_TIMEOUT = {
    State.DETECTING:  5.0,
    State.ESTIMATING: 3.0,
    State.PLANNING:   3.0,
    State.EXECUTING:  30.0,
}


class PipelineNode(Node):

    def __init__(self):
        super().__init__('grasp_pipeline')

        self.declare_parameter('loop_rate_hz', 10.0)
        self.declare_parameter('max_attempts', 3)
        rate = self.get_parameter('loop_rate_hz').value
        self._max_attempts = self.get_parameter('max_attempts').value

        self._state = State.IDLE
        self._state_entry_time = time.time()
        self._attempt = 0
        self._last_detection: Detection2DArray | None = None
        self._last_poses: PoseArray | None = None
        self._last_grasp: dict | None = None
        self._metrics: list[dict] = []

        # Subscriptions (passivas — apenas cache)
        self.create_subscription(
            Detection2DArray, '/detected_objects', self._on_detection, 10)
        self.create_subscription(
            PoseArray, '/object_poses', self._on_poses, 10)
        self.create_subscription(
            String, '/selected_grasp', self._on_grasp, 10)
        self.create_subscription(
            String, '/grasp_result', self._on_result, 10)

        # Publisher de status
        self._pub_status = self.create_publisher(String, '/pipeline/status', 10)

        # Loop principal
        self.create_timer(1.0 / rate, self._step)
        self.get_logger().info('Pipeline autônomo iniciado.')

    # ------------------------------------------------------------------
    # Callbacks passivos
    def _on_detection(self, msg):  self._last_detection = msg
    def _on_poses(self,     msg):  self._last_poses = msg
    def _on_grasp(self,     msg):  self._last_grasp = json.loads(msg.data)

    def _on_result(self, msg: String):
        result = json.loads(msg.data)
        self._metrics.append({
            'attempt': self._attempt,
            'timestamp': time.time(),
            **result,
        })
        if result['success']:
            self.get_logger().info(
                f'[SUCESSO] {result["object_label"]} '
                f'({result["grasp_type"]}) score={result["score"]:.2f}')
            self._transition(State.IDLE)
        else:
            self.get_logger().warn(
                f'[FALHA] tentativa {self._attempt}/{self._max_attempts}')
            if self._attempt >= self._max_attempts:
                self.get_logger().error('Máximo de tentativas atingido.')
                self._transition(State.ERROR)
            else:
                self._transition(State.DETECTING)

    # ------------------------------------------------------------------
    def _step(self):
        elapsed = time.time() - self._state_entry_time
        timeout = _TIMEOUT.get(self._state)

        if timeout and elapsed > timeout:
            self.get_logger().warn(
                f'Timeout no estado {self._state.name} ({elapsed:.1f}s)')
            self._transition(State.ERROR if self._attempt >= self._max_attempts
                             else State.DETECTING)
            return

        status = {'state': self._state.name, 'attempt': self._attempt}
        self._pub_status.publish(String(data=json.dumps(status)))

        match self._state:
            case State.IDLE:
                self._attempt = 0
                self._transition(State.DETECTING)

            case State.DETECTING:
                if (self._last_detection is not None
                        and len(self._last_detection.detections) > 0):
                    self._transition(State.ESTIMATING)

            case State.ESTIMATING:
                if self._last_poses is not None and len(self._last_poses.poses) > 0:
                    self._transition(State.PLANNING)

            case State.PLANNING:
                if self._last_grasp is not None:
                    self._attempt += 1
                    self._transition(State.EXECUTING)

            case State.EXECUTING:
                pass   # aguarda callback _on_result

            case State.VERIFYING:
                self._transition(State.IDLE)

            case State.ERROR:
                self.get_logger().error(
                    f'Pipeline em estado de erro. Métricas: {self._metrics}')
                # Aguarda intervenção — não re-inicia automaticamente

    # ------------------------------------------------------------------
    def _transition(self, new_state: State):
        self.get_logger().info(
            f'{self._state.name} → {new_state.name}')
        self._state = new_state
        self._state_entry_time = time.time()

        # Limpa cache ao reiniciar detecção
        if new_state == State.DETECTING:
            self._last_detection = None
            self._last_poses = None
            self._last_grasp = None

    # ------------------------------------------------------------------
    def print_metrics(self):
        if not self._metrics:
            return
        successes = sum(1 for m in self._metrics if m['success'])
        total = len(self._metrics)
        self.get_logger().info(
            f'Métricas: {successes}/{total} sucessos '
            f'({100*successes/total:.1f}%)')


def main(args=None):
    rclpy.init(args=args)
    node = PipelineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_metrics()
    finally:
        node.destroy_node()
        rclpy.shutdown()
