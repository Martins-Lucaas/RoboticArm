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
    State.DETECTING:  60.0,   # 1 min para encontrar o objeto
    State.ESTIMATING: 20.0,
    State.PLANNING:   20.0,
    State.EXECUTING:  240.0,  # pick+rotate+place com robot lento ~120-180 s
}

# Ordem de prioridade de preensão
_GRASP_ORDER = ['ball', 'cup', 'pencil']


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

        self._completed: set[str] = set()
        self._current_target: str = _GRASP_ORDER[0]

        # Subscriptions (passivas — apenas cache)
        self.create_subscription(
            Detection2DArray, '/detected_objects', self._on_detection, 10)
        self.create_subscription(
            PoseArray, '/object_poses', self._on_poses, 10)
        self.create_subscription(
            String, '/selected_grasp', self._on_grasp, 10)
        self.create_subscription(
            String, '/grasp_result', self._on_result, 10)

        # Publishers
        self._pub_status = self.create_publisher(String, '/pipeline/status', 10)
        self._pub_target = self.create_publisher(String, '/pipeline/target', 10)

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
            label = result['object_label']
            self._completed.add(label)
            self.get_logger().info(
                f'[SUCESSO] {label} ({result["grasp_type"]}) '
                f'score={result["score"]:.2f} | '
                f'concluídos: {sorted(self._completed)}')
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
                # Próximo objeto na ordem de prioridade não concluído
                for obj in _GRASP_ORDER:
                    if obj not in self._completed:
                        self._current_target = obj
                        break
                else:
                    self.get_logger().info(
                        'Sequência completa: bola → copo → lápis. Pipeline encerrado.')
                    rclpy.shutdown()
                    return
                self._attempt = 0
                self.get_logger().info(f'Próximo alvo: {self._current_target}')
                self._transition(State.DETECTING)

            case State.DETECTING:
                if (self._last_detection is not None
                        and self._target_detected()):
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
    def _target_detected(self) -> bool:
        labels = {d.results[0].hypothesis.class_id
                  for d in self._last_detection.detections}
        return self._current_target in labels

    # ------------------------------------------------------------------
    def _transition(self, new_state: State):
        self.get_logger().info(
            f'{self._state.name} → {new_state.name}')
        self._state = new_state
        self._state_entry_time = time.time()

        # Publica alvo atual para planner e pose estimator
        self._pub_target.publish(String(data=self._current_target))

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
