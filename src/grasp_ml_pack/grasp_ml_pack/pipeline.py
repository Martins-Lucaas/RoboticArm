"""
Orquestrador da célula de manufatura (modo autônomo opcional).

No modo GUI (padrão), este nó apenas agrega e republica o estado do sistema.
No modo autônomo (--autonomous), executa o ciclo completo sem intervenção:
  Advance → Detect → Grasp → Repeat

Publica:
  /pipeline/status  (std_msgs/String JSON) — estado consolidado

Subscreve:
  /conveyor/status  (String JSON)
  /cell/status      (String JSON)
  /detected_objects (Detection2DArray)
"""

from __future__ import annotations

import json
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray


class AutoState(Enum):
    IDLE     = auto()
    ADVANCE  = auto()
    WAIT_OBJ = auto()
    GRASP    = auto()
    WAIT_END = auto()
    DONE     = auto()


class PipelineNode(Node):

    def __init__(self):
        super().__init__('conveyor_pipeline')

        self.declare_parameter('autonomous', False)
        self.declare_parameter('loop_rate_hz', 5.0)
        self.declare_parameter('detect_timeout_s', 30.0)
        self.declare_parameter('grasp_timeout_s', 300.0)
        self.declare_parameter('total_cycles', 3)

        self._auto     = self.get_parameter('autonomous').value
        rate           = self.get_parameter('loop_rate_hz').value
        self._det_tmo  = self.get_parameter('detect_timeout_s').value
        self._gsp_tmo  = self.get_parameter('grasp_timeout_s').value
        self._max_cyc  = self.get_parameter('total_cycles').value

        self._conveyor: dict = {}
        self._cell:     dict = {}
        self._last_det: str | None = None
        self._cycle:    int  = 0

        self._auto_state = AutoState.IDLE
        self._state_t    = time.time()

        self.create_subscription(
            String, '/conveyor/status', self._cb_conv, 10)
        self.create_subscription(
            String, '/cell/status', self._cb_cell, 10)
        self.create_subscription(
            Detection2DArray, '/detected_objects', self._cb_det, 10)

        self._pub = self.create_publisher(String, '/pipeline/status', 10)
        self.create_timer(1.0 / rate, self._step)

        if self._auto:
            self._adv_cli = self.create_client(Trigger, '/conveyor/advance')
            self._gsp_cli = self.create_client(Trigger, '/cell/execute_grasp')

        mode = 'autônomo' if self._auto else 'GUI'
        self.get_logger().info(f'Pipeline iniciado — modo: {mode}')

    # ------------------------------------------------------------------
    def _cb_conv(self, msg: String):
        try:
            self._conveyor = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _cb_cell(self, msg: String):
        try:
            self._cell = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _cb_det(self, msg: Detection2DArray):
        if msg.detections:
            self._last_det = msg.detections[0].results[0].hypothesis.class_id
        else:
            self._last_det = None

    # ------------------------------------------------------------------
    def _step(self):
        status = {
            'conveyor': self._conveyor,
            'cell': self._cell,
            'last_detected': self._last_det,
            'cycle': self._cycle,
        }
        if self._auto:
            status['auto_state'] = self._auto_state.name
            self._step_auto()
        self._pub.publish(String(data=json.dumps(status)))

    # ------------------------------------------------------------------
    def _step_auto(self):
        """FSM autônoma para processar a fila de objetos sem GUI."""
        elapsed = time.time() - self._state_t

        match self._auto_state:
            case AutoState.IDLE:
                if self._cycle >= self._max_cyc:
                    self._auto_state = AutoState.DONE
                    self.get_logger().info('Pipeline autônomo: todos os ciclos concluídos.')
                    return
                self._do_advance()
                self._auto_state = AutoState.WAIT_OBJ
                self._state_t = time.time()

            case AutoState.WAIT_OBJ:
                if self._conveyor.get('has_object') and self._last_det:
                    self._auto_state = AutoState.GRASP
                    self._state_t = time.time()
                elif elapsed > self._det_tmo:
                    self.get_logger().warn('Timeout aguardando objeto. Tentando novamente.')
                    self._auto_state = AutoState.IDLE

            case AutoState.GRASP:
                if not self._cell.get('busy'):
                    self._do_grasp()
                    self._auto_state = AutoState.WAIT_END
                    self._state_t = time.time()

            case AutoState.WAIT_END:
                if not self._cell.get('busy'):
                    if self._cell.get('state') == 'CYCLE_DONE':
                        self._cycle += 1
                        self.get_logger().info(f'Ciclo {self._cycle}/{self._max_cyc} concluído.')
                        self._auto_state = AutoState.IDLE
                elif elapsed > self._gsp_tmo:
                    self.get_logger().error('Timeout no grasp. Abortando ciclo.')
                    self._cycle += 1
                    self._auto_state = AutoState.IDLE

            case AutoState.DONE:
                pass   # pipeline encerrado

    # ------------------------------------------------------------------
    def _do_advance(self):
        if not self._adv_cli.service_is_ready():
            return
        self._adv_cli.call_async(Trigger.Request())

    def _do_grasp(self):
        if not self._gsp_cli.service_is_ready():
            return
        self._gsp_cli.call_async(Trigger.Request())


def main(args=None):
    rclpy.init(args=args)
    node = PipelineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
