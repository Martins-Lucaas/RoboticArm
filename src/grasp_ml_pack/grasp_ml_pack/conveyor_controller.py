"""
Nó de controle da esteira — Célula de Manufatura Biomédica.

Objetos gerenciados:
  frasco — frasco de medicamento (cilindro âmbar, r=42mm, h=90mm)
  tubo   — tubo de ensaio        (cilindro azul,  r=12mm, h=120mm)
  ampola — ampola farmacêutica   (cilindro verde, r=5mm,  h=75mm)

Serviços expostos:
  /conveyor/advance  (Trigger) — traz o próximo objeto para a pick station
  /conveyor/retreat  (Trigger) — remove o objeto atual
  /conveyor/reset    (Trigger) — reinicia a sequência

Publica:
  /conveyor/status   (String JSON) — estado atual da esteira
"""

from __future__ import annotations

import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from gazebo_msgs.srv import SpawnEntity, DeleteEntity
    _GAZEBO_MSGS_OK = True
except ImportError:
    _GAZEBO_MSGS_OK = False


# ── SDF templates ─────────────────────────────────────────────────────────────
# Posição definida via initial_pose no SpawnEntity.Request — sem <pose> no SDF.

_SDF_FRASCO = """\
<sdf version="1.6">
<model name="pick_object">
  <static>false</static>
  <link name="link">
    <inertial>
      <mass>0.180</mass>
      <inertia><ixx>5e-4</ixx><iyy>5e-4</iyy><izz>2e-4</izz>
               <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
    </inertial>
    <collision name="collision">
      <geometry><cylinder><radius>0.042</radius><length>0.090</length></cylinder></geometry>
      <surface>
        <friction><ode><mu>1.5</mu><mu2>1.5</mu2><fdir1>0 0 0</fdir1><slip1>0.0</slip1><slip2>0.0</slip2></ode></friction>
        <contact><ode><max_vel>0.02</max_vel><min_depth>0.001</min_depth></ode></contact>
      </surface>
    </collision>
    <visual name="visual">
      <geometry><cylinder><radius>0.042</radius><length>0.090</length></cylinder></geometry>
      <material>
        <ambient>0.80 0.45 0.00 1</ambient>
        <diffuse>0.90 0.55 0.05 1</diffuse>
        <specular>0.60 0.35 0.10 1</specular>
      </material>
    </visual>
  </link>
</model>
</sdf>"""

_SDF_TUBO = """\
<sdf version="1.6">
<model name="pick_object">
  <static>false</static>
  <link name="link">
    <inertial>
      <mass>0.025</mass>
      <inertia><ixx>3e-5</ixx><iyy>3e-5</iyy><izz>1e-6</izz>
               <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
    </inertial>
    <collision name="collision">
      <geometry><cylinder><radius>0.012</radius><length>0.120</length></cylinder></geometry>
      <surface>
        <friction><ode><mu>1.5</mu><mu2>1.5</mu2><fdir1>0 0 0</fdir1><slip1>0.0</slip1><slip2>0.0</slip2></ode></friction>
        <contact><ode><max_vel>0.02</max_vel><min_depth>0.001</min_depth></ode></contact>
      </surface>
    </collision>
    <visual name="visual">
      <geometry><cylinder><radius>0.012</radius><length>0.120</length></cylinder></geometry>
      <material>
        <ambient>0.05 0.25 0.80 1</ambient>
        <diffuse>0.10 0.35 0.95 1</diffuse>
        <specular>0.40 0.50 0.90 1</specular>
      </material>
    </visual>
  </link>
</model>
</sdf>"""

_SDF_AMPOLA = """\
<sdf version="1.6">
<model name="pick_object">
  <static>false</static>
  <link name="link">
    <inertial>
      <mass>0.008</mass>
      <inertia><ixx>5e-6</ixx><iyy>5e-6</iyy><izz>1e-8</izz>
               <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
    </inertial>
    <collision name="collision">
      <geometry><cylinder><radius>0.005</radius><length>0.075</length></cylinder></geometry>
      <surface>
        <friction><ode><mu>1.5</mu><mu2>1.5</mu2><fdir1>0 0 0</fdir1><slip1>0.0</slip1><slip2>0.0</slip2></ode></friction>
        <contact><ode><max_vel>0.02</max_vel><min_depth>0.001</min_depth></ode></contact>
      </surface>
    </collision>
    <visual name="visual">
      <geometry><cylinder><radius>0.005</radius><length>0.075</length></cylinder></geometry>
      <material>
        <ambient>0.05 0.80 0.15 1</ambient>
        <diffuse>0.10 0.95 0.20 1</diffuse>
        <specular>0.30 0.80 0.40 1</specular>
      </material>
    </visual>
  </link>
</model>
</sdf>"""

_SPAWN_SDF: dict[str, str] = {
    'frasco': _SDF_FRASCO,
    'tubo':   _SDF_TUBO,
    'ampola': _SDF_AMPOLA,
}

# Metade da altura de cada objeto (para calcular z_center acima da esteira)
_OBJ_HALF_HEIGHT: dict[str, float] = {
    'frasco': 0.045,   # height=0.090 / 2
    'tubo':   0.060,   # height=0.120 / 2
    'ampola': 0.0375,  # height=0.075 / 2
}


class ConveyorControllerNode(Node):

    def __init__(self):
        super().__init__('conveyor_controller')

        self.declare_parameter('pick_x', 0.65)
        self.declare_parameter('pick_y', 0.00)
        self.declare_parameter('belt_surface_z', 0.806)
        self.declare_parameter('spawn_z', 2.0)
        self.declare_parameter('object_sequence', ['frasco', 'tubo', 'ampola'])
        self.declare_parameter('sim_only', True)

        self._pick_x  = self.get_parameter('pick_x').value
        self._pick_y  = self.get_parameter('pick_y').value
        self._belt_z  = self.get_parameter('belt_surface_z').value
        self._spawn_z: float = self.get_parameter('spawn_z').value
        self._sequence: list[str] = list(
            self.get_parameter('object_sequence').value)
        self._sim_only: bool = self.get_parameter('sim_only').value

        self._current_idx: int  = -1
        self._has_object:  bool = False
        self._busy:        bool = False
        self._lock = threading.Lock()

        cb = ReentrantCallbackGroup()

        self.create_service(Trigger, '/conveyor/advance',
                            self._cb_advance, callback_group=cb)
        self.create_service(Trigger, '/conveyor/retreat',
                            self._cb_retreat, callback_group=cb)
        self.create_service(Trigger, '/conveyor/reset',
                            self._cb_reset, callback_group=cb)

        # Spawns específicos por classe — permitem ao operador escolher
        # diretamente qual objeto colocar na pick station, em vez de avançar
        # ciclicamente. Usados pelo `manual_control` (botões coloridos por
        # objeto). Se já houver um objeto presente, ele é removido antes.
        for obj_class in ('frasco', 'tubo', 'ampola'):
            self.create_service(
                Trigger, f'/conveyor/spawn_{obj_class}',
                lambda req, resp, oc=obj_class: self._cb_spawn_specific(oc, req, resp),
                callback_group=cb)

        self._pub_status = self.create_publisher(String, '/conveyor/status', 10)
        self.create_timer(1.0, self._pub_status_tick)

        if self._sim_only and _GAZEBO_MSGS_OK:
            self._spawn_cli  = self.create_client(
                SpawnEntity,  '/spawn_entity',  callback_group=cb)
            self._delete_cli = self.create_client(
                DeleteEntity, '/delete_entity', callback_group=cb)
        else:
            self._spawn_cli  = None
            self._delete_cli = None

        self.get_logger().info(
            f'ConveyorController pronto | sequência: {self._sequence}')

    # ──────────────────────────────────────────────────────────────────
    def _pub_status_tick(self):
        obj = self._sequence[self._current_idx] if self._current_idx >= 0 else 'none'
        self._pub_status.publish(String(data=json.dumps({
            'has_object': self._has_object,
            'current_obj': obj,
            'queue_idx': self._current_idx,
            'queue_total': len(self._sequence),
        })))

    # ──────────────────────────────────────────────────────────────────
    def _cb_advance(self, _req, resp: Trigger.Response):
        with self._lock:
            if self._busy:
                resp.success = False
                resp.message = 'Esteira ocupada.'
                return resp
            if self._has_object:
                resp.success = False
                resp.message = 'Já há objeto na pick station. Use retreat primeiro.'
                return resp
            next_idx = (self._current_idx + 1) % len(self._sequence)
            obj = self._sequence[next_idx]
            self._busy = True

        ok = self._spawn_object(obj)

        with self._lock:
            if ok:
                self._current_idx = next_idx
                self._has_object  = True
                msg = f'Pick station: {obj}'
            else:
                msg = f'Falha ao spawnar {obj}.'
            self._busy = False

        resp.success = ok
        resp.message = msg
        self.get_logger().info(f'[CONVEYOR] advance → {msg}')
        return resp

    # ──────────────────────────────────────────────────────────────────
    def _cb_retreat(self, _req, resp: Trigger.Response):
        with self._lock:
            if self._busy:
                resp.success = False
                resp.message = 'Esteira ocupada.'
                return resp
            if not self._has_object:
                resp.success = False
                resp.message = 'Nenhum objeto na pick station.'
                return resp
            self._busy = True

        ok = self._delete_object()

        with self._lock:
            if ok:
                self._has_object  = False
                self._current_idx = max(-1, self._current_idx - 1)
                msg = 'Pick station liberada.'
            else:
                msg = 'Falha ao remover objeto.'
            self._busy = False

        resp.success = ok
        resp.message = msg
        self.get_logger().info(f'[CONVEYOR] retreat → {msg}')
        return resp

    # ──────────────────────────────────────────────────────────────────
    def _cb_spawn_specific(self, obj_class: str,
                            _req, resp: Trigger.Response):
        """
        Spawna um objeto específico (frasco/tubo/ampola). Se já houver outro
        objeto na pick station, ele é removido primeiro. Mantém `current_idx`
        sincronizado com `_sequence` para o status JSON.
        """
        if obj_class not in _SPAWN_SDF:
            resp.success = False
            resp.message = f'Classe desconhecida: {obj_class!r}'
            return resp

        with self._lock:
            if self._busy:
                resp.success = False
                resp.message = 'Esteira ocupada.'
                return resp
            self._busy = True

        # Remove qualquer objeto presente antes de spawnar o novo
        if self._has_object:
            self._delete_object()
            with self._lock:
                self._has_object = False

        ok = self._spawn_object(obj_class)

        with self._lock:
            if ok:
                # Sincroniza `current_idx` para o status refletir o spawn
                try:
                    self._current_idx = self._sequence.index(obj_class)
                except ValueError:
                    self._current_idx = -1
                self._has_object = True
                msg = f'Pick station: {obj_class}'
            else:
                msg = f'Falha ao spawnar {obj_class}.'
            self._busy = False

        resp.success = ok
        resp.message = msg
        self.get_logger().info(f'[CONVEYOR] spawn_{obj_class} → {msg}')
        return resp

    # ──────────────────────────────────────────────────────────────────
    def _cb_reset(self, _req, resp: Trigger.Response):
        with self._lock:
            if self._busy:
                resp.success = False
                resp.message = 'Esteira ocupada.'
                return resp
            self._busy = True

        if self._has_object:
            self._delete_object()

        with self._lock:
            self._current_idx = -1
            self._has_object  = False
            self._busy        = False

        resp.success = True
        resp.message = 'Esteira resetada. Sequência reiniciada.'
        self.get_logger().info('[CONVEYOR] reset completo.')
        return resp

    # ──────────────────────────────────────────────────────────────────
    def _spawn_object(self, obj_class: str) -> bool:
        if not self._sim_only or self._spawn_cli is None:
            return True
        if not self._spawn_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/spawn_entity indisponível.')
            return False

        # Posição definida exclusivamente via initial_pose do request.
        # O SDF não carrega pose — evita conflito/dupla-aplicação pelo Gazebo.
        req = SpawnEntity.Request()
        req.name = 'pick_object'
        req.xml  = _SPAWN_SDF[obj_class]
        req.initial_pose.position.x  = self._pick_x
        req.initial_pose.position.y  = self._pick_y
        req.initial_pose.position.z  = self._spawn_z
        req.initial_pose.orientation.w = 1.0

        future = self._spawn_cli.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 5.0:
            time.sleep(0.05)
        if future.done() and future.result() is not None:
            return future.result().success
        self.get_logger().error('Timeout ao spawnar.')
        return False

    # ──────────────────────────────────────────────────────────────────
    def _delete_object(self) -> bool:
        if not self._sim_only or self._delete_cli is None:
            return True
        if not self._delete_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/delete_entity indisponível.')
            return False

        req = DeleteEntity.Request()
        req.name = 'pick_object'

        future = self._delete_cli.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 5.0:
            time.sleep(0.05)
        if future.done() and future.result() is not None:
            return future.result().success
        self.get_logger().error('Timeout ao deletar.')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorControllerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
