#!/usr/bin/env python3
"""
test_9cycles.py — validação automática de 9 ciclos pick+delivery
(T30 do SDD §9.1).

Para cada objeto (frasco, tubo, ampola) e cada repetição (3 spawns),
executa o ciclo completo via serviços ROS 2:
  1. /conveyor/spawn_<objeto>
  2. /cell/execute_grasp  (acionada pelo grasp_executor)

Registra resultado (success/timeout/error) em CSV. Roda em paralelo ao
launch de conveyor_cell.launch.py (que sobe Gazebo + executor + GUI).

Uso:
    # Terminal 1
    ros2 launch grasp_ml_pack conveyor_cell.launch.py no_gui:=true
    # Terminal 2
    cd /home/lucas-lpc/RoboticArm && source install/setup.bash
    python3 src/grasp_ml_pack/scripts/test_9cycles.py
"""

from __future__ import annotations

import csv
import datetime as _dt
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String


OBJECTS = ('frasco', 'tubo', 'ampola')
REPEATS = 3
TIMEOUT_S = 35.0
OUTPUT_CSV = '/tmp/test_9cycles_results.csv'


class TestRunner(Node):
    def __init__(self):
        super().__init__('test_9cycles')
        self.spawn_cli = {
            obj: self.create_client(Trigger, f'/conveyor/spawn_{obj}')
            for obj in OBJECTS
        }
        self.reset_cli = self.create_client(Trigger, '/conveyor/reset')
        self.exec_cli  = self.create_client(Trigger, '/cell/execute_grasp')
        self.conveyor_state: dict = {}
        self.create_subscription(String, '/conveyor/status',
                                  self._cb_conv, 10)

    def _cb_conv(self, msg: String):
        try:
            self.conveyor_state = json.loads(msg.data)
        except Exception:
            pass

    def _call(self, cli, label: str, timeout: float = 8.0):
        if not cli.wait_for_service(timeout_sec=4.0):
            return None, f'{label} indisponível'
        fut = cli.call_async(Trigger.Request())
        end = time.time() + timeout
        while rclpy.ok() and not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not fut.done():
            return None, f'{label} timeout'
        res = fut.result()
        return res.success, res.message

    def run_one_cycle(self, obj: str) -> tuple[str, str]:
        ok, msg = self._call(self.reset_cli, 'reset', timeout=6.0)
        time.sleep(0.5)
        ok, msg = self._call(self.spawn_cli[obj], f'spawn_{obj}',
                              timeout=8.0)
        if not ok:
            return 'spawn_fail', msg or ''
        time.sleep(1.0)
        # Espera o conveyor reportar has_object
        deadline = time.time() + 10.0
        while time.time() < deadline:
            if self.conveyor_state.get('has_object'):
                break
            rclpy.spin_once(self, timeout_sec=0.2)
        else:
            return 'no_object_after_spawn', ''

        ok, msg = self._call(self.exec_cli, 'execute_grasp',
                              timeout=TIMEOUT_S)
        if ok is None:
            return 'timeout', msg or ''
        return ('success' if ok else 'exec_fail'), msg or ''


def main():
    rclpy.init()
    node = TestRunner()
    rows = []
    print(f'test_9cycles.py — {len(OBJECTS)} objetos × {REPEATS} repetições')
    print('=' * 60)
    t_start = _dt.datetime.now()
    for obj in OBJECTS:
        for rep in range(1, REPEATS + 1):
            print(f'\n[{obj} #{rep}] iniciando…')
            t0 = time.time()
            outcome, detail = node.run_one_cycle(obj)
            dur = time.time() - t0
            print(f'  → {outcome} ({dur:.1f}s) — {detail}')
            rows.append({'obj': obj, 'rep': rep, 'outcome': outcome,
                         'duration_s': f'{dur:.1f}', 'detail': detail})

    # Resumo
    print('\n' + '=' * 60)
    n_ok = sum(1 for r in rows if r['outcome'] == 'success')
    print(f'Resultados: {n_ok}/{len(rows)} sucesso')
    print(f'Início:  {t_start.isoformat()}')
    print(f'Fim:     {_dt.datetime.now().isoformat()}')

    # Grava CSV
    with open(OUTPUT_CSV, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['obj','rep','outcome',
                                            'duration_s','detail'])
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f'CSV gravado em {OUTPUT_CSV}')

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if n_ok == len(rows) else 1)


if __name__ == '__main__':
    main()
