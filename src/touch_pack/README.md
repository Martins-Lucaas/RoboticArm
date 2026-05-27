# touch_pack

Plataforma de **palpação tátil** com o braço CR10 e a ferramenta de contato TouchTool Square 20×20 mm (ou dedo Index COVVI com FT sensor). Reproduz o protocolo de **Gupta et al. 2021** — aproximação até contato por força, hold de força controlado, deslizamento Cartesiano e retração — em modo simulação (SIM_ONLY) ou espelho do robô real (MIRROR).

---

## Protocolo de palpação

```
IDLE → HOME → CONTACT → HOLD → SLIDING → RETRACT → DONE
```

| Fase | Descrição |
|---|---|
| **HOME** | Interpolação linear no espaço de juntas a ≤ 0.3 rad/s até a pose inicial |
| **CONTACT** | Streaming Jacobiano em −Z com perfil de velocidade fast→slow; encerra por threshold de Fz |
| **HOLD** | PID de Fz: mantém a força normal alvo durante `hold_seconds` |
| **SLIDING** | Streaming Jacobiano na direção configurada (±X ou ±Y) a velocidade constante |
| **RETRACT** | Streaming Jacobiano em +Z (oposto ao approach) até `retract_mm` |

Controle por **streaming direto** a 33 Hz — sem action server, sem fila de trajetórias. Cada setpoint é calculado e publicado individualmente, garantindo latência mínima e resposta imediata a `stop`.

---

## Nós

| Executável | Função |
|---|---|
| `tactile_explorer` | FSM da palpação: subscreve `/palpation/start`, executa o ciclo, publica `/palpation/status` |
| `palpation_gui` | GUI Tkinter: controle manual, parâmetros, calibração da célula de carga, poses e movimentos |
| `palpation_logger` | Grava CSV + JSON por run em `~/touch_pack_runs/` |
| `force_receiver` | Recebe pacotes UDP da ESP32 e publica `/load_cell/voltage` e `/load_cell/force` |
| `gazebo_home_setter` | Posiciona o Gazebo na pose atual do robô real ao iniciar (uso único no launch) |

---

## Iniciar

```bash
source install/setup.bash

# Célula completa (Gazebo + CR10 + touch_tool + GUI + logger + force_rx)
ros2 launch touch_pack tactile_cell.launch.py

# Com touch_tool (padrão) e modo espelho do robô real
ros2 launch touch_pack tactile_cell.launch.py \
    end_effector:=touch_tool \
    control_mode:=mirror \
    robot_ip:=192.168.5.2

# Headless (sem GUI Tkinter)
ros2 launch touch_pack tactile_cell.launch.py no_gui:=true
```

### Argumentos do launch

| Argumento | Default | Valores |
|---|---|---|
| `end_effector` | `hand` | `hand` · `touch_tool` |
| `control_mode` | `sim_only` | `sim_only` · `mirror` · `real_from_sim` |
| `robot_ip` | `192.168.5.2` | IP do controlador CR10 |
| `robot_dry_run` | `true` | `true` = sem abrir sockets |
| `no_gui` | `false` | `true` = sem Tkinter |

---

## GUI (`palpation_gui`)

### Aba Palpação

- Sliders: Velocidade (mm/s), Força Normal alvo (N), Distância de Sliding (mm), Distância dedo→alvo (cm), direção XY
- Calibração PID do HOLD (Kp/Ki/Kd)
- Feedback FT em tempo real (Fx/Fy/Fz)
- Barra de progresso e cronômetro por fase
- Botões Start / Stop / E-STOP

### Aba Controle Manual

- 6 sliders do braço CR10 + 6 da mão COVVI
- Presets: Abrir / Apontar / Fechar
- SpeedFactor (%) e duração de trajetória (s)
- Botões Home e salvar Home customizada

### Aba Poses & Movimentos

Permite gravar e executar sequências de poses sem programação:

- **Capturar pose**: do robô real (lê juntas via feedback port) ou do Gazebo (lê `/joint_states`)
- **Drag Teach**: habilita `DragTeachSwitch(1)` no controlador CR10 — o braço fica livre para ser movimentado manualmente; o Gazebo espelha a posição em tempo real a 33 Hz
- **Movimentos**: agrupa N poses + velocidade + duração → executa interpolando entre elas no Gazebo e com `MovJ` cadenciado no robô real (em MIRROR)
- **Persistência**: poses e movimentos salvos em `~/.config/touch_pack/poses.json`

### Header

- Campos de IP da mão e do braço CR10
- Botões Conectar/Desconectar
- Dropdown SIM_ONLY ↔ MIRROR
- ECI ON/OFF · PWR ON/OFF · E-STOP

---

## Cinemática (`kinematics.py`)

FK e Jacobiano para o touch_tool:

```python
T_TOUCH_TOOL_ATTACH  # 4×4, translação +188.5 mm em Z do Link6 (tcp_link)
```

Cadeia URDF do touch_tool:
```
Link6 → lower_coupling (+0mm) → force_sensor (+7mm) → upper_coupling (+59mm)
      → touch_tool (+74mm) → tcp_link (+188.5mm)
```

Conversão braço real ↔ URDF: `_URDF_DOBOT_OFFSET = np.zeros(6)` — convenção de sinal idêntica entre o firmware Dobot e o URDF.

---

## Célula de carga (ESP32)

O nó `force_receiver` abre um socket UDP na porta **8080** e aguarda broadcasts da ESP32.

**Payload** (little-endian, 8 bytes):
```c
float v_sensor;       // tensão do sensor (V)
float force_filtered; // ignorado — recalculado com calibração do PC
```

**Tópicos publicados:**

| Tópico | Tipo | Descrição |
|---|---|---|
| `/load_cell/voltage` | `std_msgs/Float32` | Tensão bruta (V) — sempre publicada |
| `/load_cell/force` | `std_msgs/Float32` | Força calibrada (N) — publicada com slope/intercept vigentes |
| `/load_cell/calibrated` | `std_msgs/Bool` | `true` quando calibração foi carregada do arquivo |

A calibração é lida de `~/.config/touch_pack/load_cell_calib.json` e recarregada a cada 10 s. O arquivo é gerado pelo wizard de calibração da GUI (coleta pares (massa kg, tensão V) e faz regressão linear).

---

## Logging (`palpation_logger`)

Cada run gera dois arquivos em `~/touch_pack_runs/`:

| Arquivo | Conteúdo |
|---|---|
| `<timestamp>__samples.csv` | `t_rel_s, phase, fx, fy, fz, tx, ty, tz` — uma linha por wrench recebido |
| `<timestamp>__params.json` | Parâmetros do `/palpation/start` (força, velocidade, distância, PID, etc.) |

- Run fecha automaticamente em `DONE` ou `ABORTED`
- Watchdog de 5 min: fecha o run se não receber wrench (protege contra crash do explorer)
- Flush a cada 50 amostras (~1 s @ 50 Hz) — não perde dados se o nó morrer

---

## Modo MIRROR — espelho do robô real

Em modo MIRROR, todos os comandos publicados em `/cr10_group_controller/joint_trajectory` são espelhados para o CR10 real:

- **Streaming Jacobiano (palpação)**: cada ponto publicado pelo `tactile_explorer` chega ao braço real com latência ~30 ms
- **Sliders manuais**: cada mudança nos sliders da GUI dispara um `MovJ` com debounce de 80 ms
- **Drag Teach**: poll a 33 Hz lê `read_joints_urdf()` e publica no Gazebo — mirror do movimento manual para o gêmeo digital

Velocidade do braço real controlada por `SpeedFactor(%)` — sincronizado com o slider da GUI.

---

## Disparar palpação via terminal

```bash
ros2 topic pub --once /palpation/start std_msgs/msg/String \
  "data: '{\"speed_mms\": 10.0, \"force_n\": 1.0, \"distance_mm\": 90.0,
           \"target_distance_cm\": 5.0, \"slide_dir\": \"+Y\",
           \"kp\": 0.001, \"ki\": 0.0, \"kd\": 0.0}'"

# Monitorar FSM
ros2 topic echo /palpation/status

# Parar
ros2 topic pub --once /palpation/stop std_msgs/msg/String "data: 'stop'"
```

FSM states: `IDLE` · `HOME` · `CONTACT` · `HOLD` · `SLIDING` · `RETRACT` · `DONE` · `ABORTED`

---

## Arquivos de configuração persistentes

| Arquivo | Conteúdo |
|---|---|
| `~/.config/touch_pack/robot.json` | IPs (mão e braço) + último modo (SIM_ONLY/MIRROR) |
| `~/.config/touch_pack/home_pose.json` | Home customizada do braço (graus por junta) |
| `~/.config/touch_pack/load_cell_calib.json` | Slope e intercept da calibração linear da célula de carga |
| `~/.config/touch_pack/poses.json` | Poses e movimentos gravados na aba Poses & Movimentos |
| `~/touch_pack_runs/` | Dados de palpação (CSV + JSON por run) |

---

## Dependências

```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
<depend>trajectory_msgs</depend>
<exec_depend>admittance_controller</exec_depend>
<exec_depend>kinematics_interface_kdl</exec_depend>
<exec_depend>force_torque_sensor_broadcaster</exec_depend>
```

Instalar:
```bash
sudo apt install ros-humble-admittance-controller \
                 ros-humble-kinematics-interface-kdl \
                 ros-humble-force-torque-sensor-broadcaster
```
