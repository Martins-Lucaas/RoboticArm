# touch_pack

Plataforma de **palpação tátil** com o braço CR10 e a ferramenta de contato TouchTool Square 20×20 mm (acoplada à célula de carga) — ou o dedo Index da mão COVVI. Reproduz o protocolo de **Gupta et al. 2021** (aproximação até a força alvo, hold de força controlado, deslizamento Cartesiano e retração) em **simulação (SIM_ONLY)** ou em **espelho do robô real (MIRROR)**.

Inclui ainda um **sensor de toque neuromórfico (STM32, matriz 4×4 + modelo de Izhikevich)**, gravação **sincronizada de força + toque**, e dois **modos de operação**: **Toque** (encosta com força controlada e volta) e **Deslizamento** (ciclo completo com arrasto lateral).

<p align="center">
  <img src="../../images/touch_pack_gazebo_tactile_cell.png" width="48%" alt="Célula de palpação tátil no Gazebo — CR10 com ponta TouchTool sobre a mesa de palpação"/>
  <img src="../../images/touch_pack_gui_palpation_slide.png" width="48%" alt="GUI de Palpação — aba Palpação no modo Deslizamento com painel da célula de carga ao vivo"/>
</p>
<p align="center"><em>Esquerda: cena de palpação no Gazebo Classic. Direita: GUI <code>palpation_gui</code> — aba <strong>Palpação</strong> (modo Deslizamento) com leitura ao vivo da célula de carga e do sensor de toque.</em></p>

---

## Modos de palpação

A GUI divide a palpação em dois modos (seletor na aba **Palpação**):

| Modo | Ciclo executado | Uso |
|---|---|---|
| **Toque** | `HOME → DESCENDING → HOLD → RETRACT` (repetido **N toques**) | Encostar na mesa com força controlada e voltar à home — sem deslizar. Quantidade de toques selecionável. |
| **Deslizamento** | `HOME → DESCENDING → HOLD → SLIDING → RETRACT` | Ciclo completo do protocolo Gupta, com arrasto lateral em ±X/±Y. |

Ambos os modos repetem o ciclo `repeats` vezes automaticamente (entre repetições o braço recua e refaz a home).

---

## Protocolo / FSM

```
IDLE → HOME → DESCENDING → HOLD → [SLIDING] → RETRACT → HOME → IDLE
```

| Fase | Descrição |
|---|---|
| **HOME** | Trajetória batch no espaço de juntas (S-curve do JTC) a ≤ 0.3 rad/s até a pose inicial. Verifica se o TCP aponta para baixo. |
| **DESCENDING** | Streaming Jacobiano ao longo do approach (−Z do TCP) com perfil fast→slow. **Controle por força:** termina quando a compressão atinge o *setpoint* (`force_n`). `depth_mm` é o curso **máximo de segurança**. |
| **HOLD** | PID de força leva a compressão ao setpoint e **espera estabilizar**: `|Fz − alvo| ≤ tol` por `hold_stable_s` contínuos (teto `hold_timeout_s`). |
| **SLIDING** | (só no modo Deslizamento) Streaming Jacobiano lateral em ±X/±Y a velocidade constante, com PID de força simultâneo no eixo normal e lock de orientação/profundidade. |
| **RETRACT** | Recuo Cartesiano em +Z (oposto ao approach) por `retract_mm`. |

Controle por **streaming direto** a 33 Hz — sem action server, sem fila de trajetórias. Cada setpoint é calculado e publicado individualmente, garantindo latência mínima e resposta imediata a `stop`/`pause`.

**Segurança de força:** convenção compressão = **positivo**, tração = negativo. A medição é **abortada** se a compressão exceder **15 N** (`FORCE_ABORT_LIMIT_N`); o setpoint do PID é saturado em **10 N**. Se a leitura da célula ficar velha (> 0.5 s), as fases de força abortam (`stale`).

---

## Nós

| Executável | Função |
|---|---|
| `tactile_explorer` | FSM da palpação: assina `/palpation/start`, executa o ciclo (modo Toque/Deslizamento), publica `/palpation/status` |
| `palpation_gui` | GUI Tkinter: parâmetros, modos, controle manual, calibração da célula, poses/movimentos, dashboard do sensor de toque |
| `palpation_logger` | Grava CSV + JSON por run em `sensors/Data/` |
| `palpation_report` | Gera relatório estatístico por ciclo a partir dos CSVs de run |
| `force_receiver` | Recebe UDP da ESP32 (porta **8080**) → `/load_cell/voltage`, `/load_cell/force`, `/load_cell/calibrated` |
| `touch_receiver` | Recebe UDP do plotter do touch sensor (porta **8081**) → `/touch_sensor/value` |
| `force_sync` | Pareia força × toque por chegada → `/touch_sync/data` (`SyncedTouch`) a 50 Hz |
| `mirror_node` | Espelhamento sim → CR10 real **sem a GUI** (cobre `no_gui:=true` em MIRROR) |
| `real_pose_sync` | Move o braço simulado até a pose do robô real ao iniciar (uso único no launch) |

---

## Iniciar

```bash
source install/setup.bash

# Célula completa (Gazebo + CR10 + GUI + logger + force_rx + touch_rx)
ros2 launch touch_pack tactile_cell.launch.py

# Ponta tátil + célula de carga (libera a aba Palpação) e espelho do robô real
ros2 launch touch_pack tactile_cell.launch.py \
    end_effector:=touch_tool \
    control_mode:=mirror \
    robot_ip:=192.168.5.2

# Headless (sem GUI Tkinter) — mirror_node assume o espelhamento
ros2 launch touch_pack tactile_cell.launch.py control_mode:=mirror no_gui:=true
```

### Argumentos do launch

| Argumento | Default | Valores |
|---|---|---|
| `end_effector` | `hand` | `hand` (controle da mão COVVI) · `touch_tool` (ponta de palpação + célula de carga) |
| `control_mode` | `sim_only` | `sim_only` · `mirror` · `real_from_sim` |
| `robot_ip` | `192.168.5.2` | IP do controlador CR10 real |
| `no_gui` | `false` | `true` = sem Tkinter (usa `mirror_node` em MIRROR) |

> A aba/modo **Palpação** só fica ativa com `end_effector:=touch_tool` (precisa da célula de carga). Em `hand` a GUI mostra o controle da mão.

---

## GUI (`palpation_gui`)

Notebook com 5 abas: **Palpação · Controle Manual · Célula de Carga · Poses & Movimentos · Sensores**.

### Aba Palpação
- **Seletor de modo**: Toque / Deslizamento (mostra/oculta os parâmetros de deslizamento).
- **Força Alvo** (setpoint do PID, 1–10 N, inteiro) · **Repetições / Quantidade de Toques**.
- **Deslizamento** (só nesse modo): Velocidade (mm/s), Distância (mm), Direção ±X/±Y.
- **Avançados** (recolhível): Profundidade máx. de descida, PID Kp/Ki/Kd, Velocidade de aproximação, e estabilização do HOLD (tolerância da banda, janela estável, timeout).
- Leitura da **célula de carga** ao vivo + sparkline da força.
- Botões **Iniciar / Parar / ⏸ Pausar** e **Salvar dados (força+toque)**.
- Parâmetros persistidos entre sessões (incl. o modo escolhido).

### Aba Controle Manual
- 6 sliders do braço CR10 + (em modo `hand`) 6 da mão COVVI.
- Presets Abrir / Apontar / Fechar · SpeedFactor (%) · duração de trajetória.
- Botões Home e salvar Home customizada · mini-painel da célula de carga.

<p align="center">
  <img src="../../images/touch_pack_gui_manual_gazebo.png" width="92%" alt="Aba Controle Manual lado a lado com o Gazebo — sliders das juntas do CR10 e leitura da célula de carga"/>
</p>
<p align="center"><em>Aba <strong>Controle Manual</strong> ao lado do Gazebo: jog das 6 juntas do CR10 com a célula de carga lida ao vivo (jog em <code>MovJ</code> quando em MIRROR).</em></p>

### Aba Célula de Carga
- **Leitura**: força/tensão ao vivo + tara.
- **Calibração**: wizard que coleta pares (massa kg, tensão V) e faz regressão linear (slope/intercept).

### Aba Poses & Movimentos
- **Capturar pose** do robô real (feedback port) ou do Gazebo (`/joint_states`).
- **Drag Teach**: libera o braço real (`DragTeachSwitch`); o Gazebo espelha o movimento manual a 33 Hz; detecção automática de drag por movimento de juntas.
- **Movimentos**: sequências de N poses + velocidade → interpola no Gazebo e `MovJ` cadenciado no real (MIRROR).
- Persistência em `~/.config/touch_pack/poses.json`.

### Aba Sensores
Dashboard do **sensor de toque (STM32, Izhikevich)** com 4 gráficos embutidos via matplotlib:
**Heatmap de tensão (4×4)** · **Raster RA/SA** (janela deslizante de 5 s) · **I_final** · **Neurônio pós**, ao lado da leitura ao vivo da célula de carga.
Renderização com **blit** (`FuncAnimation` @ 20 Hz) — desenha só os artistas que mudam, e pausa quando a aba não está visível (sem travamento, raster desliza suave).

<p align="center">
  <img src="../../images/touch_pack_gui_sensors_izhikevich.png" width="92%" alt="Aba Sensores — heatmap 4×4, raster RA/SA, I_final e neurônio pós do modelo de Izhikevich"/>
</p>
<p align="center"><em>Aba <strong>Sensores</strong>: heatmap de tensão 4×4, raster RA/SA, corrente <code>I_final</code> e resposta do neurônio pós-sináptico (Izhikevich), lidos do STM32 por USB-CDC.</em></p>

### Header
- IP da mão e do braço CR10 + Conectar/Desconectar · dropdown SIM_ONLY ↔ MIRROR · ECI ON/OFF · PWR ON/OFF · E-STOP.

---

## Sensor de toque (STM32)

Matriz de **4×4 taxels** lida por USB-CDC (115200 baud, auto-detecção da porta ACM/USB). O firmware emite tensões, *spikes* RA/SA (modelo neuromórfico) e a corrente final `I_final` do neurônio pós-sináptico (modelo de **Izhikevich**).

- `touch_source.py` (`TouchSensorSource`) lê a serial direto no PC da GUI e alimenta a aba **Sensores**; publica `/touch_sensor/value` (throttle de 100 Hz).
- Sem serial local, `touch_receiver` recebe a leitura retransmitida por UDP (porta **8081**) e publica o mesmo `/touch_sensor/value` — a GUI cai automaticamente para esse modo.

---

## Sincronização força × toque

`force_sync` pareia a última amostra fresca de `/load_cell/force` com a de `/touch_sensor/value` e publica `touch_pack_msgs/SyncedTouch` em `/touch_sync/data` a **50 Hz** (mesma taxa da célula). Cada par carrega `load_cell_age_ms` / `touch_age_ms` para avaliar a qualidade da sincronização *a posteriori*.

---

## Onde os dados são salvos

Tudo vai para **`<raiz_do_repo>/sensors/Data/`** (override: variável de ambiente `TOUCH_PACK_DATA_DIR`). O diretório é localizado automaticamente subindo a partir do pacote até achar `sensors/` — funciona tanto rodando do `src/` quanto do `install/`.

| Arquivo | Origem | Conteúdo |
|---|---|---|
| `<ts>__samples.csv` | `palpation_logger` | `t_rel_s, cycle, phase, force_net_n, q1..q6, tcp_x/y/z, touch_value, touch_age_ms` — uma linha por amostra |
| `<ts>__params.json` | `palpation_logger` | parâmetros do `/palpation/start` (lido pelo `palpation_report`) |
| `<ts>__sensors.csv` | botão **Salvar dados (força+toque)** da GUI | snapshot a 50 Hz: força líquida, LC bruto, tensão LC, `touch_i_final` e as **16 tensões** dos taxels (`v00..v33`) |

- O run fecha automaticamente em `DONE`/`ABORTED`; watchdog fecha se parar de receber amostras.
- Flush periódico — não perde dados se o nó morrer.

---

## Interfaces ROS (`touch_pack_msgs`)

### `/palpation/start` — `touch_pack_msgs/PalpationStart`
Mensagem **tipada** (substitui o antigo JSON em `std_msgs/String`):

| Campo | Significado |
|---|---|
| `mode` | `'TOUCH'` (toque) · `'SLIDE'` (deslizamento) · vazio = SLIDE |
| `force_n` | setpoint do PID de força (N, compressão) |
| `depth_mm` | curso máximo de descida — segurança |
| `speed_mms` · `slide_dist_mm` · `slide_dir` | parâmetros do deslizamento (`+X`/`-X`/`+Y`/`-Y`) |
| `kp` · `ki` · `kd` | ganhos do PID de força ((m/s)/N) |
| `approach_speed_mms` | velocidade da descida/recuo |
| `repeats` | nº de ciclos / toques (≥ 1) |
| `speed_factor_pct` | SpeedFactor do braço real (%) |
| `home_deg[6]` | home do braço (graus, joint1..joint6) |
| `hold_tol_n` · `hold_stable_s` · `hold_timeout_s` | estabilização do HOLD (0 = default) |

### `/palpation/status` — `touch_pack_msgs/PalpationStatus`
`phase`, `cycle`, `cycles_total`, `target_depth_mm`, `target_force_n`, `force_net_n`, `speed_mms`, `paused`.

### Outros tópicos
| Tópico | Tipo | Descrição |
|---|---|---|
| `/palpation/stop` | `std_msgs/String` | para o experimento |
| `/palpation/pause` | `std_msgs/Bool` | pausa (segura a posição) / retoma |
| `/load_cell/voltage` | `std_msgs/Float32` | tensão bruta da célula (V) |
| `/load_cell/force` | `std_msgs/Float32` | força calibrada (N, compressão +) |
| `/load_cell/force_net` | `std_msgs/Float32` | força **tare-compensada** (publicada pela GUI; consumida pelo explorer/logger) |
| `/load_cell/calibrated` | `std_msgs/Bool` | calibração carregada |
| `/touch_sensor/value` | `std_msgs/Float32` | leitura do touch sensor |
| `/touch_sync/data` | `touch_pack_msgs/SyncedTouch` | par força × toque sincronizado (50 Hz) |

---

## Disparar palpação via terminal

```bash
# Modo Toque — 3 toques a 2 N
ros2 topic pub --once /palpation/start touch_pack_msgs/msg/PalpationStart \
  "{mode: 'TOUCH', force_n: 2.0, depth_mm: 30.0, repeats: 3,
    approach_speed_mms: 50.0, speed_factor_pct: 10.0,
    kp: 0.001, ki: 0.0005, kd: 0.0}"

# Modo Deslizamento — 50 mm em +Y a 10 mm/s
ros2 topic pub --once /palpation/start touch_pack_msgs/msg/PalpationStart \
  "{mode: 'SLIDE', force_n: 2.0, depth_mm: 30.0, speed_mms: 10.0,
    slide_dist_mm: 50.0, slide_dir: '+Y', repeats: 1,
    approach_speed_mms: 50.0, speed_factor_pct: 10.0,
    kp: 0.001, ki: 0.0005, kd: 0.0}"

# Monitorar a FSM
ros2 topic echo /palpation/status

# Parar / pausar
ros2 topic pub --once /palpation/stop  std_msgs/msg/String "data: 'stop'"
ros2 topic pub --once /palpation/pause std_msgs/msg/Bool   "data: true"
```

Fases da FSM: `IDLE · HOME · DESCENDING · HOLD · SLIDING · RETRACT · DONE · ABORTED`

---

## Cinemática (`kinematics.py`)

FK e Jacobiano para o efetuador selecionado:

```python
T_TOUCH_TOOL_ATTACH  # TCP da ponta tátil — +188.5 mm em Z do Link6 (tcp_link)
T_HAND_ATTACH        # attach da mão COVVI (acoplador da prótese)
```

Cadeia URDF do touch_tool (acopladores encaixados na célula de carga):
```
Link6 → lower_coupling → force_sensor (+7mm) → upper_coupling (+59mm)
      → touch_tool (+74mm) → tcp_link (+188.5mm)
```

Convenção braço real ↔ URDF: offsets de junta tratados em `kinematics.py` (joints 2 e 4 têm offset vs. DH); `_HOME_Q` e `JOINT_MIN/MAX` também em convenção URDF.

---

## Modo MIRROR — espelho do robô real

Em MIRROR, os comandos publicados em `/cr10_group_controller/joint_trajectory` chegam ao CR10 real:

- **Palpação ativa**: `ServoJ` a 33 Hz com a posição de `/joint_states` (latência mínima p/ controle de força).
- **Jog manual (IDLE)**: `MovJ` com debounce de 80 ms a partir do último ponto publicado.
- **Drag Teach**: poll a 33 Hz lê o real e publica no Gazebo (mirror do movimento manual).
- **Sem GUI** (`no_gui:=true`): o `mirror_node` reproduz o núcleo desse comportamento.

Velocidade do braço real por `SpeedFactor(%)` — sincronizado com o slider da GUI (forçado a 10 % durante a palpação por segurança).

---

## Célula de carga (ESP32)

`force_receiver` abre UDP na porta **8080** e aguarda broadcasts da ESP32.

**Payload** (little-endian, 8 bytes): `float v_sensor; float force_filtered;` (o segundo é ignorado — a força é recalculada com a calibração do PC).

A calibração é lida de `~/.config/touch_pack/load_cell_calib.json` e recarregada periodicamente; é gerada pelo wizard da aba Célula de Carga.

---

## Arquivos persistentes

| Caminho | Conteúdo |
|---|---|
| `~/.config/touch_pack/robot.json` | IPs (mão/braço) + último modo (SIM_ONLY/MIRROR) |
| `~/.config/touch_pack/home_pose.json` | home customizada do braço |
| `~/.config/touch_pack/load_cell_calib.json` | slope/intercept da calibração |
| `~/.config/touch_pack/palpation_params.json` | últimos parâmetros da palpação (inclui o modo) |
| `~/.config/touch_pack/poses.json` | poses e movimentos gravados |
| `<repo>/sensors/Data/` | dados de palpação e do stream força+toque |

---

## Dependências

```bash
sudo apt install ros-humble-admittance-controller \
                 ros-humble-kinematics-interface-kdl \
                 ros-humble-force-torque-sensor-broadcaster
pip install "numpy<2" matplotlib pyserial   # matplotlib/pyserial: aba Sensores (opcionais)
```

> Sem `matplotlib`/`pyserial` a GUI segue funcionando em modo degradado (a aba Sensores fica desabilitada; o resto da palpação é normal).
