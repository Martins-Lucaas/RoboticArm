# RoboticArm — Gêmeo Digital CR10 + COVVI · Célula de Manufatura Biomédica

> Braço industrial + mão protética biônica + visão computacional + esteira, em simulação Gazebo, com hook para a mão real via ECI — base de treinamento para usuários de próteses de mão de múltiplos graus de liberdade.

Integra o braço **Dobot CR10** com a mão protética **COVVI Hand** em um gêmeo digital completo no **ROS 2 Humble / Gazebo Classic 11**. O sistema simula uma célula industrial de manufatura biomédica: o robô identifica objetos farmacêuticos em uma esteira, classifica-os pelo tipo de preensão necessário e os deposita nas caixas de destino corretas. A mesma stack pode comandar a mão COVVI física via servidor ECI (`covvi_hand_driver`).

Este trabalho compõe o **Trabalho de Conclusão de Curso (TCC)** em Engenharia Biomédica cujo tema é o desenvolvimento de um sistema virtual de auxílio ao treinamento de usuários de próteses de mão com múltiplos graus de liberdade.

---

## Em ação

### Célula completa no Gazebo — visão geral

| Vista lateral da célula | Vista isométrica completa |
|---|---|
| ![Célula lateral](images/conveyor_cell_gazebo_full_scene.png) | ![Célula isométrica](images/conveyor_cell_gazebo_simulation_running.png) |

> Esteira transportadora à direita, braço CR10 com mão COVVI ao centro, três caixas de classificação (vermelha/verde/azul) à esquerda. Coluna de câmera montada atrás da esteira.

### GUI de controle + visão da câmera

| Estado ocioso — objeto na pick station | Braço estendido em fase de grasp |
|---|---|
| ![GUI Idle](images/conveyor_cell_gui_camera_idle.png) | ![GUI Picking](images/conveyor_cell_gui_arm_picking.png) |

### Detalhe da célula — caixas de classificação

| Closeup dos bins de destino | Visão de cima da célula |
|---|---|
| ![Bins](images/conveyor_cell_gazebo_closeup_bins.png) | ![Overview](images/conveyor_cell_gazebo_overview_early.png) |

### Hardware

| Braço Dobot CR10 | Mão COVVI |
|---|---|
| ![CR10](images/dobot_cr10_product_photo.jpeg) | ![COVVI](images/covvi_hand_product_photo.jpg) |

| RViz — dedos abertos | RViz — dedos fechados |
|---|---|
| ![Aberta](images/covvi_hand_rviz_joints_open.png) | ![Fechada](images/covvi_hand_rviz_joints_closed.png) |

---

## Contexto do TCC

Usuários de próteses de múltiplos graus de liberdade (MGL) enfrentam uma curva de aprendizado elevada: controlar individualmente cinco dedos para diferentes tipos de tarefa requer semanas de treino com terapeuta, hardware físico e objetos reais. O gêmeo digital permite que esse treinamento seja feito em simulação, antes do contato com o dispositivo físico — reduzindo custo, tempo e fadiga do usuário.

O diferencial é o **pipeline de grasp diferenciado por visão computacional**: o robô identifica automaticamente o objeto, seleciona o tipo de preensão adequado (palm / claw / fingertip) e executa a sequência de movimento — exatamente como um sistema de controle preditivo embarcado numa prótese faria. A mesma camada de comando pode disparar grips nativos do firmware ECI da mão COVVI real.

---

## Objetos e tipos de preensão

| Objeto | Descrição | Tipo de Grasp | Caixa de Destino |
|---|---|---|---|
| **Frasco** | Frasco de medicamento (âmbar, ∅84 mm, h=90 mm) | Palm Grip | Box 1 — vermelha |
| **Tubo** | Tubo de ensaio (azul, ∅24 mm, h=120 mm) | Claw Grip | Box 2 — verde |
| **Ampola** | Ampola farmacêutica (verde, ∅10 mm, h=75 mm) | Fingertip Grip | Box 3 — azul |

Cada objeto tem cor específica na simulação Gazebo para a segmentação HSV:

| Objeto | Cor Gazebo | Faixa HSV |
|---|---|---|
| Frasco | Âmbar/laranja | H=8-26, S>120, V>80 |
| Tubo | Azul rico | H=100-135, S>80, V>50 |
| Ampola | Verde brilhante | H=38-85, S>110, V>80 |

---

## Arquitetura do sistema

Pipeline com 5 nós ROS 2 e duas GUIs alternativas:

```
┌─────────────────────────────────────────────────────────────────────┐
│  Câmera RGB (Gazebo)                                                │
│  x=1.25, z=1.70, pitch=60°, yaw=180°  —  montada atrás da esteira   │
└──────────────────────────┬──────────────────────────────────────────┘
                           │ /camera/color/image_raw
                           ▼
┌──────────────────────────────────────────────────────────────────────┐
│  [object_detector]                                                   │
│  HSV (sim) ou YOLOv8 (real) → bounding box                           │
│  Back-projection geométrica: pixel (u,v) → world frame via R·d∩z     │
└──────────────────┬───────────────────────────────────────────────────┘
                   │ /detected_objects  (Detection2DArray + pose 3D)
                   ▼
┌──────────────────────────────────────────────────────────────────────┐
│  [grasp_executor]                                                    │
│  Recebe classe do objeto → escolhe grip + caixa destino              │
│  Calcula IK (multi-start geométrico + DLS) para todos os waypoints   │
│  Executa 7 fases (F1–F7) — articular + Cartesiano misto              │
└──────────────────┬───────────────────────────────────────────────────┘
                   │ /conveyor/retreat  (remove objeto após release)
                   │
┌──────────────────▼───────────────────────────────────────────────────┐
│  [conveyor_controller]                                               │
│  Gerencia sequência de objetos na esteira                            │
│  Spawn/delete no Gazebo via /spawn_entity e /delete_entity           │
└──────────────────────────────────────────────────────────────────────┘
           ▲                              ▲
           │ /conveyor/advance            │ /cell/execute_grasp
           │ /conveyor/retreat            │ /cell/go_home
           │ /conveyor/reset              │
┌──────────┴──────────────────────────────┴────────────────────────────┐
│  [gui_control]    GUI Tkinter padrão (esteira + grasp + status)      │
│  [manual_control] GUI alternativa: sliders por junta + grips ECI     │
└──────────────────────────────────────────────────────────────────────┘
           │
┌──────────▼───────────────────────────────────────────────────────────┐
│  [conveyor_pipeline]   Orquestrador (modo GUI ou autônomo)           │
│  Modo autônomo: advance → detect → execute → repeat (N ciclos)       │
└──────────────────────────────────────────────────────────────────────┘
```

### Ciclo de grasp — 7 fases

Cada chamada de `/cell/execute_grasp` executa, depois de resolver IK para todos os waypoints:

```
[F1] HOME → pick                     (articular, mão em paralelo: open → pré-grip)
[F2] Fechar mão sobre o objeto       (cfg_closed) + attach cinemático
[F3] Levantar com objeto             (lift_pos = pick + 22 cm)
[F4] Transit lateral → via_box       (Cartesiano, z=1.15 m world)
[F5] Descida → approach_box          (Cartesiano)
[F6] Soltar acima da caixa           (detach → open hand → /conveyor/retreat)
[F7] Retorno HOME                    (Cartesiano)
```

A trajetória usa **ease-in/out sinusoidal** com 8 waypoints por segmento articular e 6 waypoints Cartesianos. Validação AABB contra `_WORLD_OBSTACLES` (esteira, pedestal, paredes, prateleira de sort) é feita pelo executor antes de enviar qualquer trajetória.

---

## Cinemática

Implementação em `grasp_ml_pack/kinematics.py`. A convenção é **URDF nativa** (não Denavit-Hartenberg):

```
T_joint = T_origin × Rz(q)
T_origin = Translation(xyz) × R_rpy(roll, pitch, yaw)
```

### Cinemática Direta (FK)

`forward_kinematics(q, include_hand=True)` compõe as 6 transformações de origem URDF (extraídas direto do `cr10_robot.xacro`) e opcionalmente aplica `T_HAND_ATTACH` para chegar ao TCP da palma:

| Junta | xyz (m) | rpy (rad) |
|---|---|---|
| joint1 | `(0, 0, 0.1765)` | `(0, 0, 0)` |
| joint2 | `(0, 0, 0)` | `(π/2, π/2, 0)` |
| joint3 | `(-0.607, 0, 0)` | `(0, 0, 0)` |
| joint4 | `(-0.568, 0, 0.191)` | `(0, 0, -π/2)` |
| joint5 | `(0, -0.125, 0)` | `(π/2, 0, 0)` |
| joint6 | `(0, 0.1084, 0)` | `(-π/2, 0, 0)` |

`T_HAND_ATTACH` posiciona o TCP **75 mm abaixo do flange Link6**, no centro da zona onde os dedos COVVI se fecham — não no acoplamento `hand_base_link`. Isso garante que a IK trabalhe com o ponto real de captura.

### Cinemática Inversa (IK)

`inverse_kinematics(p_tcp, approach_vec, q_seed=None, elbow_up=True)`:

1. **Multi-start geométrico**: 14 palpites por chamada — varredura de `q1` ±0.7 rad para ambos os ramos de cotovelo, mais o `q_seed` se fornecido.
2. **Wrist analítico**: extrai `q4_urdf = θ4_DH − π/2` (offset URDF) testando os dois sinais de `q5`.
3. **Refinamento DLS desacoplado**: 4 ciclos de (DLS 3-DOF braço + recálculo analítico do pulso) seguidos de 100 iterações DLS 6-DOF com `λ` adaptativo (decay exponencial 0.06 → 0.003).

Limites articulares em convenção URDF (joints 2 e 4 com offset de −π/2 em relação ao DH):
```
JOINT_MIN = [-180°, -260°, -135°, -260°, -135°, -360°]
JOINT_MAX = [+180°,   80°, +135°,   80°, +135°, +360°]
```

`_HOME_Q = [0, 0, π/2, −π/2, −π/2, 0]` rad — braço erguido para trás, TCP ≈ (−0.69, −0.19, 1.31) m no frame robô.

Conversão de frames **obrigatória**: `base_link` do robô fica em `world z = 0.405 m` (spawn=0.375 + world_joint=0.030). A função `_w2r(pos)` subtrai esse offset antes do IK.

### Verificar a IK

```bash
ros2 run grasp_ml_pack test_kin
```
```
✓ pick station frasco  | err=  0.17 mm
✓ pick station tubo    | err=  0.03 mm
✓ pick station ampola  | err=  0.35 mm
✓ box1 (palm)          | err=  0.00 mm
✓ box2 (claw)          | err=  0.19 mm
✓ box3 (fingertip)     | err=  0.02 mm

Resultado geral: PASS ✓
```

### Cinemática da Mão COVVI

`hand_ik(grasp_type, obj_diameter)` retorna o dicionário `{Thumb, Index, Middle, Ring, Little, Rotate}` em rad. Tipos definidos em `HAND_CONFIGS`:

| Tipo | Uso |
|---|---|
| `open` / `pinch` / `cylindrical` / `spherical` | Genéricos |
| `palm_grip` | Frasco (Box 1) |
| `claw_grip` | Tubo (Box 2) |
| `fingertip_grip` | Ampola (Box 3) |

O atuador da mão tem 31 juntas no Gazebo (6 primárias + 25 mimic). Os multiplicadores de mimic vêm direto do URDF e ficam em `grasp_executor.py::_MIMIC_MAP`.

---

## Hardware

| Componente | Modelo | Specs |
|---|---|---|
| Braço | **Dobot CR10** | 6-DOF, alcance 1375 mm, payload 10 kg |
| Mão | **COVVI Hand** | 5 dedos + 31 juntas (6 primárias + 25 mimic), interface ECI Ethernet |
| Câmera | RGB Gazebo | 848×480, FoV 70°, x=1.25 m, z=1.70 m, pitch=60°, yaw=180° |

---

## Requisitos

| | Versão |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Gazebo | Classic 11 |
| Python | 3.10+ |

### Pacotes apt

```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control \
  ros-humble-xacro \
  ros-humble-joint-state-publisher-gui \
  ros-humble-vision-msgs \
  ros-humble-cv-bridge \
  ros-humble-control-msgs \
  python3-tk \
  python3-colcon-common-extensions \
  git
```

### Python

```bash
# numpy<2 é obrigatório — cv_bridge do Humble é compilado contra NumPy 1.x
pip install "numpy<2" opencv-python

# Opcional — só se for usar use_yolo:=true (detector real)
pip install ultralytics
```

---

## Instalação

### 1) Clonar este repositório

```bash
git clone https://github.com/Martins-Lucaas/RoboticArm.git ~/RoboticArm
cd ~/RoboticArm
```

### 2) Clonar a interface ECI da mão COVVI

A stack usa `covvi_interfaces` (definições de msg/srv) e `covvi_hand_driver` (servidor que fala com a mão física via Ethernet). Ambos vêm do repo oficial COVVI:

```bash
cd ~/RoboticArm/src
git clone git@github.com:COVVI-Robotics/eci_ros.git eci_ros-main
# ou via https:
# git clone https://github.com/COVVI-Robotics/eci_ros.git eci_ros-main
```

> Mesmo em modo só-simulação, o `covvi_interfaces` precisa estar compilado — o `manual_control_node` faz `import` lazy desses tipos para enviar `SetCurrentGrip` / `SetDigitPosn` quando o toggle de "Mão Real" está ligado.

### 3) Descrição URDF do CR10 — só o pacote mínimo

O repositório oficial da Dobot (`DOBOT_6Axis_ROS2_V4`) contém ~15 pacotes para variantes de braço (cr3/5/7/10/12/16/20, nova2/5, me6) e MoveIt. **Este projeto usa apenas `cra_description`** (URDF do CR10). Os demais não são compilados nem usados.

```bash
cd ~/RoboticArm/src
git clone https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4.git
# Mantém só o pacote necessário:
cd DOBOT_6Axis_ROS2_V4
find . -mindepth 1 -maxdepth 1 ! -name 'cra_description' -exec rm -rf {} +
```

Se preferir, pode mover `cra_description` para `src/` direto e deletar `DOBOT_6Axis_ROS2_V4` por completo:

```bash
cd ~/RoboticArm/src
mv DOBOT_6Axis_ROS2_V4/cra_description ./cra_description
rm -rf DOBOT_6Axis_ROS2_V4
```

> `cra_description` não tem dependência de nenhum outro pacote DOBOT — é só URDF/Xacro + meshes.

### 4) Compilar o workspace

```bash
cd ~/RoboticArm
colcon build --symlink-install
source install/setup.bash
```

> **Erro de symlink durante o build?** Se aparecer `symbolic link ... Is a directory`:
> ```bash
> rm -rf build install
> colcon build --symlink-install
> ```

---

## Rodando

### Modo manual (default) — célula completa com GUI

```bash
source install/setup.bash
ros2 launch grasp_ml_pack conveyor_cell.launch.py
```

O que sobe:
1. **Gazebo** carrega `conveyor_cell.world` (esteira, pedestal, caixas, câmera)
2. **Robot State Publisher** com URDF combinado CR10 + COVVI
3. **Controllers** em cadeia: `joint_state_broadcaster` → `cr10_group_controller` → `hand_position_controller`
4. **Nós da célula** sobem após `hand_position_controller` ativar
5. Janela **"Pick Station — Visão do Robô"** abre com feed da câmera + bounding boxes

Pronto quando o terminal mostrar:
```
[conveyor_controller] ConveyorController pronto | sequência: ['frasco', 'tubo', 'ampola']
[grasp_executor]      GraspExecutor pronto.
[object_detector]     ObjectDetector pronto — modo: HSV-simulação | objetos: frasco / tubo / ampola
```

**Fluxo típico pela GUI (`gui_control`):**

| Botão | Ação |
|---|---|
| `Avançar Esteira` | Spawna próximo objeto em x=0.75, y=0, z=2.0 (cai por gravidade) |
| `Recuar Esteira` | Remove objeto atual da pick station |
| `Resetar Esteira` | Reinicia a sequência (frasco→tubo→ampola) |
| `AGARRAR` | Dispara ciclo F1–F7 para o objeto detectado |
| `Home` | Envia braço para `_HOME_Q` |

### Flags úteis do launch

```bash
# Modo autônomo (sem intervenção da GUI)
ros2 launch grasp_ml_pack conveyor_cell.launch.py autonomous:=true

# Sem GUI (modo headless ou para usar manual_control noutro terminal)
ros2 launch grasp_ml_pack conveyor_cell.launch.py no_gui:=true

# Detector YOLOv8 ao invés de HSV (precisa de ultralytics instalado)
ros2 launch grasp_ml_pack conveyor_cell.launch.py use_yolo:=true

# Combinar: autônomo + sem GUI
ros2 launch grasp_ml_pack conveyor_cell.launch.py no_gui:=true autonomous:=true
```

### GUI alternativa — controle manual com sliders e grips ECI

```bash
ros2 launch grasp_ml_pack conveyor_cell.launch.py no_gui:=true
# Em outro terminal:
ros2 run grasp_ml_pack manual_control
```

A `manual_control` abre uma janela Tkinter com:
- **6 sliders** do braço (joint1–joint6, em graus, com limites URDF)
- **6 sliders** da mão (Thumb/Index/Middle/Ring/Little/Rotate, escala 0–200)
- **Botões de pose** pré-calculada (`Pick Frasco`, `Pick Tubo`, `Pick Ampola`)
- **Botões de preensão do projeto** (Palm/Claw/Fingertip → Gazebo)
- **Botões dos 14 grips ECI nativos** (Tripod/Power/Trigger/Prec.Open/...) que enviam para o Gazebo E para a mão real se o toggle estiver ligado
- **Botões da esteira** (spawn frasco/tubo/ampola, reset)
- **Toggle "Real Hand"** — habilita envio de `/SetCurrentGrip` e `/SetDigitPosn` para o servidor ECI

### Conectar à mão COVVI real

1. Conecte a mão à rede e descubra o IP (default COVVI: `192.168.1.123`).
2. Suba o servidor `covvi_hand_driver` em paralelo:

```bash
ros2 run covvi_hand_driver server 192.168.1.123 \
    --ros-args --remap __ns:=/covvi --remap __name:=hand
```

Isso cria os serviços `/covvi/hand/SetCurrentGrip`, `/covvi/hand/SetDigitPosn`, etc. (catálogo completo em `src/eci_ros-main/README.md`).

3. Ligue a energia da mão:
```bash
ros2 service call /covvi/hand/SetHandPowerOn covvi_interfaces/srv/SetHandPowerOn
```

4. Na GUI `manual_control`, marque **Real Hand: ON**. Os botões de grip passam a comandar simultaneamente o gêmeo digital e a mão física.

> **Endpoint diferente?** Passe `eci_prefix` ao manual_control:
> ```bash
> ros2 run grasp_ml_pack manual_control --ros-args -p eci_prefix:=/test/server_1
> ```

### Teach Pendant — gravar waypoints

```bash
ros2 run grasp_ml_pack teach_pendant
```

GUI com sliders + botões `+`/`−` por junta e:

| Botão | Ação |
|---|---|
| `Gravar Waypoint` | Captura configuração atual |
| `Ir para` | Envia braço para waypoint selecionado |
| `Remover` | Remove waypoint da lista |
| `Exportar YAML` | Salva em `.yaml` |
| `Exportar Python` | Gera snippet `TEACH_WAYPOINTS = [...]` |

`config/teach_sequence.yaml` traz os waypoints já gravados para a pick station do frasco.

---

## Catálogo completo de comandos

### `grasp_ml_pack` — pacote principal

**Launch files:**

```bash
# Célula completa (Gazebo + controllers + nós + GUI)
ros2 launch grasp_ml_pack conveyor_cell.launch.py

# Argumentos disponíveis (combináveis):
ros2 launch grasp_ml_pack conveyor_cell.launch.py use_yolo:=true
ros2 launch grasp_ml_pack conveyor_cell.launch.py sim_only:=false
ros2 launch grasp_ml_pack conveyor_cell.launch.py no_gui:=true
ros2 launch grasp_ml_pack conveyor_cell.launch.py autonomous:=true
ros2 launch grasp_ml_pack conveyor_cell.launch.py no_gui:=true autonomous:=true
```

| Argumento | Default | Efeito |
|---|---|---|
| `use_yolo` | `false` | `true` ativa detector YOLOv8 (precisa `ultralytics`) |
| `sim_only` | `true` | `false` desativa a parte de simulação do conveyor (uso com hardware real) |
| `no_gui` | `false` | `true` não sobe a `gui_control` (útil para rodar `manual_control` em paralelo) |
| `autonomous` | `false` | `true` faz o `pipeline` rodar advance→detect→execute em loop |

**Executáveis (`ros2 run grasp_ml_pack <nome>`):**

```bash
# Nós que normalmente são lançados pelo conveyor_cell.launch.py — rodam isolados se já houver Gazebo e controllers
ros2 run grasp_ml_pack object_detector
ros2 run grasp_ml_pack grasp_executor
ros2 run grasp_ml_pack conveyor_controller
ros2 run grasp_ml_pack pipeline

# GUIs (rodam de pé sozinhas; conectam ao restante via tópicos/serviços)
ros2 run grasp_ml_pack gui_control                # GUI padrão (esteira + grasp)
ros2 run grasp_ml_pack manual_control             # GUI alternativa: sliders por junta + grips ECI
ros2 run grasp_ml_pack teach_pendant              # jog + waypoint recorder

# Manual control com prefixo ECI customizado:
ros2 run grasp_ml_pack manual_control --ros-args -p eci_prefix:=/test/server_1

# Teste de cinemática (unitário)
ros2 run grasp_ml_pack test_kin
```

### `hand_pack` — controle isolado da mão + URDF combinado

**Launch files:**

```bash
# CR10 + COVVI completo no Gazebo (mesma stack física do conveyor_cell, sem esteira/câmera/detector)
ros2 launch hand_pack cr10_covvi_gazebo.launch.py

# CR10 + COVVI no RViz (sem Gazebo — só visualização do URDF + TF)
ros2 launch hand_pack cr10_covvi_rviz.launch.py

# Apenas a mão COVVI no Gazebo (sem braço CR10)
ros2 launch hand_pack hand_gazebo.launch.py

# Display URDF simples — robot_state_publisher + joint_state_publisher_gui
ros2 launch hand_pack display.launch.py
ros2 launch hand_pack display.launch.py use_sim_time:=true

# Spawn da mão num Gazebo já rodando (XML launch)
ros2 launch hand_pack spawn_hand.launch.xml
```

**Executáveis (`ros2 run hand_pack <nome>`):**

```bash
ros2 run hand_pack hand_gui          # GUI Tkinter standalone só da mão (6 sliders dos digits)
ros2 run hand_pack combined_gui      # GUI combinada: 6 juntas CR10 + 6 digits COVVI
```

### `covvi_hand_driver` — servidor para a mão COVVI real

```bash
# IP da mão + namespace/nome do servidor (cria /covvi/hand/* serviços e tópicos)
ros2 run covvi_hand_driver server <IP_DA_MÃO> \
    --ros-args --remap __ns:=/covvi --remap __name:=hand

# Exemplo concreto:
ros2 run covvi_hand_driver server 192.168.1.123 \
    --ros-args --remap __ns:=/covvi --remap __name:=hand
```

---

## Tópicos principais

| Tópico | Tipo | Descrição |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB bruto da câmera Gazebo |
| `/detector/debug_image` | `sensor_msgs/Image` | Imagem anotada com bounding boxes |
| `/detected_objects` | `vision_msgs/Detection2DArray` | Classe + posição 3D world |
| `/conveyor/status` | `std_msgs/String` JSON | Estado da esteira |
| `/conveyor/advance` | `std_srvs/Trigger` | Spawna próximo objeto |
| `/conveyor/retreat` | `std_srvs/Trigger` | Remove objeto atual |
| `/conveyor/reset` | `std_srvs/Trigger` | Reinicia sequência |
| `/conveyor/spawn_{frasco,tubo,ampola}` | `std_srvs/Trigger` | Spawn específico (usado por `manual_control`) |
| `/cell/execute_grasp` | `std_srvs/Trigger` | Dispara ciclo F1–F7 |
| `/cell/go_home` | `std_srvs/Trigger` | Envia braço ao home |
| `/cell/status` | `std_msgs/String` JSON | Estado do executor |
| `/cr10_group_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Comandos para o braço |
| `/hand_position_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Comandos para a mão simulada |
| `/joint_states` | `sensor_msgs/JointState` | Posição das 37 juntas |
| `/covvi/hand/SetCurrentGrip` | `covvi_interfaces/srv/SetCurrentGrip` | (Real) Set grip nativo da mão |
| `/covvi/hand/SetDigitPosn` | `covvi_interfaces/srv/SetDigitPosn` | (Real) Posições absolutas dos 6 digits |
| `/covvi/hand/CurrentGripMsg` | `covvi_interfaces/msg/CurrentGripMsg` | (Real) Grip ativo na mão |
| `/covvi/hand/DigitPosnAllMsg` | `covvi_interfaces/msg/DigitPosnAllMsg` | (Real) Posições reais em tempo real |

---

## Estrutura do projeto

```
RoboticArm/
├── images/                              screenshots e mídia
├── teach_sequence.yaml                  waypoints gravados
├── collision_analysis.py                análise offline de colisão (FK + AABB)
├── projeto_grasp_autonomo_ml.txt        documento técnico do TCC
├── src/
│   ├── grasp_ml_pack/                   pacote principal — célula de manufatura
│   │   ├── config/
│   │   │   ├── pipeline_params.yaml     parâmetros de todos os nós
│   │   │   └── teach_sequence.yaml      cópia dos waypoints gravados
│   │   ├── grasp_ml_pack/
│   │   │   ├── kinematics.py            FK/IK URDF, hand_ik COVVI
│   │   │   ├── object_detector.py       HSV / YOLOv8 + back-projection 2D→3D
│   │   │   ├── grasp_executor.py        ciclo F1–F7 com validação AABB
│   │   │   ├── conveyor_controller.py   spawn/delete via Gazebo services
│   │   │   ├── gui_control_node.py      GUI padrão (esteira + grasp)
│   │   │   ├── manual_control_node.py   GUI alternativa: sliders + grips ECI
│   │   │   ├── teach_pendant.py         jog + waypoint recorder
│   │   │   └── pipeline.py              orquestrador GUI/autônomo
│   │   ├── launch/
│   │   │   └── conveyor_cell.launch.py  launch principal
│   │   └── worlds/
│   │       └── conveyor_cell.world      cena Gazebo
│   ├── hand_pack/                       URDF da mão + GUIs auxiliares
│   │   ├── urdf/linear_covvi_hand_gazebo.urdf
│   │   └── launch/cr10_covvi_gazebo.launch.py
│   ├── cra_description/                 URDF do CR10 (extraído do DOBOT_6Axis_ROS2_V4)
│   └── eci_ros-main/                    interface oficial COVVI
│       ├── covvi_interfaces/            msg/srv (compilação obrigatória)
│       ├── covvi_hand_driver/           servidor TCP/ECI para a mão real
│       └── covvi_urdf/                  URDF de referência (não usado aqui)
└── log/, build/, install/               artefatos do colcon
```

---

## Comandos úteis

```bash
# Rebuild só o pacote principal
colcon build --packages-select grasp_ml_pack --symlink-install
source install/setup.bash

# Ver o que a câmera enxerga (alternativa ao imshow)
ros2 run rqt_image_view rqt_image_view /detector/debug_image

# Monitorar estados
ros2 topic echo /conveyor/status
ros2 topic echo /cell/status

# Disparar ações por terminal
ros2 service call /conveyor/advance std_srvs/srv/Trigger {}
ros2 service call /conveyor/retreat std_srvs/srv/Trigger {}
ros2 service call /conveyor/reset   std_srvs/srv/Trigger {}
ros2 service call /cell/execute_grasp std_srvs/srv/Trigger {}
ros2 service call /cell/go_home       std_srvs/srv/Trigger {}

# Verificar controllers
ros2 control list_controllers

# Matar Gazebo travado
pkill -f gzserver; pkill -f gzclient

# Testar IK isoladamente
ros2 run grasp_ml_pack test_kin

# Análise de colisão offline
python3 collision_analysis.py
```

---

## Diagnóstico rápido

| Sintoma | Causa provável | Solução |
|---|---|---|
| `[ros2run]: Segmentation fault` ao iniciar `manual_control` | Emoji astral no Tcl/Tk 8.6 | Já corrigido — rebuild se voltar a aparecer |
| Objeto não aparece na câmera | Objeto ainda caindo / HSV errado | Aguardar 2-3 s após spawn |
| `AGARRAR` retorna "Nenhum objeto válido" | Detector sem detecção ativa | Re-avançar a esteira; conferir janela da câmera |
| `Goal rejeitado` no terminal | Controller não ativo | `ros2 control list_controllers` |
| Braço para no meio do ciclo | IK falhou em alguma pose | Conferir logs `[FALHA]` do `grasp_executor` |
| Objeto cai fora da pick station | `spawn_z` muito baixo | `spawn_z: 2.0` em `pipeline_params.yaml` |
| `ModuleNotFoundError: covvi_interfaces` | `eci_ros-main` não clonado/compilado | Passos 2 e 4 da instalação |
| `Real Hand: ON` falha em conectar | Servidor `covvi_hand_driver` não rodando | Subir o servidor com IP correto |

---

## RViz — Visualizações da mão COVVI

| Malha visual completa | Malha de colisão |
|---|---|
| ![Interna](images/covvi_hand_3d_design_internal.png) | ![Colisão](images/covvi_hand_rviz_collision_mesh.png) |

| Blender — constraints | CAD — dedos estendidos |
|---|---|
| ![Constraints](images/covvi_hand_blender_constraints.png) | ![CAD](images/covvi_hand_cad_fingers_extended.png) |

---

## Licença

Apache-2.0

Desenvolvido por **Lucas Martins** — [lucaspmartins14@gmail.com](mailto:lucaspmartins14@gmail.com)
TCC — Engenharia Biomédica
