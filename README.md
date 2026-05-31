<div align="center">

# RoboticArm
### Gêmeo Digital · CR10 + COVVI Hand · Célula de Manufatura Biomédica

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Gazebo Classic 11](https://img.shields.io/badge/Gazebo-Classic%2011-FCBA28?style=for-the-badge)](http://classic.gazebosim.org/)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04%20LTS-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/22.04/)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)
[![License Apache-2.0](https://img.shields.io/badge/License-Apache--2.0-D22128?style=for-the-badge)](LICENSE)

</div>

Gêmeo digital do braço industrial **Dobot CR10** acoplado à mão protética biônica **COVVI Hand**, rodando em **ROS 2 Humble / Gazebo Classic 11**. O sistema identifica objetos farmacêuticos em uma esteira, classifica-os pelo tipo de preensão necessário e os deposita nas caixas corretas — com canal direto para a mão COVVI física via protocolo ECI Ethernet.

Componente do **Trabalho de Conclusão de Curso (TCC) em Engenharia Biomédica** — plataforma virtual de auxílio ao treinamento de usuários de próteses de mão com múltiplos graus de liberdade. O mesmo hardware (CR10 + COVVI) é reutilizado em uma segunda célula de **palpação tátil**, que reproduz o protocolo de Gupta et al. 2021 com controle de força e deslizamento Cartesiano.

---

## Hardware

| Componente | Modelo | Especificações |
|---|---|---|
| Braço | **Dobot CR10** | 6-DOF, alcance 1375 mm, payload 10 kg, protocolo TCP/IP V4 |
| Mão | **COVVI Hand** | 5 dedos + 31 juntas (6 primárias + 25 mimic), interface ECI Ethernet |
| Câmera | RGB Gazebo | 848×480, FoV 70°, montada atrás da esteira |
| Célula de carga | ESP32 + sensor uniaxial | UDP broadcast 8080, 8 bytes por pacote (`float v_sensor, float force`) |

---

## Dependências completas

### Sistema operacional

| Dependência | Versão |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Gazebo | Classic 11 |
| Python | 3.10+ |

### Pacotes apt

```bash
sudo apt update && sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control \
  ros-humble-xacro \
  ros-humble-joint-state-publisher-gui \
  ros-humble-vision-msgs \
  ros-humble-cv-bridge \
  ros-humble-control-msgs \
  ros-humble-admittance-controller \
  ros-humble-kinematics-interface-kdl \
  ros-humble-force-torque-sensor-broadcaster \
  python3-tk \
  python3-colcon-common-extensions \
  git
```

### Python

```bash
# numpy<2 é obrigatório — cv_bridge do Humble é compilado contra NumPy 1.x
pip install "numpy<2" opencv-python

# Driver da mão COVVI — biblioteca ECI proprietária da COVVI Robotics
pip install covvi-eci==1.1.6

# Opcional — detector YOLOv8 (apenas grasp_ml_pack com use_yolo:=true)
pip install ultralytics
```

### Dependências externas — já resolvidas pelo repositório

| Pacote | Origem | Como entra |
|---|---|---|
| `cra_description` | extraído de [`Dobot-Arm/DOBOT_6Axis_ROS2_V4`](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4) | **já versionado** em `src/cra_description` — não precisa clonar nada |
| `covvi_interfaces` + `covvi_hand_driver` (`src/eci_ros`) | **submódulo git** | clonado automaticamente com `git clone --recursive` (ver Instalação) |

> **Créditos — driver da mão COVVI:** `src/eci_ros` é de autoria da **COVVI Robotics** ([`COVVI-Robotics/eci_ros`](https://github.com/COVVI-Robotics/eci_ros)). Ele entra como submódulo apontando para um fork ([`Martins-Lucaas/eci_ros`](https://github.com/Martins-Lucaas/eci_ros)) que **preserva integralmente a autoria original** e adiciona apenas um ajuste downstream de shutdown/reconexão da sessão ECI.

> **Nota:** mesmo em modo só-simulação, `covvi_interfaces` precisa estar compilado — vários nós fazem import lazy desses tipos para comandar a mão real quando habilitada.

---

## Instalação

```bash
# 1. Clonar o repositório COM o submódulo eci_ros (driver da mão COVVI)
git clone --recursive https://github.com/Martins-Lucaas/RoboticArm.git ~/RoboticArm
cd ~/RoboticArm

#    Já clonou sem --recursive? Puxe o submódulo:
#    git submodule update --init --recursive

# 2. Instalar dependências (apt + Python — ver seção "Dependências completas")

# 3. Compilar o workspace inteiro e dar source
colcon build --symlink-install
source install/setup.bash
```

`cra_description` (URDF do CR10) já vem versionado no repositório; `eci_ros`
chega pelo submódulo. Não é preciso clonar nada além do passo 1.

> **Atualizar o submódulo depois:** `git submodule update --remote src/eci_ros`
> **Erro `symbolic link ... Is a directory` na compilação:** `rm -rf build install && colcon build --symlink-install`
> **Sempre dê `source install/setup.bash`** em cada terminal novo antes de qualquer `ros2 launch`/`ros2 run`.

---

## Guia rápido por pacote

| Pacote | Função principal | README |
|---|---|---|
| **`grasp_ml_pack`** | Célula de manufatura: esteira, detecção de objetos, pick-and-place com COVVI | [→ grasp_ml_pack/README.md](src/grasp_ml_pack/README.md) |
| **`hand_pack`** | URDF combinado CR10 + COVVI, GUIs da mão, helpers de launch | [→ hand_pack/README.md](src/hand_pack/README.md) |
| **`touch_pack`** | Célula de palpação tátil: protocolo Gupta 2021, GUI, logging, célula de carga ESP32 | [→ touch_pack/README.md](src/touch_pack/README.md) |
| **`cra_description`** | URDF/Xacro do braço Dobot CR10 (extraído do repositório oficial Dobot) | [→ cra_description/README.md](src/cra_description/README.md) |

### Como rodar (cada pacote principal)

> Em **todo** terminal novo: `cd ~/RoboticArm && source install/setup.bash`

#### 1. Célula de manufatura — `grasp_ml_pack`
Esteira + detecção de objetos + pick-and-place com a mão COVVI.

```bash
ros2 launch grasp_ml_pack conveyor_cell.launch.py
```
| Argumento | Valores | Padrão | Efeito |
|---|---|---|---|
| `use_yolo` | `true` / `false` | `false` | usa detector YOLOv8 (requer `pip install ultralytics`) |
| `sim_only` | `true` / `false` | `true` | `false` habilita canal para o CR10/COVVI reais |
| `autonomous` | `true` / `false` | `false` | roda o pipeline pick-and-place sem intervenção |
| `no_gui` | `true` / `false` | `false` | sobe sem a GUI de controle |

```bash
# Exemplo: autônomo com YOLO
ros2 launch grasp_ml_pack conveyor_cell.launch.py use_yolo:=true autonomous:=true
```

#### 2. Célula de palpação tátil — `touch_pack`
Protocolo Gupta 2021: controle de força/deslizamento + GUI + logging + célula de carga.

```bash
ros2 launch touch_pack tactile_cell.launch.py
```
| Argumento | Valores | Padrão | Efeito |
|---|---|---|---|
| `end_effector` | `hand` / `touch_tool` | `hand` | `hand` → controle da mão COVVI; `touch_tool` → ponta de palpação + célula de carga |
| `control_mode` | `sim_only` / `mirror` / `real_from_sim` | `sim_only` | espelhamento/comando do CR10 real |
| `robot_ip` | IP | `192.168.5.2` | IP do controlador CR10 real |
| `robot_dry_run` | `true` / `false` | `true` | `true` não abre sockets do robô real |
| `no_gui` | `true` / `false` | `false` | sobe sem a GUI de palpação |

```bash
# Palpação com a ponta tátil (modo Palpação liberado + leitura da célula de carga na GUI):
ros2 launch touch_pack tactile_cell.launch.py end_effector:=touch_tool

# Mão COVVI (modo Palpação bloqueado; GUI mostra o controle da mão):
ros2 launch touch_pack tactile_cell.launch.py end_effector:=hand
```

#### 3. Mão COVVI no braço CR10 (só visualização) — `hand_pack`
```bash
ros2 launch hand_pack cr10_covvi_gazebo.launch.py     # CR10 + COVVI no Gazebo
ros2 launch hand_pack cr10_covvi_rviz.launch.py        # idem no RViz
```

#### Conectar o hardware real (opcional)
- **Mão COVVI:** pela GUI (`touch_pack`/`grasp_ml_pack`), informe o IP e clique **Conectar** → **ECI ON** → **PWR ON**. Internamente sobe `ros2 run covvi_hand_driver server <IP>`.
- **CR10:** ajuste `robot_ip` e use `control_mode:=mirror` (espelha o real no sim) ou `real_from_sim`. Para o *drag teach*, coloque o controlador em **modo REMOTE** no teach pendant.
- **Célula de carga (ESP32):** firmware em `sensors/ForceDriver/`; transmite por UDP broadcast na porta 8080.

---

## Estrutura do workspace

```
RoboticArm/
├── src/
│   ├── grasp_ml_pack/       célula de manufatura — detecção, grasp, GUI
│   ├── hand_pack/           URDF COVVI + launches + GUIs auxiliares
│   ├── touch_pack/          palpação tátil — FSM, GUI, força, logging
│   ├── cra_description/     URDF do CR10 (fonte: DOBOT_6Axis_ROS2_V4 — já incluso)
│   └── eci_ros/             submódulo → COVVI eci_ros (covvi_interfaces + covvi_hand_driver)
├── images/                  screenshots e mídia
├── sensors/ForceDriver/     firmware ESP32 da célula de carga
└── build/, install/, log/   artefatos do colcon (gerados)
```

---

## Licença

<div align="center">

**Apache-2.0**

Desenvolvido por **Lucas Martins** · [lucaspmartins14@gmail.com](mailto:lucaspmartins14@gmail.com)

TCC — Engenharia Biomédica

</div>
