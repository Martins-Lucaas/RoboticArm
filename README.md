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

# Opcional — detector YOLOv8 (apenas grasp_ml_pack com use_yolo:=true)
pip install ultralytics
```

### Repositórios externos obrigatórios

| Repo | Onde clonar | Função |
|---|---|---|
| `DOBOT_6Axis_ROS2_V4` (só `cra_description`) | `src/` | URDF/Xacro do CR10 |
| `eci_ros` (COVVI oficial) | `src/eci_ros-main/` | `covvi_interfaces` (msg/srv) + `covvi_hand_driver` (servidor TCP/ECI) |

> **Nota:** mesmo em modo só-simulação, `covvi_interfaces` precisa estar compilado — o nó de controle manual faz import lazy desses tipos para enviar comandos à mão real quando habilitado.

---

## Instalação

```bash
# 1. Clonar este repositório
git clone https://github.com/Martins-Lucaas/RoboticArm.git ~/RoboticArm
cd ~/RoboticArm/src

# 2. Interface ECI da mão COVVI
git clone https://github.com/COVVI-Robotics/eci_ros.git eci_ros-main

# 3. URDF do CR10 (só o pacote cra_description)
git clone https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4.git
cd DOBOT_6Axis_ROS2_V4
find . -mindepth 1 -maxdepth 1 ! -name 'cra_description' -exec rm -rf {} +
cd ~/RoboticArm

# 4. Compilar
colcon build --symlink-install
source install/setup.bash
```

> Se aparecer `symbolic link ... Is a directory` na compilação: `rm -rf build install && colcon build --symlink-install`

---

## Guia rápido por pacote

| Pacote | Função principal | README |
|---|---|---|
| **`grasp_ml_pack`** | Célula de manufatura: esteira, detecção de objetos, pick-and-place com COVVI | [→ grasp_ml_pack/README.md](src/grasp_ml_pack/README.md) |
| **`hand_pack`** | URDF combinado CR10 + COVVI, GUIs da mão, helpers de launch | [→ hand_pack/README.md](src/hand_pack/README.md) |
| **`touch_pack`** | Célula de palpação tátil: protocolo Gupta 2021, GUI, logging, célula de carga ESP32 | [→ touch_pack/README.md](src/touch_pack/README.md) |
| **`cra_description`** | URDF/Xacro do braço Dobot CR10 (extraído do repositório oficial Dobot) | [→ cra_description/README.md](src/cra_description/README.md) |

### Iniciar rapidamente

```bash
source install/setup.bash

# Célula de manufatura (pick & place)
ros2 launch grasp_ml_pack conveyor_cell.launch.py

# Célula de palpação tátil
ros2 launch touch_pack tactile_cell.launch.py

# Só a mão COVVI no Gazebo
ros2 launch hand_pack cr10_covvi_gazebo.launch.py
```

---

## Estrutura do workspace

```
RoboticArm/
├── src/
│   ├── grasp_ml_pack/       célula de manufatura — detecção, grasp, GUI
│   ├── hand_pack/           URDF COVVI + launches + GUIs auxiliares
│   ├── touch_pack/          palpação tátil — FSM, GUI, força, logging
│   ├── cra_description/     URDF do CR10 (fonte: DOBOT_6Axis_ROS2_V4)
│   └── eci_ros-main/        interface oficial COVVI (covvi_interfaces + covvi_hand_driver)
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
