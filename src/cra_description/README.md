# cra_description

Pacote com o **URDF/Xacro e meshes do braço Dobot CR10** (modelo CR10A, 6-DOF). Extraído do repositório oficial Dobot [`DOBOT_6Axis_ROS2_V4`](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4) — apenas este pacote é usado; os demais (cr3, cr5, cr7, MoveIt, nova2, etc.) podem ser descartados.

---

## Conteúdo

```
cra_description/
├── urdf/
│   └── cr10_robot.xacro          URDF principal do CR10 (6 juntas + ros2_control)
├── meshes/
│   └── *.stl / *.dae             Malhas visuais e de colisão de cada link
└── package.xml
```

---

## Como obter

```bash
cd ~/RoboticArm/src
git clone https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4.git

# Manter só o cra_description
cd DOBOT_6Axis_ROS2_V4
find . -mindepth 1 -maxdepth 1 ! -name 'cra_description' -exec rm -rf {} +

# Ou mover diretamente para src/ e deletar o clone
cd ~/RoboticArm/src
mv DOBOT_6Axis_ROS2_V4/cra_description ./cra_description
rm -rf DOBOT_6Axis_ROS2_V4
```

---

## Uso no projeto

O `cr10_robot.xacro` é processado em tempo de launch pelos pacotes `grasp_ml_pack` e `touch_pack`:

```python
import xacro
doc = xacro.parse(open(cr10_xacro_path))
xacro.process_doc(doc)
cr10_urdf = doc.toxml()
```

O URDF resultante é injetado com o efector final (mão COVVI ou touch_tool) e publicado no `robot_state_publisher`. Nenhuma modificação é feita nos arquivos originais do pacote.

---

## Parâmetros cinemáticos (juntas em convenção URDF)

| Junta | xyz (m) | rpy (rad) |
|---|---|---|
| joint1 | `(0, 0, 0.1765)` | `(0, 0, 0)` |
| joint2 | `(0, 0, 0)` | `(π/2, π/2, 0)` |
| joint3 | `(-0.607, 0, 0)` | `(0, 0, 0)` |
| joint4 | `(-0.568, 0, 0.191)` | `(0, 0, -π/2)` |
| joint5 | `(0, -0.125, 0)` | `(π/2, 0, 0)` |
| joint6 | `(0, 0.1084, 0)` | `(-π/2, 0, 0)` |

A convenção de sinal das juntas no URDF é idêntica à do firmware Dobot TCP/IP V4 — offset `_URDF_DOBOT_OFFSET = np.zeros(6)` em `kinematics.py`.
