# hand_pack

Pacote de suporte da mão **COVVI Hand** — contém o URDF combinado CR10 + COVVI, helpers de launch, funções de pós-processamento do URDF e GUIs auxiliares para controle isolado da mão.

<p align="center">
  <img src="../../images/covvi_hand_rviz_joints_open.png" width="32%" alt="Mão COVVI no RViz — dedos abertos"/>
  <img src="../../images/covvi_hand_rviz_joints_closed.png" width="32%" alt="Mão COVVI no RViz — dedos fechados"/>
  <img src="../../images/gui_hand_tab_with_rviz.png" width="32%" alt="GUI de controle da mão COVVI ao lado do RViz"/>
</p>
<p align="center"><em>URDF combinado CR10 + COVVI (31 juntas: 6 primárias + 25 <em>mimic</em>) no RViz — dedos abertos e fechados — e a GUI de controle da mão.</em></p>

---

## Conteúdo

```
hand_pack/
├── urdf/
│   └── linear_covvi_hand_gazebo.urdf   URDF completo da mão COVVI (31 juntas: 6 primárias + 25 mimic)
├── config/
│   └── cr10_covvi_controllers.yaml     ros2_control para o modo hand: JTC braço + JTC mão
├── launch/
│   ├── cr10_covvi_gazebo.launch.py     CR10 + COVVI completo no Gazebo
│   ├── cr10_covvi_rviz.launch.py       CR10 + COVVI no RViz (só visualização)
│   ├── hand_gazebo.launch.py           Só a mão COVVI no Gazebo
│   ├── display.launch.py               URDF display com joint_state_publisher_gui
│   └── spawn_hand.launch.xml           Spawn da mão num Gazebo já rodando
├── hand_pack/
│   ├── urdf_helpers.py                 Funções de pós-processamento do URDF (limites, skin, colisão)
│   └── ...
└── meshes/                             STLs e malhas da mão COVVI
```

---

## Aplicação

O `hand_pack` não é um nó de runtime — é uma **biblioteca de recursos** usada pelos outros pacotes:

- `grasp_ml_pack` e `touch_pack` importam `urdf_helpers.py` no launch para pós-processar o URDF combinado.
- Os launches do `hand_pack` são usados para desenvolvimento e visualização isolada.
- `cr10_covvi_controllers.yaml` define os dois JTCs (braço + mão) e é referenciado pelo launch do `grasp_ml_pack`.

---

## Launches

```bash
# CR10 + COVVI completo no Gazebo (mesma stack do grasp_ml_pack, sem esteira/câmera)
ros2 launch hand_pack cr10_covvi_gazebo.launch.py

# CR10 + COVVI no RViz — só visualização, sem simulação física
ros2 launch hand_pack cr10_covvi_rviz.launch.py

# Só a mão COVVI no Gazebo (sem braço)
ros2 launch hand_pack hand_gazebo.launch.py

# URDF viewer com sliders de juntas (desenvolvimento)
ros2 launch hand_pack display.launch.py

# Spawn da mão num Gazebo já rodando
ros2 launch hand_pack spawn_hand.launch.xml
```

---

## Executáveis

```bash
# GUI standalone da mão — 6 sliders (Thumb/Index/Middle/Ring/Little/Rotate)
ros2 run hand_pack hand_gui

# GUI combinada — 6 juntas do CR10 + 6 digits COVVI
ros2 run hand_pack combined_gui
```

---

## Pós-processamento do URDF (`urdf_helpers.py`)

Três funções aplicadas em tempo de launch pelo `grasp_ml_pack` e `touch_pack`:

### `clamp_hand_joint_limits(urdf_body)`

Propaga os limites reais do firmware COVVI para o URDF. O URDF bruto do CAD usa `[0, 1.6 rad]` — o firmware clampa esses valores via `DigitConfigMsg.open_limit / close_limit`:

```python
HAND_DRIVER_LIMITS = {  # close_limit do firmware — slider 100%
    'Thumb': 1.00, 'Index': 1.00, 'Middle': 1.00,
    'Ring':  1.00, 'Little': 1.00, 'Rotate': 1.00,
}
HAND_DRIVER_LOWER = {   # open_limit do firmware — slider 0% (rest pose)
    'Thumb': 0.08, 'Index': 0.12, 'Middle': 0.12,
    'Ring':  0.12, 'Little': 0.12, 'Rotate': 0.00,
}
```

Os limites são propagados para as juntas mimic via `[mult · driver_lower, mult · driver_upper]`.

### `inject_visual_skin_layer(urdf_body)`

Adiciona uma camada visual inflada (~3 mm por face) sobre as falanges e palma, criando uma superfície contínua e suave no Gazebo. Usada para simular a pele da mão protética.

### `INTER_FINGER_COLLISION_LINKS`

Lista de links onde `self_collide=true` e `mu=2.5` são aplicados — permite que os dedos interajam fisicamente entre si e com objetos sem atravessar a geometria.

---

## Controllers (`cr10_covvi_controllers.yaml`)

Dois JTCs ativos no modo `hand`:

| Controller | Tipo | Joints |
|---|---|---|
| `cr10_group_controller` | `JointTrajectoryController` | joint1–joint6 |
| `hand_position_controller` | `JointTrajectoryController` | 6 primárias + 22 mimic (28 total) |

Taxa de atualização: 250 Hz. `allow_partial_joints_goal: true` no controlador da mão — permite enviar só os 6 drivers sem as mimic.

---

## Dependências

```xml
<depend>robot_state_publisher</depend>
<depend>rviz2</depend>
<exec_depend>gazebo_ros</exec_depend>
<exec_depend>gazebo_ros2_control</exec_depend>
<exec_depend>ros2_control</exec_depend>
<exec_depend>ros2_controllers</exec_depend>
<exec_depend>grasp_ml_pack</exec_depend>  <!-- cage_check (soft-import) -->
```
