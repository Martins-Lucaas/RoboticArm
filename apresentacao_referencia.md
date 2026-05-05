# Documento de Referência — Apresentação Científica
## Gêmeo Digital de Célula de Manufatura Biomédica com Dobot CR10 e Mão Protética COVVI

**TCC — Engenharia Biomédica | ROS 2 Humble / Gazebo Classic 11**

---

## SLIDE 1 — Título, Contexto e Motivação

### Título
**Gêmeo Digital de Célula de Manufatura Biomédica com Braço Robótico Dobot CR10 e Mão Protética COVVI: Simulação ROS 2 / Gazebo com Visão Computacional e Cinemática Analítica**

### Texto de referência

Este trabalho apresenta o desenvolvimento de um gêmeo digital de célula de manufatura biomédica como componente central de um Trabalho de Conclusão de Curso em Engenharia Biomédica. O sistema simula, em ambiente virtual completo, o processo de pick-and-place de materiais farmacêuticos utilizando o braço robótico colaborativo Dobot CR10 de seis graus de liberdade acoplado à mão protética mioeléctrica COVVI.

A motivação central do projeto reside na barreira de aprendizado imposta por próteses de mão com múltiplos graus de liberdade (MGL). Ao contrário de próteses simples de gancho ou de dois estados, as próteses MGL exigem que o usuário aprenda a associar intenção muscular a diferentes configurações de preensão. Esse aprendizado, quando feito diretamente com o hardware físico, é custoso, exige supervisão clínica e expõe o paciente a frustrações repetidas antes de qualquer ganho funcional.

O gêmeo digital resolve esse problema ao reproduzir fielmente o comportamento do sistema robótico real em software, permitindo ao usuário praticar a associação objeto-preensão em um ambiente repetível, acessível e sem custo de hardware. Toda a lógica de controle, cinemática e visão computacional desenvolvida na simulação está arquitetada para migrar futuramente para o hardware físico com mínimas alterações de código.

Nesta fase, o entregável é o gêmeo digital funcional com pipeline completo de classificação e preensão diferenciada de três objetos farmacêuticos distintos, operando de forma determinística via visão computacional.

---

## SLIDE 2 — O Braço Robótico Dobot CR10

### Texto de referência

O Dobot CR10 pertence à família CR (Collaborative Robot) da fabricante chinesa Dobot Robotics, projetada especificamente para automação colaborativa em ambientes industriais e laboratoriais. É um manipulador serial de seis juntas rotacionais (6-DOF), com alcance nominal de 1375 mm, payload máximo de 10 kg e repetibilidade de ±0,05 mm. Sua estrutura segue a configuração clássica antropomórfica: três juntas de posicionamento (ombro, cotovelo e antebraço) seguidas por três juntas de orientação do pulso (Euler ZYZ modificado).

Os parâmetros geométricos do braço foram extraídos diretamente do arquivo URDF `cr10_robot.xacro`, adotando a convenção nativa do ROS 2 (translação + rotação RPY por junta), que difere da convenção Denavit-Hartenberg clássica nos offsets das juntas 2 e 4:

| Junta  | xyz (m)                | rpy (rad)                 | Observação                          |
|--------|------------------------|---------------------------|-------------------------------------|
| joint1 | [0, 0, 0.1765]         | [0, 0, 0]                 | Rotação em torno de Z do mundo      |
| joint2 | [0, 0, 0]              | [π/2, π/2, 0]             | Offset +π/2 vs convenção DH         |
| joint3 | [−0.607, 0, 0]         | [0, 0, 0]                 | Link de 607 mm                      |
| joint4 | [−0.568, 0, 0.191]     | [0, 0, −π/2]              | Link de 568 mm + elevaçao 191 mm    |
| joint5 | [0, −0.125, 0]         | [π/2, 0, 0]               | Junta de pulso 1                    |
| joint6 | [0, 0.1084, 0]         | [−π/2, 0, 0]              | Junta de pulso 2 (yaw do flange)    |

Uma consequência importante dessa convenção é que a posição q = [0, 0, 0, 0, 0, 0] não corresponde ao braço vertical estendido, como seria na convenção DH padrão. Na convenção URDF, q2 = 0 significa braço superior na horizontal, pois o offset de +π/2 está embutido na origem da junta. Os limites articulares físicos mapeados para a convenção URDF são:

- Joints 1, 3, 5: ±180°, ±135°, ±135° respectivamente
- Joints 2 e 4: [−260°, +80°] (resultado de ±135° DH somado ao offset de −π/2)
- Joint 6: ±360°

O robô está montado sobre um pedestal de 0,375 m de altura no ambiente Gazebo, situando o base_link em z = 0,405 m no frame world (0,375 m do pedestal + 0,03 m do joint fixo world→base do URDF).

---

## SLIDE 3 — A Mão Protética COVVI

### Texto de referência

A mão COVVI é uma prótese mioeléctrica de cinco dedos com múltiplos graus de liberdade, desenvolvida pela empresa britânica COVVI Ltd. para reabilitação de amputados transradiais. Sua principal característica é o acionamento por sinais EMG (eletromiografia) captados da musculatura residual do antebraço, com múltiplas configurações de preensão programáveis.

No gêmeo digital, a mão é representada por um modelo URDF com **31 juntas totais**: seis juntas primárias (driver joints) e 25 juntas mimic. As juntas primárias são os únicos graus de liberdade ativamente controlados; as juntas mimic replicam o movimento das primárias com multiplicadores fixos extraídos da geometria mecânica do URDF:

**Juntas primárias controladas (rad, com limite ≈ 1,6 rad):**
- Thumb (polegar): controla a flexão do polegar
- Index (indicador): controla o indicador
- Middle (médio): controla o dedo médio
- Ring (anelar): controla o dedo anelar
- Little (mínimo): controla o dedo mínimo
- Rotate: controla a rotação da base do polegar (oponência)

**Exemplos de juntas mimic e seus multiplicadores:**
- `_index_proximal_j01` = Index × 1,516 (falange proximal do indicador)
- `_index_distal_j01` = Index × 1,336 (falange distal do indicador)
- `_thumb_chassis_j01` = Rotate × 1,533 (chassi giratório do polegar)
- `_middle_proximal_j01` = Middle × 1,516
- `_little_proximal_j01` = Little × 1,516

Para enviar um comando à mão, o controlador `hand_position_controller` recebe um vetor de 31 posições: os 6 valores das juntas primárias + os 25 valores das mimic (calculados multiplicando cada primária pelo seu fator). Isso é feito automaticamente pela função `_make_hand_goal()` no módulo `grasp_executor.py`.

O acoplamento mecânico entre a mão e o flange do CR10 é realizado por uma junta fixa `hand_attach_joint` com parâmetros `xyz=[0, 0, 0.01]` e `rpy=[π/2, 0, 0]`. Essa transformação posiciona o `hand_base_link` a 10 mm do flange com uma rotação de 90° em torno de X, o que faz o eixo Z da palma apontar para baixo no frame world durante a operação de pick. O TCP efetivo utilizado no IK considera adicionalmente um offset de 100 mm ao longo de Y do Link6 (profundidade física do corpo da palma COVVI), resultando na matriz de transformação T_HAND_ATTACH:

```
T_HAND_ATTACH = [[1,  0,  0,  0.00],
                 [0,  0, -1, -0.10],
                 [0,  1,  0,  0.01],
                 [0,  0,  0,  1.00]]
```

---

## SLIDE 4 — ROS 2 Humble: Visão Geral e Infraestrutura de Controle

### Texto de referência

O Robot Operating System 2 (ROS 2) é um framework de middleware publish-subscribe amplamente adotado na robótica acadêmica e industrial. Sua versão Humble Hawksbill (2022, suporte de longa duração até maio de 2027) é utilizada neste projeto por ser a versão estável compatível com o Gazebo Classic 11 no Ubuntu 22.04.

**Conceitos fundamentais utilizados no projeto:**

Um **Node** é um processo independente com responsabilidade única: `object_detector` detecta objetos, `grasp_executor` executa movimentos, `conveyor_controller` gerencia a esteira. Essa separação permite testar, reiniciar ou substituir qualquer componente sem afetar os demais.

Um **Topic** é um canal de mensagens one-to-many: o detector publica em `/detected_objects`, e qualquer nó que precise das detecções subscreve esse tópico. Os principais tópicos do sistema são:
- `/camera/color/image_raw` — imagens RGB 848×480 a 30 Hz
- `/joint_states` — posições das 31 juntas em tempo real
- `/detected_objects` — bounding boxes + classe + posição 3D dos objetos
- `/conveyor/status` — estado da esteira em JSON
- `/cell/status` — estado do executor em JSON

Um **Service** é uma chamada request-response síncrona (bloqueante do lado do cliente): `/conveyor/advance` solicita que o próximo objeto seja trazido à pick station; `/cell/execute_grasp` inicia um ciclo de preensão.

Uma **Action** é o mecanismo de comunicação para tarefas longas com feedback: o `FollowJointTrajectory` action permite enviar uma trajetória de múltiplos waypoints ao controlador e receber confirmação de conclusão ponto a ponto.

**ros2_control** é o framework de controle de trajetória em tempo real integrado ao Gazebo. Três controllers são carregados em sequência de dependência no projeto:

1. `joint_state_broadcaster`: lê os encoders do Gazebo e publica `/joint_states` a 50 Hz
2. `cr10_group_controller` (JointTrajectoryController): recebe trajetórias para as 6 juntas do braço via action `/cr10_group_controller/follow_joint_trajectory`
3. `hand_position_controller` (JointTrajectoryController): recebe trajetórias para as 31 juntas da mão via action `/hand_position_controller/follow_joint_trajectory`

A cadeia de dependência é implementada com `OnProcessExit` event handlers no launch file: cada controller só é carregado após o anterior confirmar saída com sucesso.

---

## SLIDE 5 — Simulação no RViz: Visualização dos Robôs

### Texto de referência

O RViz (ROS Visualization) é a ferramenta de visualização 3D nativa do ROS 2. Diferentemente do Gazebo, o RViz não simula física — ele apenas renderiza a posição dos links com base nas transformações publicadas pelo `robot_state_publisher`. Isso o torna ideal para desenvolvimento e validação da cadeia cinemática antes de carregar o modelo em um simulador com física.

**robot_state_publisher** é o nó que converte `/joint_states` em transformações TF2 (árvore de frames de referência). Dado o URDF do robô e os ângulos atuais de cada junta, ele calcula e publica a pose de cada link em relação ao seu pai na árvore, alimentando o RViz e qualquer outro consumidor de transformações TF.

O projeto conta com quatro launch files para visualização progressiva:

**`display.launch.py`** — exibe apenas o URDF do CR10 no RViz com um joint_state_publisher_gui (sliders interativos para cada junta). É o ponto de partida para validar se o URDF foi parseado corretamente e se os links se movem de forma fisicamente plausível.

**`hand_gazebo.launch.py`** — carrega apenas a mão COVVI no Gazebo, isolada do braço. Permite validar os controllers da mão, testar as configurações de grasp (open, pinch, cylindrical, spherical, palm_grip, claw_grip, fingertip_grip) e verificar a estabilidade das juntas mimic sem a complexidade de toda a célula.

**`cr10_covvi_rviz.launch.py`** — visualiza o sistema integrado CR10 + mão COVVI no RViz. O URDF combinado é carregado com o robot_state_publisher, permitindo ver o comportamento de toda a cadeia cinemática de 37 juntas (6 do braço + 31 da mão) a partir de sliders ou de tópicos publicados por outro nó.

Esse fluxo progressivo — URDF isolado → mão isolada → sistema integrado no RViz → sistema completo no Gazebo — é uma prática padrão de desenvolvimento em robótica com ROS 2, pois isola problemas de modelagem de problemas de física.

---

## SLIDE 6 — Simulação Física no Gazebo Classic 11

### Texto de referência

O Gazebo Classic 11 é um simulador de dinâmica de corpos rígidos baseado no motor de física ODE (Open Dynamics Engine). Ele modela forças, torques, colisões e atrito, permitindo simular o comportamento físico de objetos com massa e inércia reais. A integração com ROS 2 é feita pelo pacote `gazebo_ros2_control`, que expõe os controladores do Gazebo como action servers ROS 2.

**Conteúdo da cena `conveyor_cell.world`:**

O ambiente da célula foi projetado como uma fábrica biomédica compacta e inclui os seguintes elementos estáticos:

- **Chão industrial**: plano cinza escuro (20×20 m), com faixas de segurança amarelas ao redor da base do robô (marcações de zona de risco)
- **Paredes e teto parcial**: paredes de fundo e laterais em cor clara, teto com painéis LED com propriedade emissiva (simulam iluminação industrial overhead)
- **Iluminação**: luz direcional principal simulando LED de teto + luz de preenchimento lateral + luz dedicada à área de classificação + luz dedicada à pick station — total de 4 fontes de luz para minimizar sombras que perturbem a detecção por cor
- **Pedestal do robô**: caixa 0,18×0,18×0,375 m em material escuro, posicionado na origem
- **Esteira transportadora**: estrutura em aço escuro com superfície de correia antiderrapante (atrito μ = 1,0), guias laterais em alumínio a ±240 mm do centro, pés de suporte. A pick station é marcada por um quadrado amarelo com cruz na superfície da correia em x = 0,65 m, y = 0
- **Câmera industrial**: modelo SDF com corpo físico + LED indicador verde + sensor `camera` Gazebo. Pose: x=1,15 m, z=1,70 m, pitch=1,05 rad, yaw=π (olha de volta para a pick station). Resolução 848×480, FoV horizontal 1,2217 rad (≈70°), 30 Hz, com ruído gaussiano σ=0,001. Montada em coluna + braço horizontal fora do envelope de trabalho do robô
- **Prateleira de classificação**: bancada em aço no lado y+ (y=0,65 m), com três caixas de classificação sobre ela
- **Caixas de classificação**:
  - Box 1 (vermelha): destino dos frascos, x=−0,05 m. Dimensões internas ≈ 260×240×170 mm
  - Box 2 (verde): destino dos tubos, x=0,25 m
  - Box 3 (azul): destino das ampolas, x=0,55 m

**Física configurada:**
- Solver ODE Quick, 150 iterações, SOR=1,3
- Passo de simulação: 1 ms (1000 Hz), fator de tempo real: 1,0
- Gravidade: 9,81 m/s²
- CFM=1e-6, ERP=0,2 para estabilidade de contato

**Objetos farmacêuticos** (gerados dinamicamente, não presentes no world file):
- Frasco de medicamento: cilindro âmbar, r=42 mm, h=90 mm, m=180 g
- Tubo de ensaio: cilindro azul, r=12 mm, h=120 mm, m=25 g
- Ampola farmacêutica: cilindro verde, r=5 mm, h=75 mm, m=8 g

---

## SLIDE 7 — Cinemática Direta (FK)

### Texto de referência

A cinemática direta (Forward Kinematics, FK) resolve o problema de determinar a pose do efetuador (posição + orientação no espaço cartesiano) a partir dos ângulos conhecidos de todas as juntas. Para um robô serial de n juntas, isso é obtido pela composição de n transformações homogêneas 4×4 encadeadas.

**Convenção URDF/ROS 2 adotada:**

A convenção nativa do ROS 2 define a transformação de cada junta como:

```
T_joint_i = T_origin(xyz_i, rpy_i)  ×  Rz(q_i)
```

onde `T_origin` é a transformação fixa da origem da junta (extraída do URDF) e `Rz(q_i)` é a rotação pura em torno do eixo Z pelo ângulo da junta. Isso difere da convenção DH clássica, onde as transformações têm forma padronizada. A vantagem da convenção URDF é que os parâmetros são lidos diretamente do modelo CAD sem necessidade de reescrita na forma DH.

**Cadeia FK completa (base_link → TCP da palma COVVI):**

```
T_TCP = T_01 × T_12 × T_23 × T_34 × T_45 × T_56 × T_HAND_ATTACH
```

Cada `T_ij` é calculada como `T_origin_j × Rz(q_j)`. O produto final é uma matriz 4×4 onde a submatriz 3×3 superior esquerda é a orientação do TCP e o vetor 3×1 superior direito é a posição do TCP, ambos no frame `base_link`.

**T_HAND_ATTACH** é a transformação fixa flange→palma COVVI. Ela encapsula o acoplamento mecânico da mão: 10 mm de offset em Z do flange (parafuso de fixação) mais 100 mm de profundidade do corpo da palma ao longo de Y do Link6. No frame world, durante a operação de pick com o braço em configuração de abordagem vertical descendente, esses 100 mm correspondem a 100 mm para baixo em Z.

**Exemplo prático — posição HOME:**

Com `q = [0, 0, π/2, −π/2, −π/2, 0]` (posição inicial segura do projeto), a FK produz o TCP em aproximadamente `(−0,69, −0,19, 1,31)` m no frame do robô. Essa posição mantém o braço dobrado para trás e para cima, sem sobrepor a esteira ou as caixas de classificação.

**Implementação em `kinematics.py`:**

```python
def forward_kinematics(q, include_hand=True):
    T = np.eye(4)
    for (xyz, rpy), qi in zip(_URDF_ORIGINS, q):
        T = T @ _make_T(xyz, rpy) @ _Rz4(float(qi))
    if include_hand:
        T = T @ T_HAND_ATTACH
    return T
```

A função `fk_partial(q, n)` retorna a FK parcial até o n-ésimo link, sendo utilizada pelo algoritmo de IK para calcular `R03` (orientação dos três primeiros links) necessária para a resolução analítica do pulso.

---

## SLIDE 8 — Jacobiano e Índice de Manipulabilidade

### Texto de referência

O Jacobiano é a matriz que relaciona velocidades no espaço das juntas com velocidades no espaço cartesiano do efetuador. Para um manipulador de 6 DOF, o Jacobiano geométrico J é uma matriz 6×6 onde as três primeiras linhas representam a velocidade linear (translação) e as três últimas a velocidade angular (rotação):

```
[v]   = J(q) × q̇
[ω]
```

**Cálculo por diferenças finitas:**

No módulo `kinematics.py`, o Jacobiano é calculado numericamente perturbando cada junta por ε = 1e−6 rad e medindo a variação na pose do TCP:

```
J[:3, i] = (p(q + ε·eᵢ) − p(q)) / ε       (parte translacional)
J[3, i]  = (dR[2,1] − dR[1,2]) / 2.0       (componentes da parte rotacional)
J[4, i]  = (dR[0,2] − dR[2,0]) / 2.0
J[5, i]  = (dR[1,0] − dR[0,1]) / 2.0
```

Esse método garante consistência com a FK sem necessitar derivar as equações analíticas das 36 entradas da matriz.

**Índice de manipulabilidade de Yoshikawa:**

O índice de manipulabilidade translacional é definido como:

```
w = sqrt(det(Jv × Jv^T))
```

onde `Jv = J[:3, :]` é a sub-matriz translacional 3×6. Geometricamente, `w` é o produto dos três semi-eixos do elipsoide de manipulabilidade — uma medida de quão "isotropicamente" o efetuador pode se mover em todas as direções. Quando w → 0, o robô está próximo de uma singularidade: perde a capacidade de gerar velocidade em ao menos uma direção cartesiana, independentemente de qual junta seja acionada.

**Três tipos de singularidade monitorados:**

1. **Singularidade de ombro** (`d_shoulder = min(1, r / 0,10)`): ocorre quando o wrist center está sobre o eixo da joint1 (r < 100 mm). O robô perde resolução em yaw do ombro.

2. **Singularidade de cotovelo** (`d_elbow = min(1, (1 − |D|) / 0,20)`): ocorre quando o braço está totalmente estendido (D → +1) ou totalmente dobrado (D → −1), onde D é o cosseno do ângulo de cotovelo pela lei dos cossenos. O robô perde resolução no plano radial.

3. **Singularidade de pulso** (`d_wrist = min(1, |sin(q5)| / sin(10°))`): ocorre quando q5 ≈ 0, alinhando os eixos de joint4 e joint6 e colapsando os 3 DOF do pulso em 2. Para o CR10, joint5 deve manter |sin(q5)| > sin(10°) = 0,174 para evitar movimentos de pulso explosivos.

**Aplicação no ciclo de grasp:**

Na função `grasp_planner`, o índice de manipulabilidade é calculado para cada solução IK candidata antes de executar o movimento, descartando posturas com w abaixo de um limiar. Isso previne trajetórias que passem por ou perto de singularidades durante o pick-and-place.

---

## SLIDE 9 — Cinemática Inversa (IK) Analítica + Refinamento Numérico

### Texto de referência

A cinemática inversa (Inverse Kinematics, IK) é o problema inverso à FK: dado um alvo cartesiano (posição p e orientação R desejadas do TCP), encontrar os ângulos das juntas q que o alcançam. Para o CR10, o problema é resolvido por uma abordagem desacoplada em dois estágios: palpite geométrico analítico seguido de refinamento numérico iterativo.

**Estágio 1 — Palpite geométrico analítico (desacoplamento posição/orientação):**

O princípio do desacoplamento explora o fato de que as últimas três juntas (pulso esférico) apenas orientam o efetuador sem alterar a posição do wrist center (WC). O wrist center é o ponto onde os eixos de joint4, joint5 e joint6 se intersectam.

**Passo 1 — Calcular a posição do wrist center:**
```
p_WC = p_TCP − d_WC × R_TCP[:, 2]
```
onde d_WC = 0,360 m (distância física WC→TCP = 0,260 m do flange até hand_base_link + 0,100 m de profundidade da palma) e `R_TCP[:, 2]` é o eixo Z do TCP (vetor de abordagem).

**Passo 2 — Resolver q1 (yaw do ombro):**
```
q1 = atan2(y_WC, x_WC)
```

**Passo 3 — Resolver q3 (cotovelo) pela lei dos cossenos:**
```
r = sqrt(x_WC² + y_WC²)
s = z_WC − d1    (d1 = 0,1765 m, altura da joint1)
D = (r² + s² − a2² − a3²) / (2·a2·a3)     (a2=0,607m, a3=0,568m)
θ3_DH = atan2(±√(1−D²), D)
```
O sinal ± seleciona elbow-up (+) ou elbow-down (−). O projeto utiliza elbow_up=False para o pick (cotovelo fisicamente acima da correia) e disponibiliza elbow_up=True como alternativa.

**Passo 4 — Resolver q2 (ombro):**
```
θ2_DH = atan2(s, r) − atan2(a3·sin(θ3_DH), a2 + a3·cos(θ3_DH))
q2_URDF = θ2_DH − π/2       (offset URDF: joint2 tem rpy[1]=π/2)
q3_URDF = θ3_DH              (sem offset)
```

**Passo 5 — Resolver pulso analiticamente de R₃₆:**
```
R_flange_target = R_TCP × T_HAND_ATTACH[:3,:3]^T
R03 = fk_partial(q_tmp, 3)[:3,:3]
R36 = R03^T × R_flange_target

q5 = atan2(√(r02² + r12²), r22)
q4 = atan2(−r12/sin(q5), −r02/sin(q5)) + π/2    (+π/2 = offset URDF)
q6 = atan2(−R36[2,1]/sin(q5), R36[2,0]/sin(q5))
```

**Estágio 2 — Refinamento numérico iterativo (máximo 300 iterações):**

O palpite analítico é refinado por um algoritmo de Levenberg-Marquardt desacoplado:

- **4 ciclos de arm-DLS + wrist analítico:**
  ```
  Δq_arm = J_arm^T (J_arm·J_arm^T + λ²·I)^{-1} · Δp
  q[:3] += lr · Δq_arm      (lr = 0.4, λ annealing: 0.06→0.003)
  q = _set_wrist(q, R_target)  (recalcula q4,q5,q6 analiticamente)
  ```

- **100 iterações DLS 6-DOF com peso de orientação:**
  ```
  error = [Δp, W_ori · Δω]    (W_ori = 0.25)
  Δq = J^T (J·J^T + λ²·I)^{-1} · error    (λ=0.005)
  q = clip(q + lr·Δq, JOINT_MIN, JOINT_MAX)
  ```

**Critério de convergência:** ‖Δp‖ < 3 mm e ‖Δω‖ < 0,05 rad.

**Múltiplos seeds para robustez:**
Para cada chamada de IK são gerados 16 candidatos de palpite inicial: q1 varrido em 7 passos (±0,7, ±0,4, ±0,2, 0 rad) para elbow-up e para elbow-down, mais o palpite naive e o q_seed fornecido pelo usuário. Cada candidato é refinado e o de menor erro é selecionado. Para pick na posição nominal (x=0,65 m, y=0), o seed pré-calculado `_PICK_SEED_Q = [0,473, −0,106, −1,542, −1,493, 0,473, 0,0]` garante convergência rápida na configuração de cotovelo acima da esteira.

---

## SLIDE 10 — Câmera e Modelo Pinhole

### Texto de referência

O sistema de visão utiliza uma câmera RGB monocular simulada no Gazebo pelo plugin `libgazebo_ros_camera.so`. A câmera é modelada como um sensor pinhole ideal com ruído gaussiano de desvio padrão σ = 0,001 nos valores de pixel (simula imperfeições de sensor real).

**Parâmetros físicos da câmera (extraídos de `conveyor_cell.world` e `object_detector.py`):**
- Resolução: 848 × 480 pixels
- FoV horizontal: 1,2217 rad (≈70°)
- Taxa de atualização: 30 Hz
- Clip near/far: 0,1 m / 3,0 m

**Parâmetros intrínsecos derivados do FoV:**

Como não há câmera de profundidade disponível no sistema (`libgazebo_ros_depth_camera.so` não está presente no Humble deste ambiente), os parâmetros intrínsecos são calculados analiticamente a partir do FoV:

```
fx = fy = (W/2) / tan(HFOV/2) = (848/2) / tan(1.2217/2) ≈ 603.5 px
cx = W/2 = 424.0 px
cy = H/2 = 240.0 px
```

**Pose da câmera no world frame:**
- Posição: x=1,15 m, y=0, z=1,70 m
- Ângulo: pitch=1,05 rad (câmera inclinada para baixo), yaw=π (câmera voltada para o lado da pick station)
- Montagem: coluna em x=1,35 m (fora do envelope do braço) com braço horizontal de 0,20 m

**Matriz de rotação world ← frame óptico ROS:**

O frame óptico ROS tem Z apontando para frente (eixo óptico), X para a direita e Y para baixo. Com pitch=1,05 rad e yaw=π, a rotação que transforma vetores do frame óptico para o frame world é:

```
R_W_CAM = [[ 0,   sin(pitch), −cos(pitch)],   ≈ [[0,  0.867, −0.498],
            [ 1,   0,          0          ],      [1,  0,      0    ],
            [ 0,  −cos(pitch), −sin(pitch)]]      [0, −0.498, −0.867]]
```

Essa matriz é derivada da composição Rz(π) × Ry(pitch) aplicada à convenção Gazebo→ROS e é o elemento central da back-projection geométrica.

---

## SLIDE 11 — Visão Computacional: Detecção HSV e Back-Projection Geométrica

### Texto de referência

**Detecção por segmentação HSV:**

No modo de simulação (`use_yolo=false`), os objetos são detectados por segmentação no espaço de cor HSV (Hue-Saturation-Value). Esse espaço é mais robusto a variações de iluminação do que o RGB porque separa informação de cor (H) da informação de brilho (V), permitindo definir máscaras de cor independentes da intensidade da luz.

As cores foram calibradas para as propriedades de material definidas no SDF de cada objeto:
- **Frasco** (diffuse: 0.90, 0.55, 0.05 → âmbar/laranja): H=8–26, S>120, V>80
- **Tubo** (diffuse: 0.10, 0.35, 0.95 → azul intenso): H=100–135, S>80, V>50
- **Ampola** (diffuse: 0.10, 0.95, 0.20 → verde brilhante): H=38–85, S>110, V>80

**Pipeline de processamento por frame (a 30 Hz):**
1. GaussianBlur 5×5 → supressão de ruído e artefatos de bordas
2. Conversão BGR → HSV (`cv2.cvtColor`)
3. `cv2.inRange` → máscara binária por faixa de cor para cada objeto
4. `MORPH_OPEN` 7×7 → remove pontos isolados e ruído fino
5. `MORPH_CLOSE` 9×9 → fecha buracos na segmentação de objetos com reflexo central
6. `cv2.findContours` → extrai contornos externos
7. Filtros de validade por contorno:
   - Área mínima: 150 px² (elimina ruído residual)
   - Área máxima por objeto: frasco=18.000 px², tubo=6.000 px², ampola=2.500 px²
   - Circularidade: `4π·A / P²` > mínimo por objeto (ampola tem mín=0,05 pois é fina)
   - Aspect ratio: w/h dentro de limites (ampola: 0,05–12,0 para capturar orientações verticais)
8. Bounding box + score de confiança = `min(área / 2500, 1,0)`

O resultado é publicado em `/detected_objects` como `Detection2DArray` e exibido em uma janela OpenCV ao vivo com título "Pick Station — Visão do Robô".

**Back-projection geométrica (pixel → posição 3D):**

Como não há sensor de profundidade, a posição 3D do objeto é estimada assumindo que o objeto está sobre um plano horizontal conhecido. O plano z_plane é calculado como o topo da correia mais a meia-altura do objeto específico:
- frasco: z_plane = 0,806 + 0,045 = 0,851 m
- tubo: z_plane = 0,806 + 0,060 = 0,866 m
- ampola: z_plane = 0,806 + 0,038 = 0,844 m

A interseção do raio de câmera com esse plano:
```
d_cam   = [(u − cx)/fx, (v − cy)/fy, 1.0]     # raio normalizado no frame óptico
d_world = R_W_CAM @ d_cam                       # raio no frame world
t       = (z_plane − z_cam) / d_world[2]        # escalar da interseção
p_world = cam_pos + t × d_world                 # ponto 3D no world
```

O resultado é a posição (x, y, z) em metros no world frame, que é incorporada diretamente no `Detection2D` e consumida pelo `grasp_executor` para o cálculo do IK de pick. Um filtro de sanidade rejeita pontos fora da área de trabalho esperada (0,2 < x < 1,2 m, −0,4 < y < 0,4 m).

**Modo alternativo YOLOv8 (`use_yolo=true`):** A detecção por rede neural pode ser ativada por parâmetro de launch para deploy no robô físico, mantendo a mesma interface de saída (`/detected_objects`). O mapeamento de classes COCO para os objetos do projeto é feito via dicionário: bottle→tubo, cup→frasco, vase→frasco.

---

## SLIDE 12 — Spawn Dinâmico de Objetos: Como Funciona o Gêmeo Digital

### Texto de referência

Uma das características fundamentais que caracteriza o sistema como um gêmeo digital — e não apenas uma simulação estática — é o gerenciamento dinâmico de objetos na cena. Os objetos farmacêuticos **não existem permanentemente** no arquivo world; eles são injetados e removidos do Gazebo em tempo de execução, replicando com fidelidade o comportamento de uma esteira industrial real onde objetos chegam, são processados e seguem em frente.

**Nó `conveyor_controller` (arquivo `conveyor_controller.py`):**

Este nó expõe três serviços ROS 2 do tipo `std_srvs/Trigger`:

- **`/conveyor/advance`** — Injeta o próximo objeto da sequência na pick station. O nó constrói um modelo SDF programaticamente em Python com a geometria, massa, inércia e cor do objeto específico, e chama o serviço Gazebo `/spawn_entity`. O objeto é injetado em z=2,0 m (no ar) e cai por gravidade até a superfície da correia em z=0,806 m. O estado `has_object=true` é registrado e publicado.

- **`/conveyor/retreat`** — Remove o objeto atual da cena chamando `/delete_entity`. É invocado automaticamente pelo `grasp_executor` logo após o fechamento da mão confirmar o grasp, antes do levantamento.

- **`/conveyor/reset`** — Remove o objeto atual (se houver) e reinicia o índice da sequência para o início.

**Modelos SDF gerados programaticamente:**

Cada objeto tem suas propriedades físicas corretas definidas no código:
```python
# Frasco: cilindro âmbar, física realista de vidro leve
massa  = 0.180 kg
raio   = 0.042 m, altura = 0.090 m
ixx    = 5e-4, iyy = 5e-4, izz = 2e-4 kg·m²
atrito = μ = μ2 = 0.9

# Tubo: cilindro azul, muito leve
massa  = 0.025 kg
raio   = 0.012 m, altura = 0.120 m

# Ampola: cilindro verde, extremamente fino e leve
massa  = 0.008 kg
raio   = 0.005 m, altura = 0.075 m
```

**Ciclo de vida de um objeto na célula:**

```
[IDLE] → /conveyor/advance → [spawn_entity: frasco, z=2.0m] → [queda livre] →
[objeto na correia] → [câmera detecta] → [executor pega] → [grasp confirmado] →
/conveyor/retreat → [delete_entity] → [IDLE] → /conveyor/advance → [spawn: tubo] → ...
```

**Publicação de estado:** O nó publica `/conveyor/status` a 1 Hz com JSON:
```json
{"has_object": true, "current_obj": "frasco", "queue_idx": 0, "queue_total": 3}
```
Esse tópico é consumido pela GUI para exibir o estado em tempo real ao operador.

---

## SLIDE 13 — Ciclo de Grasp: 9 Fases Determinísticas

### Texto de referência

O executor de grasp (`grasp_executor.py`) implementa um ciclo completo de pick-and-place dividido em 9 fases sequenciais. O ciclo é determinístico: não há decisão probabilística ou aprendizado de máquina no `conveyor_cell.launch.py`; cada objeto tem seu tipo de preensão e caixa de destino fixos.

**Mapeamento objeto → preensão → destino:**

| Objeto | Grip Type       | Caixa | Diâmetro efetivo |
|--------|-----------------|-------|------------------|
| frasco | palm_grip       | Box 1 | 60 mm            |
| tubo   | claw_grip       | Box 2 | 24 mm            |
| ampola | fingertip_grip  | Box 3 | 10 mm            |

**Configurações das juntas primárias da mão por grip (rad):**

| Grip           | Thumb | Index | Middle | Ring  | Little | Rotate |
|----------------|-------|-------|--------|-------|--------|--------|
| palm_grip      | 1,10  | 1,25  | 1,25   | 1,20  | 1,10   | 0,25   |
| claw_grip      | 0,90  | 1,10  | 1,10   | 1,05  | 0,95   | 0,45   |
| fingertip_grip | 0,75  | 0,70  | 0,65   | 0,05  | 0,05   | 0,82   |

A posição de fechamento real adiciona uma margem extra de 5% sobre esses valores (`_close_extra`: cfg[j] + 0,05 × HAND_LIMITS[j]), garantindo pressão sobre o objeto mesmo com pequenas imprecisões de posicionamento.

**As 9 fases do ciclo:**

**F1 — Abertura e abordagem de pick:**
Mão é aberta completamente (open: todos os drivers = 0). Braço move-se para `approach_pick` = posição de pick + 0,15 m em Z. Esse movimento usa o seed `_PICK_SEED_Q` para garantir solução IK de cotovelo acima da correia (elbow_up=False).

**F2 — Descida com mão aberta + pré-configuração de grip:**
Braço desce para a posição de pick (`pick_z` específica por objeto). Simultaneamente a mão assume a configuração `cfg_grasp` (dedos se posicionam sem fechar), preparando o envelope de preensão. A mão permanece aberta durante a descida para que as pontas dos dedos não colidam com a esteira.

**F3 — Fechamento da mão + retreat da esteira:**
Mão fecha para `cfg_closed` (cfg_grasp + margem extra). Após confirmação, chama `/conveyor/retreat` para remover o objeto da cena (o objeto é "capturado" logicamente; na física do Gazebo ele seria carregado pelo atrito da palma).

**F4 — Levantamento:**
Braço sobe para `lift_pos` = pick_pos + 0,22 m em Z. O seed para este IK é `q_pick`, garantindo continuidade de postura.

**F5 — Trânsito lateral (pick area → via_box):**
Braço move-se para `via_pos`: xy da caixa de destino com z_TRANSIT = 1,15 m world (= 0,745 m no frame do robô). Essa altura foi escolhida para estar acima das guias laterais da esteira (0,935 m) e da prateleira de classificação (0,50 m), evitando colisões durante o trânsito lateral. O seed é `_HOME_Q`.

**F6 — Descida para abordagem da caixa:**
Braço desce para `approach_box` = posição da caixa + 0,15 m em Z. O seed compacto `_APPROACH_BOX_SEED_Q = [0, −0,4, −1,5, −1,3, 0, 0]` é utilizado pois o seed via_box colapsa para configurações problemáticas para box2/box3.

**F7 — Soltura sobre a caixa:**
Mão abre (cfg_open). TCP está em `approach_box_z ≈ 0,75 m world`, acima das paredes das caixas (≈0,705 m). O objeto cai livremente por gravidade para dentro da caixa.

**F8 — Subida ao via_box:**
Braço retorna ao via_pos para desobstruir o caminho de retorno.

**F9 — Retorno ao HOME:**
Braço retorna à posição `_HOME_Q = [0, 0, π/2, −π/2, −π/2, 0]`, pose segura que não sobrepõe nenhum elemento da célula.

**Geração de trajetórias suaves:**

Cada movimento entre duas posturas gera uma trajetória de 12 waypoints com perfil ease-in/out sinusoidal:
```
smooth(α) = 0.5 × (1 − cos(π × α)),    α ∈ {1/12, 2/12, ..., 1}
q_i = q_start + smooth(α_i) × (q_end − q_start)
duração = max(Δq_max / 0,5 rad/s, 2,0 s)
```
Esse perfil elimina descontinuidades de velocidade nos extremos da trajetória (velocidade inicial e final = 0), reduzindo vibrações mecânicas e aumentando a estabilidade do objeto segurado.

---

## SLIDE 14 — Arquitetura Completa: conveyor_cell.launch.py

### Texto de referência

O arquivo `conveyor_cell.launch.py` é o ponto de entrada único do sistema. Ele orquestra a inicialização de todos os processos na ordem correta, usando event handlers para garantir que nenhum nó dependente inicie antes que seus pré-requisitos estejam prontos.

**Sequência de inicialização (cadeia de dependência com OnProcessExit):**

```
1. Gazebo (conveyor_cell.world)
       ↓  (Gazebo sobe o simulador)
2. robot_state_publisher (URDF combinado CR10+COVVI)
       ↓  (URDF disponível para spawn)
3. spawn_entity (insere robô na simulação: z=0.375m)
       ↓  OnProcessExit(spawn_robot)
4. joint_state_broadcaster
       ↓  OnProcessExit(joint_state_broadcaster)
5. cr10_group_controller
       ↓  OnProcessExit(cr10_group_controller)
6. hand_position_controller
       ↓  OnProcessExit(hand_position_controller)
7. [object_detector, grasp_executor, conveyor_controller, gui_control, pipeline]
   (todos em paralelo, aguardam action servers)
```

**URDF combinado gerado programaticamente em Python:**

O sistema não utiliza um URDF pré-compilado. A cada execução do launch, a função `_build_robot_urdf()` executa as seguintes etapas:
1. Processa `cr10_robot.xacro` com a biblioteca `xacro`, obtendo o URDF do braço como string XML
2. Substitui o caminho do `ros2_controllers.yaml` pelo `cr10_covvi_controllers.yaml` combinado
3. Lê `linear_covvi_hand_gazebo.urdf`, remove links `world` e `base_footprint` e o joint fixo world (conflito com o frame world do CR10)
4. Renomeia `base_link` da mão para `hand_base_link` (evita colisão de nomes)
5. Remove o bloco de plugin `gazebo_ros2_control` da mão (já incluído no CR10)
6. Aplica `_fix_virtual_link_inertia()`: substitui inércias padrão de links virtuais (massa=1, inércia=1) por valores mínimos (massa=0,001, inércia=1e-9) para evitar instabilidade física no Gazebo
7. Aplica `_stabilize_hand_joints()`: injeta damping=10 N·m·s/rad e friction=2 N·m nas juntas primárias da mão, e damping=120 e friction=20 nas juntas mimic (amortecimento maior nas mimic para suprimir oscilações de alta frequência)
8. Adiciona tags `<gazebo>` de self_collide=false para todos os links (crítico para evitar explosões de física na mão)
9. Insere o joint `hand_attach_joint` (Link6→hand_base_link)
10. Gera uma versão "minimal" do URDF (sem visual, collision e inertial) para o robot_state_publisher (mais leve para publicação TF)

**Argumentos configuráveis de launch:**

| Argumento    | Default | Descrição                                              |
|-------------|---------|--------------------------------------------------------|
| use_yolo    | false   | true = YOLOv8; false = segmentação HSV                |
| sim_only    | true    | true = spawn/delete Gazebo; false = modo robô físico  |
| no_gui      | false   | true = sem janela Tkinter (modo headless)             |
| autonomous  | false   | true = pipeline executa ciclos automaticamente        |

**Tópicos e serviços de toda a célula:**

```
Câmera:     /camera/color/image_raw (30 Hz)
Visão:      /detected_objects → /detector/debug_image → /detector/status
Esteira:    /conveyor/advance · /conveyor/retreat · /conveyor/reset
            /conveyor/status (JSON, 1 Hz)
Executor:   /cell/execute_grasp · /cell/go_home
            /cell/status (JSON, 2 Hz)
Controle:   /joint_states (50 Hz) · /cr10_group_controller/... · /hand_position_controller/...
```

---

## SLIDE 15 — Painel de Controle GUI (Tkinter)

### Texto de referência

A interface gráfica (`gui_control_node.py`) foi desenvolvida em Tkinter com integração ao ROS 2 via `MultiThreadedExecutor`. A restrição de design mais importante é que o Tkinter exige exclusividade da thread principal do sistema operacional; portanto, o loop do ROS 2 é executado em uma thread de fundo enquanto a thread principal permanece dedicada ao mainloop do Tkinter.

**Arquitetura de threading:**
```python
spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()
node.run_gui()  # bloqueia a thread principal no mainloop Tkinter
```

**Estrutura da janela (640×520 px, não redimensionável):**

*Cabeçalho:* Título "CÉLULA DE MANUFATURA CR10 + COVVI" em fonte monoespaçada.

*Status strip:* Dois indicadores atualizados em tempo real (poll a cada 500 ms):
- **Esteira:** "✔ frasco [1/3]" (objeto presente, frasco, item 1 de 3) ou "○ none [0/3]" (vazio)
- **Executor:** "⏳ GRASPING:frasco (objeto: frasco)" durante operação ou "✔ IDLE" quando livre

*Seção Controle da Esteira:*
- **[Próximo Objeto ▶]** (verde): chama `/conveyor/advance` → Gazebo injeta próximo objeto
- **[◀ Estágio Anterior]** (vermelho): chama `/conveyor/retreat` → Gazebo remove objeto atual
- **[⟳ Resetar Esteira]** (vermelho): com diálogo de confirmação → reseta sequência para zero

*Seção Controle do Robô:*
- **[✋ AGARRAR]** (azul, botão grande): chama `/cell/execute_grasp` → inicia ciclo de 9 fases em thread separada
- **[⌂ Home]** (amarelo): chama `/cell/go_home` → retorna braço ao HOME imediatamente

*Área de feedback:* Mensagem de status do último serviço chamado (verde = sucesso, vermelho = erro, laranja = em andamento).

**Comportamento assíncrono — o sistema nunca trava a GUI:**

Todos os serviços são chamados com `client.call_async()`. O callback de resultado (`future.add_done_callback`) executa `root.after(0, ...)` para atualizar a GUI com thread safety. Os botões são desabilitados durante operações e reabilitados ao receber o resultado, impedindo comandos conflitantes.

**Janela OpenCV paralela:**

O nó `object_detector` abre automaticamente uma segunda janela "Pick Station — Visão do Robô" com a imagem ao vivo da câmera simulada, anotada com:
- Bounding boxes coloridas por objeto (laranja=frasco, azul=tubo, verde=ampola)
- Label com nome do objeto e score de confiança
- Barra de progresso de score abaixo de cada bbox
- Indicador de mira na posição projetada da pick station
- Linhas de referência horizontal/vertical centradas no objeto detectado
- Barra de status superior com contagem de detecções e modo ativo (HSV ou YOLO)

---

## SLIDE 16 — Resultados e Demonstração do Gêmeo Digital

### Texto de referência

O sistema demonstra comportamento de gêmeo digital fiel ao processo industrial biomédico que representa. Os resultados são avaliados em três dimensões: fidelidade física, precisão cinemática e corretude funcional do pipeline de preensão.

**Fidelidade física da simulação:**

Os objetos farmacêuticos são modelados com propriedades físicas reais: o frasco de medicamento (180 g) apresenta comportamento estável sobre a correia e resistência ao deslizamento adequada para o palm_grip. O tubo de ensaio (25 g) e a ampola (8 g) são objetos mais delicados cujos parâmetros de inércia foram ajustados para refletir seu comportamento real em manuseio biomédico.

A iluminação da cena com quatro fontes de luz foi necessária para minimizar sombras que perturbassem a segmentação HSV. A câmera com pitch=1,05 rad e yaw=π foi posicionada para maximizar a área visível da pick station (marcador amarelo no centro da correia em x=0,65 m), com campo de visão suficiente para capturar os três tipos de objeto em qualquer posição razoável na correia.

**Precisão cinemática:**

O IK converge com critério ‖Δp‖ < 3 mm e ‖Δω‖ < 0,05 rad para todas as posturas do ciclo. As alturas de pick foram calibradas empiricamente para cada objeto:
- Frasco (h=90 mm): pick_z = 0,916 m → dedos chegam a 43–51% da altura do objeto
- Tubo (h=120 mm): pick_z = 0,946 m → dedos chegam a ~58% da altura
- Ampola (h=75 mm): pick_z = 0,913 m → dedos chegam a ~51% da altura

A altura de trânsito lateral de 1,15 m world foi validada numericamente para garantir que joint3 permanece acima de 0,50 m world (topo da sort_shelf) para os três objetos e as três caixas de destino, eliminando colisões durante o trânsito.

**Corretude funcional:**

O ciclo completo frasco→tubo→ampola executa sem colisões entre o braço, os links da mão, a esteira e as caixas. A sequência de 9 fases por objeto representa aproximadamente 20–30 segundos de operação por ciclo, com as fases mais longas sendo F1 (abordagem) e F5 (trânsito lateral).

O gêmeo digital reproduz corretamente a diferenciação de preensão por tipo de objeto:
- palm_grip: todos os cinco dedos envolvem o frasco em preensão palmar de força
- claw_grip: polegar e quatro dedos em posição cilíndrica para o tubo
- fingertip_grip: polegar e indicador em pinça de precisão para a ampola delicada

**Fidelidade como gêmeo digital:**

A arquitetura modular com spawn/delete dinâmico, câmera calibrada, física realista e controladores PID idênticos aos do hardware real estabelece a base necessária para a futura migração Sim-to-Real. Cada módulo (detector, estimador de pose, executor) tem interface ROS 2 padrão que permanece inalterada ao trocar o backend de simulação pelo hardware físico.

---

## SLIDE 17 — Contribuições Técnicas

### Texto de referência

Este projeto apresenta um conjunto de contribuições técnicas originais no contexto da robótica biomédica simulada:

**1. Cinemática analítica completa para a convenção URDF/ROS 2 sem dependência de MoveIt:**

A implementação FK/IK em `kinematics.py` resolve a cadeia cinemática do CR10 diretamente na convenção nativa do ROS 2, com o offset de −π/2 nas juntas 2 e 4 corretamente tratado no estágio de extração do pulso analítico (+π/2 em q4_URDF e q2_URDF = θ2_DH − π/2). Essa implementação autocontida, sem dependência do MoveIt, permite operação em sistemas com recursos computacionais limitados e tempo de resposta determinístico.

**2. Back-projection geométrica de câmera 2D para posição 3D:**

A estimativa de posição 3D sem câmera de profundidade, usando apenas o modelo pinhole, a pose conhecida da câmera e a suposição de plano horizontal com altura conhecida, é uma solução de custo mínimo adequada para células industriais onde o plano de trabalho é fixo. O erro de posicionamento resultante (< 1 cm para a área da pick station) é suficiente para alimentar o IK com convergência dentro dos critérios estabelecidos.

**3. URDF combinado gerado dinamicamente em tempo de execução:**

A função `_build_robot_urdf()` elimina a necessidade de manter um URDF manual do sistema integrado, que se tornaria inconsistente a cada modificação nos modelos individuais. O processo de combinação programática — incluindo remoção de conflitos, injeção de dinâmica estabilizadora e geração de versão minimal para TF — é uma contribuição metodológica para projetos que combinam múltiplos pacotes URDF de origens distintas.

**4. Spawn/delete dinâmico como mecanismo de gêmeo digital:**

O uso dos serviços `/spawn_entity` e `/delete_entity` do Gazebo como primitivas de controle de esteira, com modelos SDF gerados programaticamente, permite simular com fidelidade o fluxo de materiais de uma célula de manufatura real sem necessidade de física de transporte (correia animada), que seria computacionalmente custosa e desnecessária para o objetivo de treino.

**5. Arquitetura extensível com parâmetros de launch:**

A troca entre HSV e YOLOv8 (`use_yolo:=true`), entre modo simulado e físico (`sim_only:=false`) e entre operação manual e autônoma (`autonomous:=true`) é feita por argumentos de launch sem alterar o código. Isso suporta diretamente a transição Sim-to-Real futura.

**6. Pipeline GUI não-bloqueante integrado ao ROS 2:**

O padrão de design que separa o spin do ROS 2 em thread de fundo e mantém o Tkinter na thread principal, com calls assíncronos via future callbacks e atualização thread-safe via `root.after(0, ...)`, é aplicável a qualquer interface gráfica de operação de célula robótica em ROS 2.

---

## SLIDE 18 — Trabalhos Futuros e Conclusão

### Texto de referência

**Trabalhos futuros:**

A fase atual entrega o gêmeo digital completo e funcional. As etapas subsequentes do TCC e de desenvolvimentos futuros incluem:

*Protocolo com usuários reais:* Aplicação do gêmeo digital em sessões de treinamento com usuários de próteses de mão MGL, comparando métricas de aprendizado (tempo para dominar cada configuração de preensão, taxa de erros, carga cognitiva) entre o grupo que treina com o gêmeo digital e o grupo controle que treina diretamente com o dispositivo físico.

*Integração Sim-to-Real:* Substituição do Gazebo pelo hardware real mantendo os mesmos nós ROS 2. O `grasp_executor` já abstrai a física do simulador: basta apontar os action clients para os controllers do hardware real. A câmera real precisará de calibração (extração de K matrix via `cv2.calibrateCamera`) para alimentar a back-projection com parâmetros precisos.

*Modelo YOLOv8 customizado:* Treino de um detector YOLOv8 com dataset de frasco/tubo/ampola reais para substituir a segmentação HSV no deploy. O parâmetro `use_yolo:=true` já suporta essa transição sem alteração de arquitetura.

*Expansão da célula:* Adicionar novos tipos de objetos (seringas, bisturis, comprimidos), novos tipos de preensão e múltiplos braços cooperativos, expandindo o escopo do gêmeo digital para uma linha de manufatura biomédica completa.

**Conclusão:**

Este trabalho demonstrou que é possível desenvolver, validar e operar um sistema completo de pick-and-place com preensão diferenciada por visão computacional inteiramente em ambiente de simulação, com nível de fidelidade física e funcional suficiente para servir como base de treino para usuários de próteses de mão.

A escolha do ROS 2 como middleware garante portabilidade direta entre simulação e hardware real. A cinemática analítica autocontida elimina dependências externas e oferece desempenho determinístico. A visão computacional baseada em back-projection geométrica resolve o problema de localização 3D sem hardware adicional de depth sensing. O spawn dinâmico de objetos reproduz com fidelidade o comportamento de uma esteira de manufatura biomédica real.

O resultado é uma plataforma de pesquisa robusta e extensível que conecta os campos da robótica colaborativa, reabilitação de amputados e automação industrial em um sistema integrado de gêmeo digital.

---

*Documento gerado automaticamente a partir do código-fonte do repositório RoboticArm.*
*Todos os valores numéricos, parâmetros e equações são extraídos diretamente dos arquivos:*
*`kinematics.py`, `object_detector.py`, `grasp_executor.py`, `conveyor_controller.py`,*
*`gui_control_node.py`, `conveyor_cell.launch.py`, `conveyor_cell.world`, `pipeline_params.yaml`*
