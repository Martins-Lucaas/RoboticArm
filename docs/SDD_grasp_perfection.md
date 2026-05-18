# SDD — Perfeição de Grasp & Entrega (CR10 + COVVI)

> **Spec-Driven Development** para o ciclo completo de pick & place da
> célula de manufatura farmacêutica. Este documento é a única fonte da
> verdade para poses, fases, configurações de dedos e critérios de
> aceitação. Todo código (manual_control_node.py, grasp_executor.py,
> kinematics.py) deve ser derivado deste documento.

| Campo            | Valor                                                 |
| ---------------- | ----------------------------------------------------- |
| Versão           | 1.2.0                                                 |
| Data             | 2026-05-16                                            |
| Autor            | Lucas Martins                                         |
| Status           | Em redação — bloqueia merge de mudanças de pose       |
| Pacotes afetados | grasp_ml_pack (poses, collision, manual_control, grasp_executor) |
| Referências      | images/claw.png, images/palmGrip.png, images/fingertip.png |

### Changelog
- **v1.3.0 (2026-05-16)** — T21-T30 totalmente implementados:
  descent_extra na GUI, Hover/Close, TF tcp_target em RViz, scripts
  `tune_rotate.py`/`tune_descent.py`/`test_9cycles.py`. Sign de
  descent_extra corrigido: positivo = palma desce mais.
- **v1.2.0 (2026-05-16)** — Adicionada §4.0 com geometria física
  (objetos + braço + mão + dedos). HAND_GRIPS restaurados aos valores
  originais de `kinematics.HAND_CONFIGS`. PICK z calibrado por palpação
  geométrica (frasco/ampola 0.87, tubo 0.866). Tasks T21–T30 para
  refinamento metódico.
- **v1.1.0 (2026-05-16)** — IK collision-aware via `collision.py`. R_tcp
  explícito (palm-down / lateral-claw). Cascata de seeds appr→pre/pick.
- **v1.0.0 (2026-05-16)** — Documento inicial.

---

## 0. Glossário

- **TCP** — Tool Center Point, ponto de convergência das pontas dos dedos
  COVVI quando a mão está fechada (frame `T_HAND_ATTACH`, 75 mm na
  direção `−Y_link6`).
- **pre-approach** — Pose 15–20 cm antes do contato, sem risco de varrer
  o objeto, com a mão totalmente aberta.
- **approach** — Pose 4–5 cm antes do contato, com mão em pré-shape.
- **grasp** — Pose de contato físico real (TCP no centro de captura).
- **pré-shape (pre-grasp)** — Configuração intermediária dos dedos antes
  do fechamento final, evitando que dedos varrem o objeto durante a
  descida.
- **lift** — Pose 15 cm acima de grasp com mão fechada.

---

## 1. Visão

Hoje a célula executa um ciclo de pick com a mesma estratégia
(approach lateral genérico) para os três objetos, o que produz preensões
ruins e poses de approach visualmente incorretas (a mão chega de lado
no frasco e na ampola, quando deveria descer top-down). O grasp da
ampola usa todos os dedos quando deveria ser uma pinça polegar+indicador.
A entrega usa uma única pose top-down agnóstica, sem fase de descida
suave.

A v1.0 deste SDD redefine **poses canônicas por objeto**, **pré-shapes
por preensão**, **ciclo de pick em 6 fases** e **entrega objeto-específica**,
expondo todos esses parâmetros e fases na GUI `manual_control`.

---

## 2. Stakeholders & Casos de uso

| Stakeholder            | Caso de uso                                            |
| ---------------------- | ------------------------------------------------------ |
| Operador / pesquisador | Acionar pick por objeto em modo step-by-step para auditoria |
| Operador / pesquisador | Ver a fase atual do ciclo em LED no GUI                |
| Engenheiro de teste    | Editar offset de descida, tempo por fase, threshold de toque sem recompilar |
| Engenheiro de ML       | Reutilizar exatamente as mesmas poses no grasp_executor autônomo |

---

## 3. Requisitos (R1–R12)

### Requisitos funcionais

- **R1** — Para **frasco**, o approach é **top-down** (vetor `(0,0,-1)`),
  a palma fica horizontal sobre o topo do frasco, e o grasp se dá pela
  envoltura palmar dos 4 dedos longos sobre o ombro do frasco.
- **R2** — Para **ampola**, o approach é **top-down** (vetor `(0,0,-1)`),
  e o grasp é uma **pinça precisa polegar+indicador** sobre a ponta
  superior da ampola. Os dedos médio/anelar/mínimo permanecem recolhidos
  mas sem tocar a ampola.
- **R3** — Para **tubo**, o approach é **lateral, frontal ao tubo,
  com a palma virada para o lado** (igual à imagem `images/claw.png`):
  a mão chega pela frente, claw-style, e abraça o tubo pelo lado.
- **R4** — Cada pick executa em **6 fases discretas e auditáveis**:
  F1 pre-approach (articular), F2 approach (Cartesiano curto), F3 pré-shape
  da mão, F4 descida/avanço final ao TCP de grasp, F5 fechamento incremental
  com detecção de toque, F6 lift.
- **R5** — A GUI `manual_control` deve permitir disparar (a) pick completo
  por objeto, (b) cada fase individualmente, (c) "step-by-step" com pausa
  entre fases.
- **R6** — A GUI deve mostrar o estado da fase atual (LED + texto).
- **R7** — A GUI deve expor parâmetros editáveis: offset de descida,
  duração por fase, threshold de toque.
- **R8** — A entrega ocorre em 4 sub-fases: F1' pre-deliver (15 cm acima
  da caixa), F2' descida (10 cm), F3' release (abertura da mão), F4'
  retract (volta a pre-deliver). Para o **tubo**, a orientação lateral
  é mantida até F3'.
- **R9** — Aproach por objeto deve ser também acionável **isoladamente**
  (sem fechar a mão), na aba **Mão COVVI**, para testes de geometria.

### Requisitos não-funcionais

- **R10** — As poses canônicas vivem em **um único módulo** (`poses.py`),
  reutilizado por `manual_control_node.py` e `grasp_executor.py`.
- **R11** — IK deve ser determinístico (mesmo branch entre approach e
  grasp do mesmo objeto). Fallbacks em graus são versionados.
- **R12** — Nenhum dedo pode penetrar o AABB do objeto durante o
  approach/descida (apenas no grasp, e com profundidade ≤ raio do objeto).

---

## 4.0 Geometria física (fonte: URDF + SDF)

### 4.0.1 Base e braço CR10

| Parâmetro                       | Valor       | Origem                              |
| ------------------------------- | ----------- | ----------------------------------- |
| Altura do base_link no mundo    | 0.405 m     | spawn z=0.375 + URDF z=0.030        |
| L_link2 (upper arm)             | 0.607 m     | URDF joint3.origin.x = -0.607       |
| L_link3 (forearm)               | 0.568 m     | URDF joint4.origin.x = -0.568       |
| Reach total (Link6 origin)      | ≈1.30 m     | soma das normas das origens         |
| _D_WC_TCP (wrist→TCP)           | 0.375 m     | kinematics.py                       |
| Limites q2 (URDF)               | -260..+80°  | kinematics.JOINT_MIN/MAX            |
| Limites q3 (URDF)               | -135..+135° | idem                                |

### 4.0.2 Mão COVVI

| Parâmetro                       | Valor       | Origem                              |
| ------------------------------- | ----------- | ----------------------------------- |
| T_HAND_ATTACH translate (z)     | 0.115 m     | kinematics.T_HAND_ATTACH            |
| MCP Index (hand_base, m)        | (+0.023, +0.091, -0.015) | URDF              |
| MCP Middle (hand_base, m)       | (+0.003, +0.094, -0.016) | URDF              |
| MCP Ring (hand_base, m)         | (-0.016, +0.094, -0.011) | URDF              |
| MCP Little (hand_base, m)       | (-0.033, +0.084, -0.004) | URDF              |
| Thumb chassis pivot             | (+0.024, +0.023, +0.013) | kinematics        |
| Comprimento phalanx proximal    | 0.045 m (`_L_PROX`)        | kinematics      |
| Comprimento phalanx distal      | 0.030 m (`_L_DIST`)        | kinematics      |
| Comprimento dedo (extendido)    | ≈0.075 m                   | _L_PROX+_L_DIST |
| Fingertip-curl reach (closed)   | ≈0.024 m                   | calibrado FK    |
| TCP-MCP (em finger direction)   | ≈0.115 m                   | T_HAND_ATTACH   |

### 4.0.3 Objetos picáveis (frame mundo)

| Objeto  | Raio (mm) | Altura (mm) | Centro Z (m) | Topo Z (m) | Spawn XY |
| ------- | --------- | ----------- | ------------ | ---------- | -------- |
| frasco  | 42        | 90          | 0.851        | 0.896      | (0.75, 0) |
| tubo    | 12        | 120         | 0.866        | 0.926      | (0.75, 0) |
| ampola  | 5         | 75          | 0.844        | 0.881      | (0.75, 0) |

Belt top em world z=0.806 (objetos apoiados sobre a esteira).

### 4.0.4 Implicação para o TCP de pick

Para palm-down (TCP_z = +X world), a palma (hand_base_link origin)
fica ~24 mm "atrás" do TCP em −TCP_z. Portanto, para a palma encostar
no **topo** do objeto, TCP_z_world ≈ obj_top − 0.024 m.

Mas **Link6 STL** estende ~42 mm em −local_z, e com a orientação
palm-down ele "pendura" para baixo. Distância mínima do wrist ao belt
para evitar penetração de Link6: ≈64 mm. Logo TCP_z_min = belt_top +
0.064 ≈ 0.87 m (validado numericamente).

Por isso usamos `frasco/ampola pick z = 0.87` (mínimo seguro). Para o
**tubo**, a palma encosta lateralmente (z = obj_center), sem risco
para Link6 — usamos `pick z = 0.866`.

---

## 4. Especificação por objeto

### 4.1 Frasco (frasco de medicamento — r=42 mm, h=90 mm)

- **Tipo de preensão:** Palm Grip — envoltura palmar (TODOS os dedos).
- **R_tcp:** `_Rtcp_palm_down(finger_dir=(+1,0,0))` — palma normal −Z,
  dedos extendem em +X.
- **Pose `pre_approach`:** TCP em `(0.75, 0.00, 1.00)` world.
- **Pose `approach`:** TCP em `(0.75, 0.00, 0.92)` world.
- **Pose `grasp`:** TCP em `(0.75, 0.00, 0.87)` world (palma encosta
  no topo do frasco; convergência 26 mm abaixo do topo). Mínimo Z
  seguro — abaixo Link6 toca belt_surface.
- **Pré-shape Palm:** `Thumb=70, Index=80, Middle=80, Ring=75,
  Little=70, Rotate=25` (sliders 0–200) — ≈50 % do grip final.
- **Grasp final Palm:** `Thumb=138, Index=156, Middle=156, Ring=150,
  Little=138, Rotate=50` — derivado de `HAND_CONFIGS['palm_grip']`
  (rad/MAX_RAD*200).
- **ECI:** `Cylinder (id 8)`.

### 4.2 Ampola (ampola farmacêutica — r=5 mm, h=75 mm)

- **Tipo de preensão:** Fingertip Pinch (APENAS polegar + indicador;
  médio/anelar/mínimo PERMANECEM em 0).
- **R_tcp:** `_Rtcp_palm_down(finger_dir=(+1,0,0))` — top-down.
- **Pose `pre_approach`:** TCP em `(0.75, 0.00, 1.00)` world.
- **Pose `approach`:** TCP em `(0.75, 0.00, 0.92)` world.
- **Pose `grasp`:** TCP em `(0.75, 0.00, 0.87)` world (pinça polegar+
  indicador converge sobre o ombro da ampola, 11 mm abaixo do topo).
- **Pré-shape Pinch:** `Thumb=55, Index=50, Middle=0, Ring=0,
  Little=0, Rotate=82` — polegar e indicador parcialmente flexionados;
  M/R/L mantidos abertos.
- **Grasp final Pinch:** `Thumb=106, Index=100, Middle=0, Ring=0,
  Little=0, Rotate=164` — derivado de `HAND_CONFIGS['fingertip_grip']`.
- **ECI:** `Prec. Closed (id 5)`.

### 4.3 Tubo (tubo de ensaio — r=12 mm, h=120 mm)

- **Tipo de preensão:** Claw lateral (preensão em garra, do lado).
- **R_tcp:** `_Rtcp_lateral_claw(palm_dir=(0,+1,0), finger_dir=(0,0,−1))`
  — palma normal +Y, dedos descem em −Z.
- **Pose `pre_approach`:** TCP em `(0.75, −0.12, 0.866)` world — mão
  chega de −Y com palma encarando +Y.
- **Pose `approach`:** TCP em `(0.75, −0.05, 0.866)` world.
- **Pose `grasp`:** TCP em `(0.75, 0.00, 0.866)` world — centro do
  tubo. Os 4 dedos longos descem cruzando o eixo do tubo; polegar
  pressiona do lado oposto.
- **Pré-shape Claw:** `Thumb=60, Index=70, Middle=70, Ring=65,
  Little=60, Rotate=45` — ≈50 % do grip final.
- **Grasp final Claw:** `Thumb=113, Index=138, Middle=138, Ring=131,
  Little=119, Rotate=90` — derivado de `HAND_CONFIGS['claw_grip']`.
- **ECI:** `Tripod (id 1)`.

---

## 5. Arquitetura

```
                ┌────────────────────────┐
                │   docs/SDD_*.md (este) │
                │     única fonte        │
                └───────────┬────────────┘
                            ▼
              ┌────────────────────────────┐
              │  grasp_ml_pack/poses.py    │
              │  APPROACH_POSES_DEG[obj]   │
              │  PICK_POSES_DEG[obj]       │
              │  PRE_APPROACH_POSES_DEG    │
              │  HAND_PRESHAPE[grip]       │
              │  HAND_GRIPS[grip]          │
              │  APPROACH_VEC[obj]         │
              └──────────┬────────┬────────┘
                         │        │
            ┌────────────┘        └────────────┐
            ▼                                  ▼
  manual_control_node.py                 grasp_executor.py
    (GUI + pick step-by-step)            (ciclo autônomo)
```

A criação de `poses.py` é o **primeiro passo** da implementação. O
módulo é puro (sem dependências ROS), e expõe IK lazy via
`kinematics.py`.

---

## 6. Ciclo de pick — 6 fases

| Fase | Nome           | Cinemática        | Duração padrão | Ação na mão               |
| ---- | -------------- | ----------------- | -------------- | -------------------------- |
| F1   | pre-approach   | Articular         | 2.0 s          | Abrir totalmente           |
| F2   | approach       | Cartesiano linear | 1.2 s          | (sem ação)                 |
| F3   | preshape       | (parado)          | 0.6 s          | Aplicar `HAND_PRESHAPE`    |
| F4   | descend/advance| Cartesiano linear | 1.2 s          | (sem ação)                 |
| F5   | close          | (parado)          | 1.5 s          | `smart_close` incremental  |
| F6   | lift           | Cartesiano linear | 1.2 s          | (manter fechado)           |

Para frasco/ampola, F2 e F4 se traduzem em descidas em −Z. Para tubo,
F2 e F4 se traduzem em deslocamentos em +Y (lateral).

## 7. Ciclo de entrega — 4 sub-fases

| Fase | Nome         | Cinemática        | Duração | Ação na mão              |
| ---- | ------------ | ----------------- | ------- | ------------------------- |
| F1'  | pre-deliver  | Articular         | 2.0 s   | (manter fechado)          |
| F2'  | descend bin  | Cartesiano (−Z)   | 1.2 s   | (manter fechado)          |
| F3'  | release      | (parado)          | 0.9 s   | Abrir totalmente          |
| F4'  | retract      | Cartesiano (+Z)   | 1.2 s   | (manter aberto)           |

## 8. Critérios de aceitação

- **CA1** — Cada uma das 3 poses `pre_approach` é visualmente
  semelhante a `images/palmGrip.png` (frasco), `images/fingertip.png`
  (ampola) e `images/claw.png` (tubo). Avaliação por screenshot.
- **CA2** — 9 execuções consecutivas (3 spawns × 3 objetos) terminam
  com objeto efetivamente entregue na caixa de cor correta, sem
  escorregamento na fase F6.
- **CA3** — Nenhum dedo penetra o AABB do objeto antes da F5
  (verificado via `_fingers_clear_objects` em sweep).
- **CA4** — Pose mantida em F4 do tubo deixa os 4 dedos paralelos ao
  eixo do tubo (não-cruzados); o polegar fica do lado oposto ao
  operador.
- **CA5** — Em modo step-by-step, cada fase aguarda confirmação manual
  e o LED da fase em curso fica amarelo; ao concluir, verde; ao
  abortar, vermelho.

---

## 9. Tasks (T1–T20)

T1. Criar `grasp_ml_pack/poses.py` com `APPROACH_VEC[obj]`,
    `PRE_APPROACH_POSES_DEG[obj]`, `APPROACH_POSES_DEG[obj]`,
    `PICK_POSES_DEG[obj]`, `HAND_PRESHAPE[grip]`, `HAND_GRIPS[grip]`.

T2. Migrar `manual_control_node.py` para consumir `poses.py` (remover
    dicts locais duplicados).

T3. Implementar IK por objeto: 3 approachs distintos, 3 picks distintos
    (em `poses.py._compute_targets()`), com seeds fallback verificados.

T4. Definir `HAND_PRESHAPE` por preensão (Palm/Claw/Pinch).

T5. Atualizar `HAND_GRIPS` finais (especialmente Pinch — só polegar +
    indicador, sem médio/anelar/mínimo).

T6. Implementar `_do_pick_cycle` em 6 fases (F1..F6) com mensagens de
    status, encadeamento via `root.after`.

T7. Adicionar **card "APROACH POR OBJETO"** na aba Mão COVVI: 3 botões
    (Frasco top-down, Ampola top-down, Tubo lateral). Botão = move ao
    `pre_approach` + aplica pré-shape sem fechar.

T8. Adicionar **card "SEQUÊNCIA DE GRASP" (step-by-step)** na aba
    Célula: 6 LEDs (F1..F6), botão "Iniciar", botão "Próxima fase"
    quando step-by-step ativo, botão "Abortar".

T9. Adicionar **card "Parâmetros do Ciclo"** na aba Célula: spinboxes
    para offset de descida, duração F4, duração F6, touch threshold.

T10. Implementar `_do_delivery` em 4 sub-fases (F1'..F4'), com
     orientação preservada até F3' para o tubo.

T11. Estender `_path_max_safe_alpha` para considerar a pré-shape
     vigente durante o sweep de F2/F4.

T12. Verificar que `_fingertips_world` reflete o pré-shape real (não
     o slider state), evitando falsos positivos.

T13. Replicar `poses.py` em `grasp_executor.py` (substituir
     `_PICK_SEED_Q` e poses locais por chamadas a `poses.py`).

T14. Atualizar `MEMORY.md` para refletir a nova arquitetura de poses.

T15. Testes manuais (CA1) — capturar 3 screenshots de pre-approach por
     objeto e comparar com `images/{palmGrip,fingertip,claw}.png`.

T16. Testes manuais (CA2) — 9 ciclos completos, anotar falhas.

T17. Testes manuais (CA5) — validar fluxo step-by-step com LEDs.

T18. Refinamento iterativo: ajustar `_PICK_TCP_WORLD` por objeto até
     CA2 ≥ 9/9.

T19. Atualizar `README.md` com fluxograma de 6 fases e screenshots
     novos.

T20. Code review final + tag `v1.0.0-grasp-perfection`.

---

## 9.1 Tasks de refinamento metódico (T21–T30)

Adicionadas em v1.2 após relatos de palma sem contato (frasco) e grips
parecidos. Foco em **calibração precisa** com base na geometria física.

T21. **Documentar geometria** — preencher §4.0 com URDF + SDF (objetos,
     braço, mão, dedos, MCPs). ✅
T22. **Restaurar HAND_CONFIGS originais** — `poses.HAND_GRIPS` deve
     espelhar `kinematics.HAND_CONFIGS` 1:1 (palm_grip / claw_grip /
     fingertip_grip). ✅
T23. **Calibrar TCP de pick por geometria** — para cada objeto, derivar
     z_pick a partir de obj_top, fingertip-curl-reach (≈24 mm) e
     Link6_STL_clearance (≈64 mm de belt). ✅
T24. **Adicionar parâmetro "descent_extra" na GUI** — spinbox -30..+30
     no Cell tab (controla F4); `_adjusted_pick_pose` aplica via
     joint2/joint3. Sinal: +mm = palma desce mais. ✅
T25. **Calibrar descent por objeto** — `scripts/tune_descent.py`
     reporta colisão + folga palma-topo para d_extra ∈ {−30..+50}mm.
     Resultado: frasco/ampola limitados a d≤0 (Link6 toca belt);
     tubo tem 50mm de margem. ✅
T26. **Ajuste fino de pré-shape contra penetração** —
     `_apply_arm(hand_state_override)` propaga preshape ao sweep
     `_path_max_safe_alpha`. F4 do `_do_pick_cycle` passa
     `_preshape_hand_state(grip_key)` como override. ✅
T27. **Visualização do TCP em RViz** — `_publish_tcp_target` emite
     TF `tcp_pre / tcp_appr / tcp_pick` no `world` a cada início de
     ciclo (via `tf2_ros.TransformBroadcaster`). ✅
T28. **Auto-tuning do Rotate (polegar)** —
     `scripts/tune_rotate.py` varre Rotate ∈ {0..200} e mede
     distância polegar-objeto via FK. Resultados:
     frasco Rotate=200 → 5mm; tubo Rotate=120 → 11mm; ampola
     Rotate=200 → 37mm (pinch curto). ✅
T29. **Modo "Hover" pré-pick** — três botões coloridos no Cell tab +
     "Close (após Hover)". `_do_hover` chama `_do_pick_cycle` com
     `hover_only=True`. ✅
T30. **Validação 9-ciclos automática** — `scripts/test_9cycles.py`
     chama `/conveyor/spawn_<obj>` + `/cell/execute_grasp` para
     3 obj × 3 reps, registra outcome+duration em
     `/tmp/test_9cycles_results.csv`. ✅

---

## 10. Cronograma sugerido

| Sprint | Tasks                   | Duração   |
| ------ | ----------------------- | --------- |
| S1     | T1, T2, T3              | 2 dias    |
| S2     | T4, T5, T6              | 2 dias    |
| S3     | T7, T8, T9              | 2 dias    |
| S4     | T10, T11, T12           | 1,5 dias  |
| S5     | T13, T14                | 1 dia     |
| S6     | T15, T16, T17           | 2 dias    |
| S7     | T18, T19, T20           | 1,5 dias  |

Total estimado: **~12 dias úteis** (≈ 2,5 sprints semanais).

---

## 11. Riscos & mitigação

| Risco                                                  | Mitigação                              |
| ------------------------------------------------------ | -------------------------------------- |
| IK fica em branch inconsistente entre approach e pick  | Seeds versionados em `poses.py`        |
| Pinch da ampola dispersa a ampola ao tocar             | Aumentar Rotate, validar com CA4       |
| Tubo escorrega na fase de lift                         | Adicionar `_close_extra` no lift       |
| pose-time mismatch (mão chega antes do braço)          | Sincronizar via `root.after` por fase  |
| Conflito com `grasp_executor` durante refatoração      | Manter `_FALLBACK_*` antigos até T13   |

---

*FIM do SDD v1.0.0.* Atualizações pontuais (ajustes finos de cota)
devem incrementar o patch (`1.0.1`); mudanças de fluxo de fases
incrementam o minor (`1.1.0`); reescritas de arquitetura incrementam
o major (`2.0.0`).
