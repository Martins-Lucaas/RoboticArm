"""Helpers de pós-processamento do URDF da mão COVVI.

Reúnem as transformações aplicadas em tempo de launch sobre o URDF
exportado do Onshape (`linear_covvi_hand_gazebo.urdf`):

  * :func:`clamp_hand_joint_limits` — restringe as juntas primárias
    (Thumb/Index/Middle/Ring/Little/Rotate) aos limites factíveis do
    manual técnico da COVVI Hand (CV-000918-TC, Rev. 6, seção 4.5),
    reduzindo também os limites das juntas-espelho (mimic) para o
    valor coerente com o novo cap do driver.

  * :func:`fix_virtual_link_inertia` — substitui a inércia "fantasma"
    de 1 kg/1.0·I por valores pequenos nos eixos virtuais dos
    mecanismos de 4-barras (mantém o solver estável).

  * :func:`stabilize_hand_joints` — ajusta damping, friction e effort
    para grasp por contato físico (cf. SDD v1.0.0).

  * :func:`inject_skin_layer` — adiciona uma camada de "pele" macia
    (envelope inflado) ao redor das falanges e da palma para
    suavizar cantos vivos das AABBs do export.

Todas operam sobre o *corpo* do URDF (string contida em ``<robot>…</robot>``)
e devolvem uma nova string com as substituições aplicadas. São idempotentes
no nível do contrato: aplicá-las duas vezes não produz erro, embora o
resultado da segunda aplicação possa ser redundante.
"""

from __future__ import annotations

import re
from typing import Dict


# ---------------------------------------------------------------------------
# 1) Limites factíveis das juntas — baseado no manual técnico da COVVI Hand
# ---------------------------------------------------------------------------
#
# Manual CV-000918-TC Rev. 6 (Technical Manual UK) — "Finger flexion: 81°"
# (publicado também na ManualsLib pela COVVI Ltd.). Esse 81° é a rotação
# TOTAL da ponta do dedo (fingertip), do estado aberto ao fechado, e NÃO
# a rotação do driver no URDF.
#
# Esta mão é underactuated por linkage: o driver é o curso normalizado do
# atuador, e as juntas espelho (mimic) somam-se na cadeia cinemática:
#
#   fingertip_flex ≈ driver × (mult_proximal + mult_distal)
#
# Multiplicadores observados no `linear_covvi_hand_gazebo.urdf`:
#
#   Index/Middle/Ring/Little: prox ≈ 1.516,  dist ≈ 1.336  →  Σ ≈ 2.852
#   Thumb (sem proximal+distal nominais, três mimic na cadeia):
#                              dist ≈ 1.067, link ≈ 0.768  →  Σ ≈ 1.787
#
# Para 81° na ponta (1.414 rad):
#   • Fingers   → driver = 1.414 / 2.852 = 0.496 rad (~28.4°).
#   • Thumb     → driver = 1.414 / 1.787 = 0.791 rad (~45.3°).
#
# Damos uma folga de 10–15% sobre o valor exato (para o usuário poder
# fechar um pouco mais em power-grip sem auto-colisão imediata) e
# arredondamos os caps abaixo. Acima desses valores a falange distal
# entra no volume da palma.
#
# Para "Rotate" (oposição do polegar) o manual não publica grau, mas o
# curso observado no produto físico (~57°) ≈ 1.0 rad — preservado do URDF.
HAND_DRIVER_LIMITS: Dict[str, float] = {
    'Thumb':  1.00,   # ~57° driver → 102° na ponta
    'Index':  1.00,   # ~57° driver → 163° na ponta (wrap de power-grip)
    'Middle': 1.00,
    'Ring':   1.00,
    'Little': 1.00,
    'Rotate': 1.00,
}

# Note: com `enable_inter_finger_self_collision` ativo (skin + palm em
# self_collide=true), a falange distal NÃO atravessa a palma mesmo no
# cap 1.0 — o ODE detecta o contato e bloqueia. Por isso podemos abrir
# o cap até wrap de ~163° (suficiente para envelopar um cilindro de
# 45mm de raio), mantendo a segurança física por colisão.

# ``lower`` calibrado — equivalente ao ``open_limit`` do
# DigitConfigMsg da mão real. O zero geométrico do URDF (saída do
# Onshape) coloca as falanges totalmente estendidas, mas a mão COVVI
# física repousa com uma leve curvatura natural (mola do mecanismo
# 4-barras + tendões em tensão de repouso). Sem este offset, a sim
# fica visivelmente "mais ereta" que o produto real.
#
# Σ(mults) para cada cadeia (vide ``hand_gui.MIMIC_JOINTS``):
#   Index/Middle/Ring/Little ≈ 2.85 → 0.12 rad driver ≈ 20° na ponta
#   Thumb ≈ 1.79              → 0.08 rad driver ≈ 14° na ponta
#
# Estes valores casam com fotos de produto do Nexus Hand em repouso
# (catálogo COVVI e ManualsLib). Para calibrar contra uma unidade
# específica, ler o ``open_limit`` real via ``GetDigitConfig`` e
# converter o valor uint8 (0..100) em rad via cap correspondente.
HAND_DRIVER_LOWER: Dict[str, float] = {
    'Thumb':  0.08,
    'Index':  0.12,
    'Middle': 0.12,
    'Ring':   0.12,
    'Little': 0.12,
    'Rotate': 0.00,   # oposição mantém neutra (palma aberta)
}


def clamp_hand_joint_limits(urdf_body: str) -> str:
    """Ajusta ``lower``/``upper`` dos drivers e propaga para os mimics.

    Para cada junta revolute, se a sua *driver* (ela própria, no caso
    de Thumb/Index/.../Rotate, ou a junta referenciada em ``<mimic>``)
    estiver em :data:`HAND_DRIVER_LIMITS`, atualiza ``lower`` (de
    :data:`HAND_DRIVER_LOWER`) e ``upper`` do ``<limit>``. Para mimics,
    o intervalo equivalente é ``[mult·driver_lower, mult·driver_upper]``
    (ordenado, para multipliers negativos não inverterem o intervalo;
    offset do URDF é < 1e-7 e portanto desprezado).
    """
    joint_re = re.compile(r'<joint\b[^>]*?\bname="([^"]+)"[^>]*?>.*?</joint>',
                          re.DOTALL)
    mimic_re = re.compile(
        r'<mimic\s+joint="([^"]+)"\s+multiplier="([-\deE.+]+)"'
        r'(?:\s+offset="[-\deE.+]+")?\s*/>')
    limit_lower_re = re.compile(r'(<limit\b[^/]*?\blower=")([-\deE.+]+)(")')
    limit_upper_re = re.compile(r'(<limit\b[^/]*?\bupper=")([-\deE.+]+)(")')

    def _patch_joint(match: re.Match) -> str:
        jxml = match.group(0)
        jname = match.group(1)

        new_lower: float | None = None
        new_upper: float | None = None
        if jname in HAND_DRIVER_LIMITS:
            new_lower = HAND_DRIVER_LOWER[jname]
            new_upper = HAND_DRIVER_LIMITS[jname]
        else:
            m = mimic_re.search(jxml)
            if m is not None:
                driver = m.group(1)
                mult = float(m.group(2))
                if driver in HAND_DRIVER_LIMITS:
                    a = mult * HAND_DRIVER_LOWER[driver]
                    b = mult * HAND_DRIVER_LIMITS[driver]
                    new_lower, new_upper = (a, b) if a <= b else (b, a)

        if new_upper is None:
            return jxml

        jxml = limit_lower_re.sub(
            lambda lm: f'{lm.group(1)}{new_lower:.8f}{lm.group(3)}',
            jxml, count=1)
        jxml = limit_upper_re.sub(
            lambda lm: f'{lm.group(1)}{new_upper:.8f}{lm.group(3)}',
            jxml, count=1)
        return jxml

    return joint_re.sub(_patch_joint, urdf_body)


# ---------------------------------------------------------------------------
# 2) Inércia "fantasma" dos eixos virtuais — pequena & estável
# ---------------------------------------------------------------------------
def fix_virtual_link_inertia(urdf_body: str) -> str:
    phantom = (
        r'<inertial>\s*'
        r'<mass value="1"\s*/>\s*'
        r'<inertia ixx="1\.0" ixy="0\.0" ixz="0\.0" iyy="1\.0" iyz="0\.0" izz="1\.0"\s*/>\s*'
        r'</inertial>'
    )
    minimal = (
        '<inertial>'
        '<mass value="0.001"/>'
        '<inertia ixx="1e-9" ixy="0.0" ixz="0.0" iyy="1e-9" iyz="0.0" izz="1e-9"/>'
        '</inertial>'
    )
    return re.sub(phantom, minimal, urdf_body, flags=re.DOTALL)


# ---------------------------------------------------------------------------
# 3) Dinâmica das juntas — preensão por contato físico
# ---------------------------------------------------------------------------
def stabilize_hand_joints(urdf_body: str) -> str:
    """Patcha damping/friction/effort das revolute joints.

    Drivers ficam com effort=8 N·m (suficiente para grip ~30 kg conforme
    manual, com folga). Mimics recebem damping mais alto (30) para que o
    acoplamento seja "rígido" sem oscilação.
    """
    def _patch(m: re.Match) -> str:
        jxml = m.group(0)
        if 'type="revolute"' not in jxml:
            return jxml
        is_mimic = '<mimic' in jxml
        damp, fric = (30.0, 10.0) if is_mimic else (5.0, 1.0)
        dyn_tag = f'<dynamics damping="{damp}" friction="{fric}"/>'
        if '<dynamics' in jxml:
            jxml = re.sub(r'<dynamics[^/]*/>', dyn_tag, jxml)
        else:
            jxml = jxml.replace('</joint>',
                                f'      {dyn_tag}\n    </joint>')
        if not is_mimic:
            jxml = re.sub(r'effort="[\d.]+"', 'effort="8.0"', jxml)
        return jxml
    return re.sub(r'<joint\b[^>]*>.*?</joint>', _patch, urdf_body, flags=re.DOTALL)


# ---------------------------------------------------------------------------
# 4) Camada de "pele" macia — análoga à luva de silicone IP44 da COVVI
# ---------------------------------------------------------------------------
def inject_skin_layer(urdf_body: str, inflate_m: float = 0.002) -> str:
    """Adiciona <collision name="skin"> ao redor de cada falange + palma.

    Args:
        urdf_body: corpo do URDF (sem a tag <robot> externa, ou com — não
            importa: as substituições são por padrão regex local).
        inflate_m: espessura por face da "pele", em metros. Default 2 mm
            (a COVVI publica luva IP44 com perfil similar). Valores típicos:
            0.002 (pacote padrão), 0.003 (envelope mais conservador,
            agravam auto-colisão), 0.001 (pele mínima, cantos ainda
            visíveis ao objeto).
    """
    finger_link_pat = re.compile(
        r'(<link\s+name="(?:thumb|index|middle|ring|little)_(?:proximal|distal)"[^>]*>'
        r'.*?</link>)',
        re.DOTALL)
    box_coll_pat = re.compile(
        r'(<collision>\s*<geometry>\s*<box\s+size="([^"]+)"\s*/>\s*</geometry>'
        r'\s*<origin\s+xyz="([^"]+)"\s+rpy="([^"]+)"\s*/>\s*</collision>)',
        re.DOTALL)

    def _patch_finger_link(m: re.Match) -> str:
        link_xml = m.group(0)
        if '<collision name="skin"' in link_xml:
            return link_xml  # idempotência

        def _patch_collision(cm: re.Match) -> str:
            original = cm.group(1)
            sizes = [float(s) for s in cm.group(2).split()]
            xyz = cm.group(3)
            rpy = cm.group(4)
            inflated = ' '.join(
                f'{s + 2 * inflate_m:.5f}' for s in sizes)
            skin = (
                f'\n        <collision name="skin">'
                f'<geometry><box size="{inflated}"/></geometry>'
                f'<origin xyz="{xyz}" rpy="{rpy}"/>'
                f'</collision>'
            )
            return original + skin

        return box_coll_pat.sub(_patch_collision, link_xml, count=1)

    urdf_body = finger_link_pat.sub(_patch_finger_link, urdf_body)

    # Palma (link 'lisa') — colisão de malha + box-skin aproximado.
    if '<collision name="palm_skin"' not in urdf_body:
        palm_skin = (
            '\n        <collision name="palm_skin">'
            '<geometry><box size="0.085 0.092 0.045"/></geometry>'
            '<origin xyz="0 0.046 0" rpy="0 0 0"/>'
            '</collision>'
        )
        urdf_body = re.sub(
            r'(<link\s+name="lisa">.*?<collision>\s*<geometry>\s*<mesh\b[^>]*?/>'
            r'\s*</geometry>\s*<origin\b[^>]*?/>\s*</collision>)',
            lambda m: m.group(1) + palm_skin,
            urdf_body, count=1, flags=re.DOTALL)

    return urdf_body


# ---------------------------------------------------------------------------
# 5) Habilitar colisão ENTRE dedos diferentes
# ---------------------------------------------------------------------------
#
# Por default, links que compartilham junta no URDF NÃO geram contatos
# uns com os outros — ODE (e Gazebo via gazebo_ros2_control) suprime
# colisão de pares "estruturalmente conectados". Falanges de dedos
# DIFERENTES, porém, não compartilham junta (vão até `base_link`/`lisa`
# pela cadeia), então a colisão entre, p.ex., `index_distal` e
# `middle_distal` SÓ ocorre se `<self_collide>true</self_collide>` for
# explicitamente habilitado em pelo menos uma das partes.
#
# Esta função habilita o flag para todas as 10 falanges + palma. Pares
# que JÁ compartilham junta (proximal↔distal do mesmo dedo) permanecem
# excluídos pelo ODE automaticamente — não geram contatos espúrios.

FINGER_PHALANGE_LINKS: tuple = tuple(
    f'{f}_{s}'
    for f in ('thumb', 'index', 'middle', 'ring', 'little')
    for s in ('proximal', 'distal')
)
PALM_LINK = 'lisa'

INTER_FINGER_COLLISION_LINKS: tuple = FINGER_PHALANGE_LINKS + (PALM_LINK,)


def enable_inter_finger_self_collision(urdf_body: str) -> str:
    """Ativa ``<self_collide>true</self_collide>`` nas falanges e palma.

    Permite que o solver ODE detecte interpenetração ENTRE dedos
    distintos quando trajetórias de juntas comandadas pelo controlador
    levam dois dedos ao mesmo volume — sem essa flag, eles atravessam-se
    livremente porque o default do gazebo_ros é `<self_collide>false</self_collide>`.

    Idempotente: remove qualquer tag pré-existente para o link e
    re-injeta a versão "true".
    """
    parts: list[str] = []
    for link in INTER_FINGER_COLLISION_LINKS:
        # Remove qualquer tag self_collide existente (true ou false)
        # para este link.
        pattern = re.compile(
            rf'<gazebo\s+reference="{re.escape(link)}"\s*>'
            rf'\s*<self_collide>\s*(?:true|false)\s*</self_collide>'
            rf'\s*</gazebo>',
            re.DOTALL)
        urdf_body = pattern.sub('', urdf_body)
        parts.append(
            f'    <gazebo reference="{link}">'
            f'<self_collide>true</self_collide>'
            f'</gazebo>')

    block = '\n' + '\n'.join(parts) + '\n'
    if '</robot>' in urdf_body:
        return urdf_body.replace('</robot>', block + '</robot>', 1)
    return urdf_body + block


# Alias retro-compatível para chamadas antigas.
def disable_intra_finger_self_collision(urdf_body: str) -> str:  # pragma: no cover
    """Deprecated: agora HABILITAMOS colisão entre dedos. Mantido como
    alias temporário para imports antigos."""
    return enable_inter_finger_self_collision(urdf_body)


# ---------------------------------------------------------------------------
# 6) Pele VISUAL — cobre o esqueleto mecânico para parecer com a luva COVVI
# ---------------------------------------------------------------------------
#
# A COVVI Nexus Hand do produto real é coberta por uma luva de silicone
# (carbon black / titan grey / white / rose gold). No URDF de Onshape, as
# falanges e a palma aparecem como aço inoxidável + alumínio cast (cinza
# claro / branco). Para reproduzir a aparência do produto, adicionamos um
# segundo <visual name="skin"> em cada link com a MESMA mesh, escalada
# levemente para fora (4%) e com material escuro semi-translúcido — sobre
# o esqueleto branco, isso produz a aparência de luva cobrindo a estrutura
# mecânica.

_SKIN_VISUAL_SCALE = 1.04        # 4% maior por eixo
_SKIN_VISUAL_COLOR = '0.12 0.12 0.13 1.0'   # Carbon Black opaco
_SKIN_VISUAL_NAME = 'covvi_glove'


def inject_visual_skin_layer(urdf_body: str,
                             scale_factor: float = _SKIN_VISUAL_SCALE,
                             color_rgba: str = _SKIN_VISUAL_COLOR) -> str:
    """Adiciona um <visual name="skin"> escalado sobre cada falange + palma.

    A nova visual usa a MESMA mesh STL do export Onshape, apenas com
    `<scale>` multiplicado por ``scale_factor`` (default 1.04) e material
    escuro tipo "Carbon Black" (rgba ``0.12 0.12 0.13 1.0``). O efeito é
    cobrir o aço/alumínio branco visível das juntas e dar à mão o aspecto
    da luva de silicone do produto real.

    Aplicado em: thumb/index/middle/ring/little × proximal/distal (10
    falanges) + ``lisa`` (palma).
    """
    target_links = {f'{f}_{s}' for f in
                    ('thumb', 'index', 'middle', 'ring', 'little')
                    for s in ('proximal', 'distal')}
    target_links.add('lisa')

    # Importante: excluir self-closing (<link name="x" />), senão a captura
    # casa o auto-fechado e devora conteúdo até o próximo </link>.
    link_re = re.compile(
        r'(<link\s+name="([^"]+)"[^/>]*>)(?!\s*</link>)(.*?)(</link>)',
        re.DOTALL)
    visual_mesh_re = re.compile(
        r'<visual>\s*'
        r'(<origin\b[^/]*/>)\s*'
        r'<geometry>\s*'
        r'<mesh\s+filename="([^"]+)"\s+scale="([\d.eE+\- ]+)"\s*/>\s*'
        r'</geometry>\s*'
        r'<material\b[^>]*>.*?</material>\s*'
        r'</visual>',
        re.DOTALL)

    def _patch_link(m: re.Match) -> str:
        open_tag, link_name, body, close_tag = m.groups()
        if link_name not in target_links:
            return m.group(0)
        if f'<visual name="{_SKIN_VISUAL_NAME}"' in body:
            return m.group(0)  # idempotência

        vm = visual_mesh_re.search(body)
        if vm is None:
            return m.group(0)

        origin = vm.group(1)
        mesh_file = vm.group(2)
        scale_vals = [float(s) for s in vm.group(3).split()]
        new_scale = ' '.join(f'{s * scale_factor:.8f}' for s in scale_vals)
        skin_visual = (
            f'\n        <visual name="{_SKIN_VISUAL_NAME}">\n'
            f'            {origin}\n'
            f'            <geometry>\n'
            f'                <mesh filename="{mesh_file}" scale="{new_scale}" />\n'
            f'            </geometry>\n'
            f'            <material name="{_SKIN_VISUAL_NAME}_mat">\n'
            f'                <color rgba="{color_rgba}" />\n'
            f'            </material>\n'
            f'        </visual>'
        )
        # Insere ao final do body do link, antes de </link>
        return open_tag + body + skin_visual + '\n    ' + close_tag

    return link_re.sub(_patch_link, urdf_body)


def apply_all(urdf_body: str, *,
              skin_inflate_m: float = 0.002,
              visual_skin_scale: float = _SKIN_VISUAL_SCALE,
              visual_skin_color: str = _SKIN_VISUAL_COLOR) -> str:
    """Aplica a pipeline padrão de pós-processamento na ordem correta."""
    urdf_body = fix_virtual_link_inertia(urdf_body)
    urdf_body = clamp_hand_joint_limits(urdf_body)
    urdf_body = stabilize_hand_joints(urdf_body)
    urdf_body = inject_skin_layer(urdf_body, inflate_m=skin_inflate_m)
    urdf_body = inject_visual_skin_layer(urdf_body,
                                          scale_factor=visual_skin_scale,
                                          color_rgba=visual_skin_color)
    urdf_body = enable_inter_finger_self_collision(urdf_body)
    return urdf_body
