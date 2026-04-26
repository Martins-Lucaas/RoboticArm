import xml.etree.ElementTree as ET

def merge_urdfs():
    print("🛠️ Iniciando a construção do Gêmeo Digital de Alta Fidelidade...")

    # Nomes dos seus arquivos (ajuste se o nome do seu original for diferente)
    file_gazebo = 'linear_covvi_hand_gazebo.urdf'
    file_cad = 'linear_covvi_hand_right.urdf'
    file_perfect = 'linear_covvi_hand_perfect.urdf'

    # 1. Carrega o Gazebo URDF (Para extrair a base_link e os controles ros2)
    tree_gaz = ET.parse(file_gazebo)
    root_gaz = tree_gaz.getroot()

    # 2. Carrega o CAD URDF (Para extrair toda a cinemática, lisa, knuckles, etc)
    tree_cad = ET.parse(file_cad)
    root_cad = tree_cad.getroot()

    # Nova raiz do robô
    new_root = ET.Element('robot', {'name': 'covvi_digital_twin_perfect'})

    # --- PARTE 1: BASE FOOTPRINT E MUNDO (Do Gazebo) ---
    keep_links = ['world', 'base_footprint', 'base_link']
    for link in root_gaz.findall('link'):
        if link.get('name') in keep_links:
            new_root.append(link)

    for joint in root_gaz.findall('joint'):
        if joint.get('name') in ['world_fixed', 'base_joint']:
            new_root.append(joint)

    # --- PARTE 2: A INTRINCADA TEIA DO CAD ---
    ignore_cad_links = ['__base_link', '_base_link', 'base_link']
    
    for link in root_cad.findall('link'):
        name = link.get('name')
        if name not in ignore_cad_links:
            
            # O SEGREDO DO GAZEBO: Elos invisíveis (l0, l1) PRECISAM ter massa e inércia > 0
            inertial = link.find('inertial')
            if inertial is None:
                inertial = ET.SubElement(link, 'inertial')

            mass = inertial.find('mass')
            if mass is None:
                mass = ET.SubElement(inertial, 'mass')
            if float(mass.get('value', '0')) <= 0.001:
                mass.set('value', '0.01') # Massa mínima para estabilidade

            inertia = inertial.find('inertia')
            if inertia is None:
                inertia = ET.SubElement(inertial, 'inertia')
            for axis in ['ixx', 'iyy', 'izz']:
                if float(inertia.get(axis, '0')) <= 0.00001:
                    inertia.set(axis, '0.00001')

            # Consertar materiais vazios que deixam a malha invisível
            visual = link.find('visual')
            if visual is not None:
                mat = visual.find('material')
                if mat is not None and not mat.get('name'):
                    mat.set('name', 'generic_mat')
                    
            # Adicionar tags de COLISÃO usando a mesma geometria visual
            if not name.startswith('_'): # Apenas nas peças visíveis
                if visual is not None and link.find('collision') is None:
                    collision = ET.SubElement(link, 'collision')
                    geom = visual.find('geometry')
                    if geom is not None:
                        collision.append(ET.fromstring(ET.tostring(geom)))
                    orig = visual.find('origin')
                    if orig is not None:
                        collision.append(ET.fromstring(ET.tostring(orig)))

            new_root.append(link)

    # Injetar as Juntas do CAD
    ignore_cad_joints = ['__joint_base_link', '_joint_base_link']
    for joint in root_cad.findall('joint'):
        if joint.get('name') not in ignore_cad_joints:
            
            # Redirecionar as juntas antigas do pulso para a nossa nova base_link
            parent = joint.find('parent')
            if parent is not None and parent.get('link') in ignore_cad_links:
                parent.set('link', 'base_link')

            # Limites de esforço não podem ser ZERO no Gazebo
            limit = joint.find('limit')
            if limit is not None:
                if float(limit.get('effort', '0')) == 0:
                    limit.set('effort', '10.0')
                if float(limit.get('velocity', '0')) == 0:
                    limit.set('velocity', '1.0')

            new_root.append(joint)

    # --- PARTE 3: CONTROLES DO GAZEBO ---
    for gazebo_tag in root_gaz.findall('gazebo'):
        new_root.append(gazebo_tag)

    for ros2_control in root_gaz.findall('ros2_control'):
        new_root.append(ros2_control)

    # Salva o arquivo XML estruturado
    try:
        ET.indent(new_root, space="    ", level=0)
    except AttributeError:
        pass # Ignora se a versão do Python for antiga
        
    tree_new = ET.ElementTree(new_root)
    tree_new.write(file_perfect, encoding='utf-8', xml_declaration=True)
    
    print(f"✅ Sucesso! O arquivo '{file_perfect}' foi gerado.")
    print("Agora ele contém toda a cinemática complexa, limites, mimic joints e massa estabilizada!")

if __name__ == "__main__":
    merge_urdfs()