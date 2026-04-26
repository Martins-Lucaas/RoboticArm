import os
import xml.etree.ElementTree as ET

def fix_digital_twin():
    urdf_file = 'linear_covvi_hand_gazebo.urdf'
    mesh_dir = 'linear_meshes' 

    if not os.path.exists(urdf_file):
        print(f"Erro: Arquivo {urdf_file} não encontrado!")
        return

    tree = ET.parse(urdf_file)
    root = tree.getroot()

    # 1. CONSERTAR JUNTAS (O que impede o erro -11)
    for joint in root.findall('joint'):
        limit = joint.find('limit')
        if limit is not None:
            # Atribui valores reais: 90N de força e 2.0 rad/s de velocidade
            limit.set('effort', '90.0')
            limit.set('velocity', '2.0')

    # 2. CONSERTAR LINKS E MALHAS
    for link in root.findall('link'):
        link_name = link.get('name')
        if not link_name or link_name == "world": continue

        # Identifica se a peça tem um arquivo STL correspondente
        possible_stl = [f"right_{link_name}.STL", f"right_{link_name.lstrip('_')}.STL", f"{link_name}.STL"]
        stl_found = None
        if os.path.exists(mesh_dir):
            for stl in possible_stl:
                if os.path.exists(os.path.join(mesh_dir, stl)):
                    stl_found = stl
                    break

        # Ajusta Inércia e Massa para estabilidade
        inertial = link.find('inertial')
        if inertial is not None:
            mass = inertial.find('mass')
            if mass is not None: mass.set('value', '0.05')
            inertia = inertial.find('inertia')
            if inertia is not None:
                for axis in ['ixx', 'iyy', 'izz']:
                    inertia.set(axis, '0.0001')

        # Configura Visual (CAD real vs Invisível)
        visual = link.find('visual')
        if visual is not None:
            geom = visual.find('geometry')
            mat = visual.find('material')
            if stl_found:
                if geom is not None:
                    geom.clear()
                    ET.SubElement(geom, 'mesh', {
                        'filename': f"package://hand_pack/urdf/linear_meshes/{stl_found}",
                        'scale': "0.0625 0.0625 0.0625"
                    })
                if mat is not None:
                    color = mat.find('color')
                    if color is None: color = ET.SubElement(mat, 'color')
                    color.set('rgba', '0.2 0.2 0.2 1.0') # Cinza COVVI
            else:
                # Se não tem STL, torna o link invisible
                if geom is not None:
                    geom.clear()
                    ET.SubElement(geom, 'box', {'size': "0.001 0.001 0.001"})
                if mat is not None:
                    color = mat.find('color')
                    if color is None: color = ET.SubElement(mat, 'color')
                    color.set('rgba', '0 0 0 0')

    # Salva o arquivo sem a declaração Unicode que o Humble rejeita
    with open(urdf_file, 'wb') as f:
        f.write(ET.tostring(root))
        
    print(f"\n✅ DIGITAL TWIN PRONTO!")
    print(f"🚀 Esforço das juntas: 90N (Resolvido erro -11)")
    print(f"💎 Peças reais mapeadas para STL e links extras ocultados.")

if __name__ == '__main__':
    fix_digital_twin()
