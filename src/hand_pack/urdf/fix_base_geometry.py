import re

def fix_urdf_geometry(input_file, output_file):
    with open(input_file, 'r') as f:
        content = f.read()

    # Configurações do modelo correto extraídas do CAD original
    mesh_path = "package://hand_pack/urdf/linear_meshes/right_base_link.STL"
    mesh_scale = "0.0625 0.0625 0.0625"
    
    # 1. Definir o bloco de geometria correto
    correct_geometry = f'<geometry><mesh filename="{mesh_path}" scale="{mesh_scale}"/></geometry>'

    # 2. Regex para capturar e substituir especificamente os links de base que estão como 'box'
    # Alvos: __base_link e _base_link
    target_links = ["__base_link", "_base_link"]
    
    for link in target_links:
        # Padrão para encontrar o link e suas tags internas de visual e collision
        pattern = rf'(<link name="{link}">.*?)<geometry><box size="1 1 1"/></geometry>(.*?<collision>.*?)<geometry><box size="1 1 1"/></geometry>'
        
        # Substituição mantendo o resto da estrutura (origens, materiais, etc)
        replacement = rf'\1{correct_geometry}\2{correct_geometry}'
        content = re.sub(pattern, replacement, content, flags=re.DOTALL)

    # 3. Ajuste opcional: Se houver massas de 1kg (padrão) nos links de driver, 
    # reduzimos para valores reais da COVVI (aprox 5g a 15g por peça pequena)
    content = re.sub(r'<mass value="1"/>', '<mass value="0.015"/>', content)

    with open(output_file, 'w') as f:
        f.write(content)
    
    print(f"Sucesso! O arquivo '{output_file}' agora utiliza o CAD real da COVVI.")

# Executar a correção
fix_urdf_geometry('linear_covvi_hand_gazebo.urdf', 'linear_covvi_hand_gazebo_fixed.urdf')