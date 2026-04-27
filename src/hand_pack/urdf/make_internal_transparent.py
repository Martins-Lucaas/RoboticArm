import os
import re

def stabilize_urdf(filename):
    print(f"A processar arquivo: {filename}...")
    
    if not os.path.exists(filename):
        print("Erro: Arquivo não encontrado.")
        return

    with open(filename, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    new_lines = []
    is_l1_link = False
    
    # Regex para capturar a cor e a massa
    color_re = re.compile(r'rgba="([\d\.]+)\s+([\d\.]+)\s+([\d\.]+)\s+([\d\.]+)"')
    mass_re = re.compile(r'mass value="([\d\.]+)"')

    for line in lines:
        # Detecta se entramos num link do tipo _l1
        if '<link name="' in line and '_l1"' in line:
            is_l1_link = True
        
        if is_l1_link:
            # 1. Força Transparência (Alpha = 0.0)
            if '<color rgba=' in line:
                match = color_re.search(line)
                if match:
                    r, g, b, _ = match.groups()
                    line = re.sub(color_re, f'rgba="{r} {g} {b} 0.0"', line)
            
            # 2. Força Massa Desprezível (para parar o pêndulo)
            if '<mass value=' in line:
                line = re.sub(mass_re, 'mass value="0.0001"', line)
                
            # 3. Zera a Inércia (torna o link "leve" para o motor)
            if '<inertia' in line:
                line = '            <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001" />\n'

        if '</link>' in line:
            is_l1_link = False
            
        new_lines.append(line)

    with open(filename, 'w', encoding='utf-8') as f:
        f.writelines(new_lines)
    print("✅ Concluído: Links L1 agora são invisíveis e ultra-leves.")

if __name__ == '__main__':
    stabilize_urdf('linear_covvi_hand_gazebo.urdf')