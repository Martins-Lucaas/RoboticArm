* Digital Twin & ROS 2 Control

Este repositório contém o Gêmeo Digital (Digital Twin) da mão biônica COVVI, desenvolvido para simulação avançada e controle em tempo real utilizando ROS 2 e Gazebo. O projeto foca na precisão cinemática, estabilidade física e facilidade de operação manual através de uma interface gráfica (GUI) dedicada.
🚀 Funcionalidades

    Simulação de Alta Fidelidade: URDF otimizado com malhas (meshes) originais da COVVI Hand.

    Estabilidade Física: Correção de inércias, massas e tratamento de juntas mecânicas auxiliares (fixed joints) para evitar oscilações em ambiente Gazebo.

    Interface de Controle Customizada: GUI desenvolvida em Python (Tkinter) para controle individual de cada dedo e rotação do polegar, com ajuste dinâmico de velocidade e interpolação.

    Integração ROS 2 Control: Utiliza joint_trajectory_controller para movimentos suaves e precisos.

    Arquitetura Modular: Pronto para integração com braços robóticos industriais (UR, Kuka, etc.) ou próteses biomédicas.

🛠️ Requisitos do Sistema
Pré-requisitos

    Sistema Operacional: Ubuntu 22.04 (Jammy Jellyfish)

    ROS 2: Humble Hawksbill

    Simulador: Gazebo Classic

Dependências do ROS 2

Certifique-se de ter os seguintes pacotes instalados:
Bash

sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs \
                 ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-xacro \
                 python3-tk -y

📂 Estrutura do Projeto
Plaintext

hand_pack/
├── config/             # Configurações do ROS 2 Control (YAML)
├── hand_pack/          # Módulo Python (Scripts da GUI e Lógica)
│   ├── hand_gui.py     # Interface gráfica de sliders
├── launch/             # Arquivos de inicialização (XML/Python)
├── urdf/               # Modelos cinemáticos e arquivos Xacro
│   └── linear_meshes/  # Malhas 3D (.STL) da mão
└── worlds/             # Ambientes de simulação do Gazebo

🔧 Instalação

    Clone o repositório no seu workspace:
    Bash

    cd ~/RoboticArm/src
    git clone https://github.com/Martins-Lucaas/RoboticArm.git .

    Compile o workspace:
    Bash

    cd ~/RoboticArm
    colcon build --packages-select hand_pack
    source install/setup.bash

🎮 Como Usar

Para lançar a simulação no Gazebo junto com a interface de controle manual, execute:
Bash

ros2 launch hand_pack spawn_hand.launch.xml

Controles da GUI:

    Tempo de Movimento: Define quão rápido os dedos fecham (segundos).

    Sliders Proximais: Controlam a flexão principal (Thumb, Index, Middle, Ring, Little).

    Rotate: Controla a oposição/rotação do polegar.

    Sliders Distais (_j01): Controlam as falanges das pontas dos dedos.

🧠 Detalhes Técnicos (Physics Tuning)

Para atingir o estado de "Digital Twin perfeito", este projeto implementa:

    Mímica de Juntas: Links mecânicos de suporte (_l1) configurados como invisíveis e fixos para garantir a integridade da malha visual sem afetar a física.

    Power-Up de Atuadores: Limites de esforço (effort="1000") e velocidade ajustados para simular a força real dos motores COVVI sem atrasos de processamento.

🤝 Contribuição

Contribuições para melhorar a estabilidade física ou adicionar grips (gestos) pré-definidos são bem-vindas. Sinta-se à vontade para abrir uma Issue ou enviar um Pull Request.
📄 Licença

Este projeto é distribuído sob a licença Apache-2.0.

Desenvolvido por: Lucas Martins
