# arm_controller.py
from controller import Robot
import math

TIME_STEP = 32

# inicialização
robot = Robot()

# nomes das juntas do UR5e
joint_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
]

# pega cada motor, configura posição e velocidade
joints = {}
for name in joint_names:
    m = robot.getDevice(name)
    if m is None:
        print(f"⚠️ Device '{name}' not found on UR5e")
    else:
        m.setPosition(0.0)    # posição inicial
        m.setVelocity(1)    # velocidade máxima
    joints[name] = m

# loop principal
while robot.step(TIME_STEP) != -1:
    t = robot.getTime()
    # Exemplo de controle simples: oscila each joint
    for name, m in joints.items():
        if m:
            target = 0.5 * math.sin(t)  # só como demonstração
            m.setPosition(target)
