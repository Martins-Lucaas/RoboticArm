# arm_controller.py
from controller import Robot
import math

TIME_STEP = 32

# inicialização
robot = Robot()

# nomes das juntas do UR5e
arm_joint_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
]

# pega motores do braço
arm_joints = {}
for name in arm_joint_names:
    m = robot.getDevice(name)
    if m is None:
        print(f"⚠️ Device '{name}' not found on UR5e")
    else:
        m.setPosition(0.0)
        m.setVelocity(1)
    arm_joints[name] = m


# loop principal
while robot.step(TIME_STEP) != -1:
    t = robot.getTime()

    # movimento oscilatório do braço
    for name, m in arm_joints.items():
        if m:
            target = 0.5 * math.sin(t)
            m.setPosition(target)
