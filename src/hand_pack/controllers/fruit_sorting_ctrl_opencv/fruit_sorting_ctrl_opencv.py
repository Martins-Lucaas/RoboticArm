"""fruit_sorting_ctrl_opencv controller."""
import cv2
import numpy as np
from controller import Supervisor, Keyboard

robot = Supervisor()
keyboard = Keyboard()

# Set the time step
timestep = 30
keyboard.enable(timestep)

# Velocidade do robô UR5e
speed = 2
step_delta = 0.05  # incremento para movimentação das juntas

# Dispositivos dos motores do UR5e
ur_motors = [
    robot.getDevice('shoulder_pan_joint'),
    robot.getDevice('shoulder_lift_joint'),
    robot.getDevice('elbow_joint'),
    robot.getDevice('wrist_1_joint'),
    robot.getDevice('wrist_2_joint')
]

# Posições atuais (iniciadas como 0.0)
current_positions = [0.0] * len(ur_motors)

# Configura velocidade (sem mover posição inicial)
for motor in ur_motors:
    motor.setVelocity(speed)

# Inicializa câmera
camera = robot.getDevice('camera')
camera.enable(timestep)

# Inicializa display
display = robot.getDevice('display')
display.attachCamera(camera)
display.setColor(0x00FF00)
display.setFont('Verdana', 16, True)

def resetDisplay():
    display.setAlpha(0.0)
    display.fillRectangle(0, 0, 200, 150)
    display.setAlpha(1.0)

def printDisplay(x, y, w, h, name):
    resetDisplay()
    display.drawRectangle(x, y, w, h)
    display.drawText(name, x - 2, y - 20)

selected_joint = 0  # Junta padrão

print("Controles:\n- Teclas 1 a 5 para selecionar a junta\n- Setas ↑ e ↓ para mover a junta selecionada\n")

# Loop principal
while robot.step(timestep) != -1:
    # Captura da imagem (em RGBA)
    img = camera.getImage()
    if img:
        img_array = np.frombuffer(img, dtype=np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
        roi = img_array[0:150, 35:165, :3]  # pegar apenas RGB para evitar erro
        # Corrigir formato para Webots (RGB -> BGRA)
        roi_bgra = np.zeros((roi.shape[0], roi.shape[1], 4), dtype=np.uint8)
        roi_bgra[:, :, :3] = roi[:, :, ::-1]  # converte RGB para BGR
        roi_bgra[:, :, 3] = 255  # canal alfa

        # Corrigir tipo de ponteiro
        display.imagePaste(roi_bgra.tobytes(), 0, 0, False)

    # Leitura do teclado
    key = keyboard.getKey()
    while key != -1:
        if key == ord('1'):
            selected_joint = 0
            print("Junta selecionada: 1")
        elif key == ord('2'):
            selected_joint = 1
            print("Junta selecionada: 2")
        elif key == ord('3'):
            selected_joint = 2
            print("Junta selecionada: 3")
        elif key == ord('4'):
            selected_joint = 3
            print("Junta selecionada: 4")
        elif key == ord('5'):
            selected_joint = 4
            print("Junta selecionada: 5")
        elif key == Keyboard.UP:
            current_positions[selected_joint] += step_delta
            ur_motors[selected_joint].setPosition(current_positions[selected_joint])
            print(f"↑ Movimento da junta {selected_joint+1}: {current_positions[selected_joint]:.3f}")
        elif key == Keyboard.DOWN:
            current_positions[selected_joint] -= step_delta
            ur_motors[selected_joint].setPosition(current_positions[selected_joint])
            print(f"↓ Movimento da junta {selected_joint+1}: {current_positions[selected_joint]:.3f}")
        key = keyboard.getKey()
