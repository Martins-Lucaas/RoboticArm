"""fruit_sorting_ctrl_opencv controller."""
import cv2
import numpy as np
from controller import Supervisor

robot = Supervisor()

# Set the time step in 32
timestep = 30

# Initial position for the UR5e Robot
initial_positions = [1.28318, -3.143454353, 3.143245, 3.123544, 3.14]

# Speed of UR5e Robot
speed = 2

# Getting and declaring the robot motors
ur_motors = [
    robot.getDevice('shoulder_pan_joint'),
    robot.getDevice('shoulder_lift_joint'),
    robot.getDevice('elbow_joint'),
    robot.getDevice('wrist_1_joint'),
    robot.getDevice('wrist_2_joint')
]

# Set velocity of UR5e motors
for motor in ur_motors:
    motor.setVelocity(speed)

# Move the arm to the initial position and keep it locked
for i in range(len(ur_motors)):
    ur_motors[i].setPosition(initial_positions[i])

# Initialize camera
camera = robot.getDevice('camera')
camera.enable(timestep)

# Initialize display
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

# Main loop:
while robot.step(timestep) != -1:
    # Display camera feed (optional)
    img = np.frombuffer(camera.getImage(), dtype=np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    roi = img[0:150, 35:165]
    display.imagePaste(roi, 0, 0, False)  # Adicionado o argumento 'blend=False'

    pass