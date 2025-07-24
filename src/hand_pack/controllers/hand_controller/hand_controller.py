from controller import Robot

robot = Robot()
TIME_STEP = 32

thumb = robot.getDevice("Thumb")
thumb.setPosition(1.0)

while robot.step(TIME_STEP) != -1:
    pass
