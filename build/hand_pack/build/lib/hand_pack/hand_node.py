#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from controller import Robot, Motor

# nomes dos seus graus de liberdade/webots devices
JOINT_NAMES = ['Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Rotate']

class HandNode(Node):
    def __init__(self):
        super().__init__('hand_node')
        # inicializa Webots
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # prepara os motores
        self.motors = {}
        for j in JOINT_NAMES:
            m = self.robot.getDevice(j)
            m.setVelocity(1.0)    # configure aqui a velocidade máxima
            m.setPosition(0.0)    # posição inicial
            self.motors[j] = m

        # armazena os alvos vindos do ROS
        self.targets = {j: 0.0 for j in JOINT_NAMES}

        # subscription ROS2
        self.joint_sub = self.create_subscription(
            JointState,
            'hand/joint_states',  # ou outro tópico de sua escolha
            self.joint_state_callback,
            10
        )

        # loop de simulação
        self.create_timer(self.timestep/1000.0, self.step)

    def joint_state_callback(self, msg: JointState):
        # atualiza apenas as juntas que reconhecemos
        for name, pos in zip(msg.name, msg.position):
            if name in self.targets:
                self.targets[name] = pos

    def step(self):
        if self.robot.step(self.timestep) == -1:
            return

        # envia cada alvo para o motor correspondente
        for name, motor in self.motors.items():
            motor.setPosition(self.targets[name])

def main(args=None):
    rclpy.init(args=args)
    node = HandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
