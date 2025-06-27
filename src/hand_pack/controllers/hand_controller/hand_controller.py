#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from controller import Robot

class HandController(Robot, Node):
    def __init__(self):
        # inicializa Webots Robot
        Robot.__init__(self)
        self.timestep = int(self.getBasicTimeStep())

        # inicializa ROS2
        rclpy.init(args=None)
        Node.__init__(self, 'hand_controller')

        # mapeamento joints ↔ sensors (mesmo que você já tinha)
        self.finger_joints = [
            ("Thumb",  "Thumb_sensor"),
            ("Index",  "Index_sensor"),
            ("Middle", "Middle_sensor"),
            ("Ring",   "Ring_sensor"),
            ("Little", "Little_sensor")
        ]

        # containers
        self.motors = {}
        self.joint_targets = {name: 0.0 for name,_ in self.finger_joints}

        # carrega devices
        for joint_name, sensor_name in self.finger_joints:
            m = self.getDevice(joint_name)
            s = self.getDevice(sensor_name)
            if not m or not s:
                self.get_logger().error(f"Motor ou sensor faltando: {joint_name}/{sensor_name}")
                continue
            m.setPosition(0.0)
            m.setVelocity(2.0)
            s.enable(self.timestep)
            self.motors[joint_name] = m

        # subscriber ROS2
        self.create_subscription(
            JointState,
            '/hand/joint_states',      # ajuste para o tópico que você usa
            self.joint_state_callback,
            10
        )

    def joint_state_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_targets:
                self.joint_targets[name] = pos

    def run(self):
        # loop principal Webots + ROS2
        while self.step(self.timestep) != -1 and rclpy.ok():
            for name, motor in self.motors.items():
                target = self.joint_targets[name]
                motor.setPosition(target)
            # processa callbacks ROS2 sem travar o loop
            rclpy.spin_once(self, timeout_sec=0.0)

        # limpeza
        self.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    controller = HandController()
    controller.run()
