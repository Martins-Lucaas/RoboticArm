#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from pynput.keyboard import Listener

# Mapeamento das teclas para juntas
MAP_INPUT_TO_MOTOR = [
    ['w', 's'],  # BASE_UZ
    ['d', 'e'],  # SHOULDER_UY
    ['f', 'r'],  # ELBOW_UY
    ['g', 't'],  # WRIST_UY
    ['y', 'h'],  # WRIST_UZ
    ['u', 'j'],  # LEFT_GRIPPER_FINGER
    ['i', 'k']   # RIGHT_GRIPPER_FINGER
]

# Limites das juntas (radianos ou metros)
MIN_MAX = [
    [-1.57, 1.57],  # BASE_UZ
    [-1.0, 1.0],    # SHOULDER_UY
    [-1.57, 1.57],  # ELBOW_UY
    [0.0, 1.57],    # WRIST_UY
    [-1.57, 1.57],  # WRIST_UZ
    [0.0, 0.04],    # LEFT_GRIPPER_FINGER
    [0.0, 0.04]     # RIGHT_GRIPPER_FINGER
]

class ArmControlROS:

    def __init__(self):
        # Inicializa o nó ROS
        rospy.init_node('arm_control', anonymous=True)
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        # Estado inicial das juntas
        self.joint_positions = [0.0] * len(MAP_INPUT_TO_MOTOR)
        self.previous_positions = self.joint_positions.copy()
        self.joint_names = [
            "BASE_UZ_joint", "SHOULDER_UY_joint", "ELBOW_UY_joint",
            "WRIST_UY_joint", "WRIST_UZ_joint", "LEFT_GRIPPER_FINGER_joint",
            "RIGHT_GRIPPER_FINGER_joint"
        ]

    def publish_joint_states(self):
        """Publica os estados das juntas no tópico /joint_states apenas se houver mudanças."""
        if self.joint_positions != self.previous_positions:
            joint_state = JointState()
            joint_state.header = Header(stamp=rospy.Time.now())
            joint_state.name = self.joint_names
            joint_state.position = self.joint_positions.copy()  # Garantir que usamos uma cópia do estado atual

            self.pub.publish(joint_state)
            rospy.loginfo(f"Published joint states: {self.joint_positions}")

            # Atualiza as posições anteriores
            self.previous_positions = self.joint_positions.copy()


    def on_press(self, key):
        """Callback para tecla pressionada."""
        try:
            # Verifica se a tecla pressionada corresponde a um controle
            for joint_id, keys in enumerate(MAP_INPUT_TO_MOTOR):
                if key.char in keys:
                    direction = 1 if keys.index(key.char) == 0 else -1
                    increment = 0.01 if joint_id >= 5 else 0.05  # Incremento menor para grippers
                    new_position = self.joint_positions[joint_id] + direction * increment

                    # Limita os valores dentro dos limites definidos
                    new_position = max(new_position, MIN_MAX[joint_id][0])
                    new_position = min(new_position, MIN_MAX[joint_id][1])

                    # Atualiza apenas se houve mudança
                    if new_position != self.joint_positions[joint_id]:
                        self.joint_positions[joint_id] = new_position
                        rospy.loginfo(f"Joint {self.joint_names[joint_id]} moved to {self.joint_positions[joint_id]}")
                        self.publish_joint_states()  # Publica apenas quando há mudança
        except AttributeError:
            # Ignora teclas especiais
            rospy.loginfo(f"Special key pressed: {key}")

    def run(self):
        """Inicia o listener de teclado."""
        with Listener(on_press=self.on_press) as listener:
            rospy.loginfo("Listening for keyboard inputs...")
            listener.join()

if __name__ == '__main__':
    try:
        control = ArmControlROS()
        control.run()
    except rospy.ROSInterruptException:
        pass
