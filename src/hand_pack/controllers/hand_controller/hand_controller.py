# hand_controller.py  (ficando dentro de
#   share/hand_pack/controllers/hand_controller/hand_controller.py)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from controller import Robot, Motor

JOINT_NAMES = ['Thumb','Index','Middle','Ring','Little','Rotate']

class HandController(Node, Robot):
    def __init__(self):
        # inicializa ROS
        Node.__init__(self, 'hand_controller')
        # inicializa Webots
        Robot.__init__(self)
        self.timestep = int(self.getBasicTimeStep())
        # pega motores
        self.motors = {}
        for name in JOINT_NAMES:
            m = self.getDevice(name)
            m.setVelocity(1.0)
            m.setPosition(0.0)
            self.motors[name] = m
        # targets
        self.targets = {n: 0.0 for n in JOINT_NAMES}
        # subscriber ROS
        self.create_subscription(JointState,
                                 'hand/joint_states',
                                 self.cb_joint_state,
                                 10)

    def cb_joint_state(self, msg: JointState):
        for n, p in zip(msg.name, msg.position):
            if n in self.targets:
                self.targets[n] = p

    def run(self):
        while self.step(self.timestep) != -1:
            for name, m in self.motors.items():
                m.setPosition(self.targets[name])
            rclpy.spin_once(self, timeout_sec=0.0)

def main():
    rclpy.init()
    ctrl = HandController()
    ctrl.run()
    ctrl.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
