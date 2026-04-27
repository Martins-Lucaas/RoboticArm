import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tkinter as tk
from tkinter import ttk
import threading

class HandControlGUI(Node):
    def __init__(self):
        super().__init__('hand_custom_gui')
        self.publisher_ = self.create_publisher(JointTrajectory, '/hand_position_controller/joint_trajectory', 10)
        
        self.joint_names = [
            'Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Rotate',
            '_thumb_distal_j01', '_index_distal_j01', '_middle_distal_j01', 
            '_ring_distal_j01', '_little_distal_j01'
        ]
        
        self.root = tk.Tk()
        self.root.title("COVVI Hand Control")
        self.sliders = {}

        tk.Label(self.root, text="Controle Manual da Mão", font=('Arial', 12, 'bold')).pack(pady=10)

        tk.Label(self.root, text="Tempo de Movimento (s):").pack()
        self.time_slider = tk.Scale(self.root, from_=0.1, to=5.0, resolution=0.1, orient=tk.HORIZONTAL)
        self.time_slider.set(1.0)
        self.time_slider.pack(fill='x', padx=20) # Corrigido para padx

        ttk.Separator(self.root, orient='horizontal').pack(fill='x', pady=10)

        for joint in self.joint_names:
            frame = tk.Frame(self.root)
            frame.pack(fill='x', padx=20) # Corrigido para padx
            tk.Label(frame, text=f"{joint}:", width=15, anchor='w').pack(side=tk.LEFT)
            
            max_val = 3.2 if joint == 'Rotate' else 1.6
            s = tk.Scale(frame, from_=0.0, to=max_val, resolution=0.01, orient=tk.HORIZONTAL, 
                         command=lambda x: self.publish_position())
            s.pack(side=tk.LEFT, fill='x', expand=True)
            self.sliders[joint] = s

    def publish_position(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(self.sliders[name].get()) for name in self.joint_names]
        duration_sec = self.time_slider.get()
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        msg.points.append(point)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    gui = HandControlGUI()
    # Roda o ROS em uma thread separada para a interface não travar
    thread = threading.Thread(target=rclpy.spin, args=(gui,), daemon=True)
    thread.start()
    gui.root.mainloop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()