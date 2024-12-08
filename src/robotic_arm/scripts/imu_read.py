#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import serial
import time
import math


class MPU6050Node:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        """
        Inicializa a comunicação serial e o nó ROS.
        :param port: Porta serial do Arduino.
        :param baudrate: Velocidade de comunicação serial.
        """
        self.serial_connection = serial.Serial(port, baudrate, timeout=1)
        self.publisher = rospy.Publisher("/imu_data", Imu, queue_size=10)
        rospy.init_node("mpu6050_node", anonymous=True)
        self.rate = rospy.Rate(10)  # Frequência de publicação (10 Hz)

    def parse_serial_data(self, data):
        """
        Converte os dados recebidos via serial para floats.
        :param data: Linha recebida da porta serial.
        :return: pitch, roll, yaw (em radianos).
        """
        try:
            # Exemplo de dado: "P:12.34, R:56.78, Y:90.12"
            parts = data.strip().split(",")
            pitch = float(parts[0].split(":")[1]) * math.pi / 180.0
            roll = float(parts[1].split(":")[1]) * math.pi / 180.0
            yaw = float(parts[2].split(":")[1]) * math.pi / 180.0
            return pitch, roll, yaw
        except (IndexError, ValueError):
            rospy.logwarn("Erro ao processar os dados: %s", data)
            return None, None, None

    def publish_imu_data(self, pitch, roll, yaw):
        """
        Publica os dados do MPU6050 como uma mensagem ROS de tipo Imu.
        :param pitch: Ângulo de inclinação (rad).
        :param roll: Ângulo de rolamento (rad).
        :param yaw: Ângulo de guinada (rad).
        """
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"

        # Definindo os valores de orientação (pitch, roll, yaw)
        imu_msg.orientation.x = roll
        imu_msg.orientation.y = pitch
        imu_msg.orientation.z = yaw
        imu_msg.orientation.w = 0.0  # Para quaternions, ajustar conforme necessário

        # Publica os dados no tópico
        self.publisher.publish(imu_msg)
        rospy.loginfo(f"Publicado IMU: pitch={pitch}, roll={roll}, yaw={yaw}")

    def run(self):
        """
        Loop principal para ler os dados da porta serial e publicar no ROS.
        """
        while not rospy.is_shutdown():
            try:
                if self.serial_connection.in_waiting > 0:
                    serial_data = self.serial_connection.readline().decode('utf-8')
                    pitch, roll, yaw = self.parse_serial_data(serial_data)

                    if pitch is not None and roll is not None and yaw is not None:
                        self.publish_imu_data(pitch, roll, yaw)

                self.rate.sleep()
            except serial.SerialException as e:
                rospy.logerr(f"Erro na comunicação serial: {e}")
                break
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logerr(f"Erro inesperado: {e}")


if __name__ == "__main__":
    try:
        # Altere o parâmetro "port" conforme necessário para o seu dispositivo
        mpu_node = MPU6050Node(port="/dev/ttyUSB0", baudrate=115200)
        mpu_node.run()
    except rospy.ROSInterruptException:
        pass
