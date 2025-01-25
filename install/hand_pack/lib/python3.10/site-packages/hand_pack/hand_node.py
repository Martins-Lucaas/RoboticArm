#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class HandNode(Node):
    def __init__(self):
        super().__init__('hand_node')
        self.get_logger().info('Hand Node está rodando!')

def main(args=None):
    rclpy.init(args=args)
    node = HandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
