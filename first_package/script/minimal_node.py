#!/usr/bin/env python3

import rclpy  # ROS 2 Python client
from rclpy.node import Node  # Node class


def main(args=None):
    # 1. Initialize ROS communications for a given context
    rclpy.init(args=args)
    # 2. Instantiate a Node
    node = Node("echo_node_py")
    node.get_logger().info(f"Hello from {node.get_name()}")
    rclpy.spin(node)
    # 3. Shutdown a previously initialized context
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
