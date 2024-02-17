#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    node = Node("minimal_py")
    node.get_logger().info(f"Hello from {node.get_name()}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Log a message when the node is manually terminated
        node.get_logger().warn("Keyboard interrupt detected")
    finally:
        # Shut down the ROS 2 Python client library
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()